"""Structure-from-motion over the extracted frames via ``pycolmap``.

Produces per-frame camera poses + a sparse point cloud -- the input a 3DGS
trainer needs.  Runs entirely on CPU (the pip ``pycolmap`` wheel has no CUDA);
for a few-hundred downscaled frames that is minutes, not hours.  No system COLMAP
binary required.
"""
from __future__ import annotations

from pathlib import Path


class SfmError(RuntimeError):
    """SfM failed to register a usable model (e.g. textureless / too little overlap)."""


def run_sfm(image_dir: str | Path, work_dir: str | Path, *, matcher: str = "sequential",
            sequential_overlap: int = 10, min_registered: int = 8,
            log=lambda m: None) -> dict:
    """Extract features, match, and incrementally map ``image_dir``.

    Writes the winning sparse model to ``work_dir/sparse`` and returns a stats
    dict (registered/total images, point count, mean track length, model dir).
    Raises :class:`SfmError` if too few frames register to be worth training on.
    """
    import pycolmap

    image_dir = Path(image_dir)
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    db_path = work_dir / "database.db"
    sparse_dir = work_dir / "sparse"
    sparse_dir.mkdir(exist_ok=True)
    if db_path.exists():
        db_path.unlink()

    n_images = len(list(image_dir.glob("*.jpg")))
    log(f"[sfm] extracting SIFT features ({n_images} images, CPU)")
    pycolmap.extract_features(db_path, image_dir, device=pycolmap.Device.cpu)

    use = matcher
    if use == "sequential" and n_images <= 60:
        use = "exhaustive"   # cheap enough, and more robust when frames are few
    log(f"[sfm] matching ({use})")
    if use == "exhaustive":
        pycolmap.match_exhaustive(db_path, device=pycolmap.Device.cpu)
    else:
        pairing = pycolmap.SequentialPairingOptions()
        pairing.overlap = sequential_overlap
        pairing.quadratic_overlap = True
        pycolmap.match_sequential(db_path, pairing_options=pairing,
                                  device=pycolmap.Device.cpu)

    log("[sfm] incremental mapping")
    recons = pycolmap.incremental_mapping(db_path, image_dir, sparse_dir)
    if not recons:
        raise SfmError("COLMAP registered no images -- the scene may be too "
                       "textureless or the frames may not overlap enough. Try a "
                       "higher --fps or a slower pan.")

    best_idx = max(recons, key=lambda i: recons[i].num_reg_images())
    rec = recons[best_idx]
    model_dir = sparse_dir / str(best_idx)
    registered = rec.num_reg_images()
    if registered < min_registered:
        raise SfmError(f"only {registered} of {n_images} frames registered "
                       f"(need >= {min_registered}); reconstruction would be unusable.")

    stats = {
        "images_total": n_images,
        "images_registered": registered,
        "registered_ratio": round(registered / max(1, n_images), 3),
        "points3D": rec.num_points3D(),
        "mean_track_length": round(rec.compute_mean_track_length(), 2),
        "mean_reprojection_error": round(rec.compute_mean_reprojection_error(), 3),
        "model_dir": str(model_dir),
    }
    log(f"[sfm] registered {registered}/{n_images} frames, "
        f"{stats['points3D']} points, mean track {stats['mean_track_length']}")
    return stats
