"""Structure-from-motion over the extracted frames via ``pycolmap``.

Produces per-frame camera poses + a sparse point cloud -- the input a 3DGS
trainer needs.  Runs entirely on CPU (the pip ``pycolmap`` wheel has no CUDA);
for a few-hundred downscaled frames that is minutes, not hours.  No system COLMAP
binary required.

``run_sfm`` is the single implementation behind both the build pipeline and
``splat_sfm_probe``: the probe sweeps the same tuning knobs (matcher, feature
count, overlap) so a winning config behaves identically in a real build.  The
returned stats expose sub-model FRAGMENTATION -- COLMAP can split a walkthrough
into several disconnected reconstructions and this function keeps only the
largest; a low ``largest_ratio`` is why a splat is sparse even when the card has
VRAM to spare.
"""
from __future__ import annotations

from pathlib import Path


class SfmError(RuntimeError):
    """SfM failed to register a usable model (e.g. textureless / too little overlap)."""


def run_sfm(image_dir: str | Path, work_dir: str | Path, *, matcher: str = "sequential",
            sequential_overlap: int = 10, min_registered: int = 8,
            max_num_features: int = 8192, estimate_affine_shape: bool = False,
            domain_size_pooling: bool = False, loop_detection: bool = False,
            exhaustive_max_images: int = 60, log=lambda m: None) -> dict:
    """Extract features, match, and incrementally map ``image_dir``.

    Writes the winning sparse model to ``work_dir/sparse`` and returns a stats
    dict (registered/total images, sub-model fragmentation, point count, mean
    track length, model dir).  Raises :class:`SfmError` if too few frames register
    to be worth training on.

    Tuning knobs (all exposed so ``splat_sfm_probe`` can sweep the real path):
    ``matcher`` ``sequential``/``exhaustive``; ``exhaustive_max_images`` is the
    frame count below which ``sequential`` auto-escalates to exhaustive (cheap and
    more robust when frames are few); ``max_num_features`` /
    ``estimate_affine_shape`` / ``domain_size_pooling`` are SIFT knobs that help on
    textureless indoor walls; ``loop_detection`` adds vocab-tree loop pairs to the
    sequential matcher (reconnects revisited areas -- needs a bundled vocab tree,
    and raises if unavailable).
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
    log(f"[sfm] extracting SIFT features ({n_images} images, CPU, "
        f"max_features={max_num_features})")
    feo = pycolmap.FeatureExtractionOptions()
    feo.sift.max_num_features = max_num_features
    feo.sift.estimate_affine_shape = estimate_affine_shape
    feo.sift.domain_size_pooling = domain_size_pooling
    pycolmap.extract_features(db_path, image_dir, extraction_options=feo,
                              device=pycolmap.Device.cpu)

    use = matcher
    if use == "sequential" and n_images <= exhaustive_max_images:
        use = "exhaustive"   # cheap enough, and more robust when frames are few
    log(f"[sfm] matching ({use}{', +loop' if loop_detection and use == 'sequential' else ''})")
    if use == "exhaustive":
        pycolmap.match_exhaustive(db_path, device=pycolmap.Device.cpu)
    else:
        pairing = pycolmap.SequentialPairingOptions()
        pairing.overlap = sequential_overlap
        pairing.quadratic_overlap = True
        if loop_detection:
            pairing.loop_detection = True   # vocab-tree loop closure (reconnects revisits)
        pycolmap.match_sequential(db_path, pairing_options=pairing,
                                  device=pycolmap.Device.cpu)

    log("[sfm] incremental mapping")
    recons = pycolmap.incremental_mapping(db_path, image_dir, sparse_dir)
    if not recons:
        raise SfmError("COLMAP registered no images -- the scene may be too "
                       "textureless or the frames may not overlap enough. Try a "
                       "higher --fps or a slower pan.")

    # Fragmentation: COLMAP returns one Reconstruction per disconnected sub-model.
    # We train on the largest, so `largest_ratio` (not `registered_ratio`) is the
    # real usable-frame fraction -- if it is low, better matching (exhaustive /
    # loop closure / denser frames) will register more than a bigger VRAM cap ever will.
    submodel_sizes = sorted((r.num_reg_images() for r in recons.values()), reverse=True)
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
        # registered_ratio == largest_ratio: we train on the largest sub-model, so
        # this is the usable-frame fraction. total_placed_ratio counts frames COLMAP
        # placed in ANY sub-model -- if it is much higher than largest_ratio the loss
        # is FRAGMENTATION (better matching merges them), not failed registration.
        "registered_ratio": round(registered / max(1, n_images), 3),
        "largest_ratio": round(registered / max(1, n_images), 3),
        "total_placed_ratio": round(sum(submodel_sizes) / max(1, n_images), 3),
        "n_submodels": len(recons),
        "submodel_sizes": submodel_sizes,
        "points3D": rec.num_points3D(),
        "mean_track_length": round(rec.compute_mean_track_length(), 2),
        "mean_reprojection_error": round(rec.compute_mean_reprojection_error(), 3),
        "model_dir": str(model_dir),
    }
    log(f"[sfm] registered {registered}/{n_images} frames in {len(recons)} sub-model(s) "
        f"{submodel_sizes}, {stats['points3D']} points, mean track {stats['mean_track_length']}")
    return stats
