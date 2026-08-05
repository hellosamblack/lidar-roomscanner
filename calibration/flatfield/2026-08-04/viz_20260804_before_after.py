"""Before/after flat-field correction, numerically + visually.

Uses the HELD-OUT cross-room case (a map built in the first room, applied to the
larger unseen room, wall-rejected) — the honest deployment performance, not the
self-scored best case. Reuses `_build` from the cross-room analysis so the wall
rejection and native transform are identical.

Figure `beforeafter_20260804.png`, per family (Precision, Ambient):
  col 1  FPN before  (reflectance / smooth - 1, %)   -- the sensor's fixed grid
  col 2  FPN after   (corrected the same way)        -- grid removed
  col 3  column-averaged ripple profile, before vs after
  col 4  per-zone residual histogram, before vs after
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
for p in (ROOT / "host" / "src", ROOT / "host", HERE):
    sys.path.insert(0, str(p))

from analyze_20260804_crossroom import _build, ROOM_A, ROOM_B  # noqa: E402
from roomscan.flatfield import _blur  # noqa: E402


def detrend_pct(img: np.ndarray) -> np.ndarray:
    """High-frequency reflectance residual in %: what flat-field targets."""
    illum = _blur(img, 2.5)
    good = np.isfinite(img) & np.isfinite(illum) & (illum > 1e-6)
    out = np.full(img.shape, np.nan)
    out[good] = 100.0 * (img[good] / illum[good] - 1.0)
    return out


def main() -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    A = {fam: _build(name, "roomA") for fam, name in ROOM_A.items()}
    B = {fam: _build(name, "roomB") for fam, name in ROOM_B.items()}

    rows = []
    for fam in ("precision", "ambient"):
        before_img = B[fam]["mean"]
        after_img = B[fam]["mean"] * A[fam]["gain"]      # held-out: room-A map
        rb, ra = detrend_pct(before_img), detrend_pct(after_img)
        rows.append((fam, rb, ra,
                     float(np.nanstd(rb)), float(np.nanstd(ra))))

    fig, axes = plt.subplots(2, 4, figsize=(17, 7.4), constrained_layout=True)
    LIM = 15.0
    for i, (fam, rb, ra, sb, sa) in enumerate(rows):
        ax = axes[i]
        for j, (img, ttl, s) in enumerate((
                (rb, f"BEFORE  (raw FPN {sb:.1f}%)", sb),
                (ra, f"AFTER  (corrected {sa:.1f}%)", sa))):
            im = ax[j].imshow(img, cmap="RdBu_r", vmin=-LIM, vmax=LIM, interpolation="nearest")
            ax[j].set_title(ttl, fontsize=10)
            ax[j].set_xlabel("zone column"); ax[j].set_ylabel("zone row")
            fig.colorbar(im, ax=ax[j], shrink=0.82, label="reflectance / smooth − 1 (%)")
        # column-averaged ripple profile
        ax[2].plot(np.nanmean(rb, axis=0), color="#c0392b", lw=1.4, label=f"before ({sb:.1f}%)")
        ax[2].plot(np.nanmean(ra, axis=0), color="#2471a3", lw=1.4, label=f"after ({sa:.1f}%)")
        ax[2].axhline(0, color="0.6", lw=0.7)
        ax[2].set_title("column-averaged ripple", fontsize=10)
        ax[2].set_xlabel("zone column"); ax[2].set_ylabel("mean residual (%)")
        ax[2].legend(fontsize=8, loc="upper right")
        # histogram
        bins = np.linspace(-LIM, LIM, 61)
        ax[3].hist(rb[np.isfinite(rb)], bins=bins, color="#c0392b", alpha=0.55,
                   label=f"before σ={sb:.1f}%")
        ax[3].hist(ra[np.isfinite(ra)], bins=bins, color="#2471a3", alpha=0.55,
                   label=f"after σ={sa:.1f}%")
        ax[3].set_title("per-zone residual distribution", fontsize=10)
        ax[3].set_xlabel("residual (%)"); ax[3].set_ylabel("zones")
        ax[3].legend(fontsize=8, loc="upper right")
        ax[0].annotate(fam.upper(), xy=(-0.32, 0.5), xycoords="axes fraction",
                       rotation=90, fontsize=13, fontweight="bold", va="center")

    fig.suptitle("Flat-field before/after — HELD-OUT (map from a different room, wall-rejected)",
                 fontsize=13)
    fig.savefig(HERE / "beforeafter_20260804.png", dpi=150)
    plt.close(fig)

    print("\n=== BEFORE / AFTER flat-field (per-zone reflectance FPN, %) ===")
    print(f"{'family':10} {'raw':>7} {'held-out':>10} {'reduction':>10}   "
          f"{'self-map':>9} {'reduction':>10}")
    for fam in ("precision", "ambient"):
        raw = B[fam]["raw_fpn"]; held = None
        # recompute held-out + self for the table
        ho = float(np.nanstd(detrend_pct(B[fam]["mean"] * A[fam]["gain"])))
        self_ = B[fam]["self_fpn"]
        print(f"{fam:10} {raw:6.2f}% {ho:9.2f}% {100*(raw-ho)/raw:9.0f}%   "
              f"{self_:8.2f}% {100*(raw-self_)/raw:9.0f}%")
    print("\nheld-out = map from a DIFFERENT room (deployment-realistic)")
    print("self-map = map from this same capture (optimistic upper bound)")
    print("wrote beforeafter_20260804.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
