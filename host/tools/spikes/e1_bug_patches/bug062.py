"""BUG-062: the live path hand-picked five of SlamConfig's eighteen Mapper
knobs, so [slam] keys silently did not reach Live SLAM. NOT REJECTED: forwarding
a subset of keyword arguments is legal; Pyright has no exhaustiveness check for
field forwarding (the fix was `SlamConfig.mapper_kwargs()` discipline)."""
from roomscan.slam.config import SlamConfig
from roomscan.slam.mapper import Mapper


def make_mapper(cfg: SlamConfig) -> Mapper:
    return Mapper(54, 42, icp_mode=cfg.icp_mode, voxel_size=cfg.voxel_size)
