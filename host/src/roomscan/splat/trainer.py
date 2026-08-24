"""Alias for roomscan.splat.train (Issue #160)."""
from .train import *  # noqa: F403
from .train import (  # explicit re-exports
    SH_C0,
    compute_centered_log_gains,
    train_splat,
)

__all__ = ["train_splat", "compute_centered_log_gains", "SH_C0"]
