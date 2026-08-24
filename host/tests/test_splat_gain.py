"""Tests for cross-frame RGB gain compensation (Issue #160).

Covers:
- SplatPreset / SplatConfig defaults and fingerprint backward-compatibility.
- Zero-mean gauge constraint across views.
- Clamping of centered log-gains to +/- max_stops.
- Photometric loss scaling with multiplicative gain.
- Separate optimizer stepping and learning gain on synthetic multi-view data.
- CLI argument parsing and overrides for RGB gain options.
"""
import math
import torch

from roomscan.splat import SplatConfig, SplatPreset
from roomscan.splat.cli import _PRESET_OVERRIDES, main
from roomscan.splat.train import (
    _gaussian_window,
    _ssim,
    compute_centered_log_gains,
)


def test_splat_preset_gain_defaults():
    preset = SplatPreset()
    assert preset.rgb_gain_compensation is False
    assert math.isclose(preset.rgb_gain_lr, 1e-2, rel_tol=1e-6)
    assert math.isclose(preset.rgb_gain_regularization, 1e-3, rel_tol=1e-6)
    assert math.isclose(preset.rgb_gain_max_stops, 1.0, rel_tol=1e-6)

    # SplatConfig alias
    assert SplatConfig is SplatPreset
    cfg = SplatConfig()
    assert cfg.rgb_gain_compensation is False


def test_fingerprint_backward_compatibility_when_disabled():
    # Historical baseline fingerprint without RGB gain compensation
    baseline_fp = "778c7afccec75718"
    default_preset = SplatPreset()
    assert default_preset.fingerprint() == baseline_fp

    # When rgb_gain_compensation is False, tuning any rgb_gain_* parameter must
    # NOT alter the fingerprint so existing reconstruction caches are preserved.
    tuned_when_disabled = SplatPreset(
        rgb_gain_compensation=False,
        rgb_gain_lr=0.05,
        rgb_gain_regularization=0.1,
        rgb_gain_max_stops=2.5,
    )
    assert tuned_when_disabled.fingerprint() == baseline_fp


def test_fingerprint_changes_when_enabled_or_modified():
    baseline_fp = "778c7afccec75718"
    enabled_preset = SplatPreset(rgb_gain_compensation=True)
    enabled_fp = enabled_preset.fingerprint()

    # Enabling compensation must change the fingerprint
    assert enabled_fp != baseline_fp

    # Modifying parameters when enabled must produce distinct fingerprints
    fp_diff_lr = SplatPreset(rgb_gain_compensation=True, rgb_gain_lr=0.05).fingerprint()
    fp_diff_reg = SplatPreset(rgb_gain_compensation=True, rgb_gain_regularization=0.01).fingerprint()
    fp_diff_stops = SplatPreset(rgb_gain_compensation=True, rgb_gain_max_stops=2.0).fingerprint()

    assert fp_diff_lr != enabled_fp
    assert fp_diff_reg != enabled_fp
    assert fp_diff_stops != enabled_fp
    assert fp_diff_lr != fp_diff_reg


def test_gauge_constraint_mean_log_gain_is_zero():
    torch.manual_seed(42)
    # Test across multiple view counts
    for num_views in (1, 4, 15, 100):
        raw_log_gains = torch.randn(num_views, 3) * 0.5
        centered, gains = compute_centered_log_gains(raw_log_gains, max_stops=2.0)

        # Centered log-gains must have zero mean per channel across all views
        channel_means = centered.mean(dim=0)
        assert torch.allclose(channel_means, torch.zeros(3), atol=1e-6), (
            f"Expected zero mean for num_views={num_views}, got {channel_means}"
        )

        # In un-clamped cases, geometric mean of gains across views is exactly 1.0
        # prod(gains_c)^(1/N) = exp(mean(centered_c)) = exp(0) = 1.0
        geom_mean = torch.exp(centered.mean(dim=0))
        assert torch.allclose(geom_mean, torch.ones(3), atol=1e-6)


def test_gain_clamping_to_max_stops():
    # max_stops = 1.0 -> +/-1 stop -> range [0.5, 2.0]
    raw_log_gains = torch.tensor([
        [-10.0, -10.0, -10.0],  # strongly negative
        [0.0, 0.0, 0.0],        # zero
        [10.0, 10.0, 10.0],     # strongly positive
    ])

    centered, gains = compute_centered_log_gains(raw_log_gains, max_stops=1.0)
    max_log = 1.0 * math.log(2.0)
    # Effective clamped log-gain bounds
    effective_log_gains = torch.log(gains)
    assert float(effective_log_gains.min()) >= -max_log - 1e-6
    assert float(effective_log_gains.max()) <= max_log + 1e-6

    # Gain multiplicative bounds: [0.5, 2.0]
    assert math.isclose(float(gains.min()), 0.5, rel_tol=1e-5)
    assert math.isclose(float(gains.max()), 2.0, rel_tol=1e-5)

    # Test max_stops = 2.0 -> +/-2 stops -> range [0.25, 4.0]
    _, gains_2stops = compute_centered_log_gains(raw_log_gains, max_stops=2.0)
    assert math.isclose(float(gains_2stops.min()), 0.25, rel_tol=1e-5)
    assert math.isclose(float(gains_2stops.max()), 4.0, rel_tol=1e-5)

    # Test max_stops = 0.5 -> +/-0.5 stop -> range [2^-0.5, 2^0.5]
    _, gains_half = compute_centered_log_gains(raw_log_gains, max_stops=0.5)
    assert math.isclose(float(gains_half.min()), 2.0 ** (-0.5), rel_tol=1e-5)
    assert math.isclose(float(gains_half.max()), 2.0 ** 0.5, rel_tol=1e-5)


def test_photometric_loss_scaling_with_gain():
    torch.manual_seed(0)
    # Synthetic render and ground truth
    pred = torch.full((16, 16, 3), 0.4)
    # Target image is 1.5x brighter in red, 1.2x in green, 0.8x in blue
    gt_scale = torch.tensor([1.5, 1.2, 0.8])
    gt = pred * gt_scale

    # 1) Without gain compensation (gain = [1, 1, 1]): non-zero L1 loss
    unscaled_l1 = (pred - gt).abs().mean().item()
    expected_l1 = (pred * (1.0 - gt_scale)).abs().mean().item()
    assert math.isclose(unscaled_l1, expected_l1, rel_tol=1e-5)
    assert unscaled_l1 > 0.05

    # 2) With exact matching gain: L1 loss becomes 0
    exact_gain = gt_scale
    pred_scaled = pred * exact_gain
    scaled_l1 = (pred_scaled - gt).abs().mean().item()
    assert math.isclose(scaled_l1, 0.0, abs_tol=1e-7)

    # 3) Check gradient propagation back to log_gains
    log_gains = torch.zeros((2, 3), requires_grad=True)
    centered, gains = compute_centered_log_gains(log_gains, max_stops=1.0)
    loss = (pred * gains[0] - gt).abs().mean()
    loss.backward()

    assert log_gains.grad is not None
    # For view 0, gt is brighter in red (1.5x) and dimmer in blue (0.8x)
    # Increasing log_gain[0, 0] increases pred_scaled, reducing loss -> negative grad
    # Increasing log_gain[0, 2] increases pred_scaled away from gt -> positive grad
    assert float(log_gains.grad[0, 0]) < 0.0
    assert float(log_gains.grad[0, 2]) > 0.0


def test_separate_optimizer_stepping_and_learning_gain():
    torch.manual_seed(123)
    num_views = 3
    h, w = 24, 24

    # Base synthetic scene
    base_render = torch.rand((h, w, 3)) * 0.6 + 0.2

    # Target exposure scale factors for the 3 views:
    # View 0: darker (0.7x)
    # View 1: nominal (1.0x)
    # View 2: brighter (1.3x)
    true_factors = [
        torch.tensor([0.7, 0.7, 0.7]),
        torch.tensor([1.0, 1.0, 1.0]),
        torch.tensor([1.3, 1.3, 1.3]),
    ]
    gts = [base_render * f for f in true_factors]

    # Initialize log_gains at 0
    log_gains = torch.zeros((num_views, 3), requires_grad=True)
    optimizer = torch.optim.Adam([log_gains], lr=0.05)
    window = _gaussian_window(3, torch.device("cpu"))

    def compute_total_loss():
        centered, gains = compute_centered_log_gains(log_gains, max_stops=1.0)
        tot = 0.0
        for i in range(num_views):
            pred_i = base_render * gains[i]
            l1 = (pred_i - gts[i]).abs().mean()
            ssim_val = _ssim(pred_i.permute(2, 0, 1)[None], gts[i].permute(2, 0, 1)[None], window)
            loss_i = 0.8 * l1 + 0.2 * (1.0 - ssim_val)
            tot = tot + loss_i
        tot = tot / num_views + 1e-3 * (centered ** 2).mean()
        return tot, centered, gains

    initial_loss, _, _ = compute_total_loss()

    # Train for 80 steps
    for _ in range(80):
        optimizer.zero_grad()
        loss, centered, _ = compute_total_loss()
        loss.backward()
        optimizer.step()

    final_loss, final_centered, final_gains = compute_total_loss()

    # Loss must drop dramatically (>85% reduction)
    assert final_loss.item() < initial_loss.item() * 0.15

    # Gauge constraint must hold on centered log gains
    assert torch.allclose(final_centered.mean(dim=0), torch.zeros(3), atol=1e-5)

    # Ratios between learned view gains must match ratios between true exposures
    # View 0 (0.7x) should have lowest gain, View 2 (1.3x) highest
    g0 = float(final_gains[0].detach().mean())
    g1 = float(final_gains[1].detach().mean())
    g2 = float(final_gains[2].detach().mean())

    assert g0 < g1 < g2
    # Check relative ratio g2 / g0 is close to 1.3 / 0.7 ~ 1.857 (within 10%)
    assert math.isclose(g2 / g0, 1.3 / 0.7, rel_tol=0.10)


def test_cli_argument_parsing(monkeypatch):
    assert "rgb_gain_compensation" in _PRESET_OVERRIDES
    assert "rgb_gain_lr" in _PRESET_OVERRIDES
    assert "rgb_gain_regularization" in _PRESET_OVERRIDES
    assert "rgb_gain_max_stops" in _PRESET_OVERRIDES

    # Test parser flags
    captured_preset = None

    def fake_build_splat(video, name, results_dir, *, preset, **kwargs):
        nonlocal captured_preset
        captured_preset = preset
        return {"ok": True, "slug": "test", "stats": {"gaussians": 10}}

    monkeypatch.setattr("roomscan.splat.pipeline.build_splat", fake_build_splat)

    cmd = [
        "build", "dummy.mp4", "--name", "Test",
        "--rgb-gain-compensation",
        "--rgb-gain-lr", "0.02",
        "--rgb-gain-regularization", "0.005",
        "--rgb-gain-max-stops", "1.5",
    ]
    ret = main(cmd)
    assert ret == 0
    assert captured_preset is not None
    assert captured_preset.rgb_gain_compensation is True
    assert math.isclose(captured_preset.rgb_gain_lr, 0.02, rel_tol=1e-6)
    assert math.isclose(captured_preset.rgb_gain_regularization, 0.005, rel_tol=1e-6)
    assert math.isclose(captured_preset.rgb_gain_max_stops, 1.5, rel_tol=1e-6)
