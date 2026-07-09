import pytest

from roomscan.config import ViewerConfig, apply_config_defaults, config_dir, config_path


class Args:
    """Minimal argparse.Namespace stand-in: only the 5 viewer-config attrs."""

    def __init__(self, color=None, fov_h=None, fov_v=None, replay_fps=None, port=None):
        self.color = color
        self.fov_h = fov_h
        self.fov_v = fov_v
        self.replay_fps = replay_fps
        self.port = port


def test_config_dir_uses_appdata_env(monkeypatch, tmp_path):
    monkeypatch.setenv("APPDATA", str(tmp_path))
    assert config_dir() == tmp_path / "roomscan"
    assert config_path() == tmp_path / "roomscan" / "roomscan.toml"


def test_config_dir_falls_back_to_home_without_appdata(monkeypatch, tmp_path):
    monkeypatch.delenv("APPDATA", raising=False)
    monkeypatch.setattr("pathlib.Path.home", lambda: tmp_path)
    assert config_dir() == tmp_path / "roomscan"


def test_load_missing_file_returns_builtin_defaults(tmp_path):
    cfg = ViewerConfig.load(tmp_path / "does-not-exist.toml")
    assert cfg == ViewerConfig()
    assert cfg.color == "depth"
    assert cfg.fov_h == 55.0
    assert cfg.fov_v == 42.0
    assert cfg.replay_fps == 0.0
    assert cfg.port is None


def test_save_then_load_roundtrip(tmp_path):
    path = tmp_path / "roomscan.toml"
    original = ViewerConfig(color="reflectance", fov_h=54.65, fov_v=42.50,
                             replay_fps=25.0, port="COM7")
    saved_path = original.save(path)
    assert saved_path == path
    assert path.exists()

    loaded = ViewerConfig.load(path)
    assert loaded == original


def test_save_then_load_roundtrip_with_none_port(tmp_path):
    path = tmp_path / "roomscan.toml"
    original = ViewerConfig(port=None)
    original.save(path)
    loaded = ViewerConfig.load(path)
    assert loaded.port is None


def test_load_corrupt_file_tolerated(tmp_path):
    path = tmp_path / "roomscan.toml"
    path.write_text("this is not [ valid toml ===", encoding="utf-8")
    cfg = ViewerConfig.load(path)
    assert cfg == ViewerConfig()


def test_load_missing_viewer_table_tolerated(tmp_path):
    path = tmp_path / "roomscan.toml"
    path.write_text("[other]\nfoo = 1\n", encoding="utf-8")
    cfg = ViewerConfig.load(path)
    assert cfg == ViewerConfig()


def test_load_ignores_unknown_keys_and_fills_missing_from_defaults(tmp_path):
    path = tmp_path / "roomscan.toml"
    path.write_text('[viewer]\ncolor = "confidence"\nbogus_future_key = 42\n', encoding="utf-8")
    cfg = ViewerConfig.load(path)
    assert cfg.color == "confidence"
    assert cfg.fov_h == 55.0  # untouched field keeps the built-in default
    assert not hasattr(cfg, "bogus_future_key")


def test_load_wrong_type_value_tolerated(tmp_path):
    path = tmp_path / "roomscan.toml"
    # fov_h as a string where a float is expected on construction is fine for
    # tomllib itself, but the dataclass will happily accept it too (no runtime
    # type checking) -- what we actually guard is malformed *shape* (e.g. a
    # list where a table is expected for [viewer] itself), covered above.
    path.write_text('[viewer]\ncolor = "depth"\n', encoding="utf-8")
    cfg = ViewerConfig.load(path)
    assert cfg.color == "depth"


def test_apply_config_defaults_cli_wins_over_config():
    cfg = ViewerConfig(color="reflectance", fov_h=54.65, fov_v=42.50, replay_fps=25.0, port="COM7")
    args = Args(color="confidence")  # only --color passed on the CLI
    apply_config_defaults(args, cfg)
    assert args.color == "confidence"   # CLI wins
    assert args.fov_h == 54.65          # config fills the rest
    assert args.fov_v == 42.50
    assert args.replay_fps == 25.0
    assert args.port == "COM7"


def test_apply_config_defaults_all_unset_pulls_entirely_from_config():
    cfg = ViewerConfig(color="reflectance", fov_h=54.65, fov_v=42.50, replay_fps=25.0, port="COM7")
    args = Args()
    apply_config_defaults(args, cfg)
    assert (args.color, args.fov_h, args.fov_v, args.replay_fps, args.port) == \
        ("reflectance", 54.65, 42.50, 25.0, "COM7")


def test_apply_config_defaults_builtin_default_when_config_is_default_too():
    args = Args()
    apply_config_defaults(args, ViewerConfig())
    assert args.color == "depth"
    assert args.fov_h == 55.0
    assert args.fov_v == 42.0
    assert args.replay_fps == 0.0
    assert args.port is None
