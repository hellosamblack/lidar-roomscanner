"""Server subpackage for the roomscan real-time web instrument."""
from __future__ import annotations

from . import app as _app_mod
from . import mdns as _mdns_mod
from . import routes as _routes_mod
from . import session as _session_mod
from . import ws_broadcaster as _ws_broadcaster_mod
from . import ws_thin as _ws_thin_mod

# Re-export everything from submodules (including private/internal helpers used by tests)
for _mod in (_session_mod, _ws_broadcaster_mod, _ws_thin_mod, _routes_mod, _mdns_mod, _app_mod):
    for _k, _v in vars(_mod).items():
        if not _k.startswith("__"):
            globals()[_k] = _v

__all__ = [k for k in globals() if not k.startswith("__")]
