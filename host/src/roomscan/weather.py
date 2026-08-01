"""Sea-level reference pressure for the Sensors card's elevation readout.

The barometer on the IKS4A1 measures *absolute* pressure. Turning that into an
elevation needs a reference: the pressure the atmosphere would have at sea level
right now, at this location. That number moves by tens of hPa with the weather --
a fixed 101325 Pa reference is wrong by up to ~300 m -- so it is fetched from
Open-Meteo (free, no API key) once every `refresh_s` (30 min by default) and
cached.

**This is the app's first and only outbound third-party request.** Everything
else `roomscan-web` does is local. It is deliberately:

* stdlib `urllib.request` only, run inside `asyncio.to_thread` -- the existing
  precedent (`mcp_server/tools_rig.py`, `mcp_server/session.py`,
  `tools/web_ui_shot.py`); one GET per 30 minutes does not earn a dependency;
* non-fatal in every failure mode. A box with no internet (this rig is often
  behind a battery Wi-Fi bridge with no uplink) falls back to 101325.0 Pa and
  **says so** on the wire (`msl_source`), rather than silently reporting an
  elevation that is off by a couple of hundred feet;
* never able to raise into the broadcaster. `maybe_refresh()` schedules a task
  that swallows everything; the getters are pure attribute reads.

The parse is split out as a pure `parse_msl_pressure(json_text)` so the wire
format can be tested offline with no network at all (see test_weather.py).
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
import urllib.parse
import urllib.request

log = logging.getLogger(__name__)

#: ICAO standard atmosphere sea-level pressure. The documented fallback -- and
#: also what `baro_height_m` degenerates to if nothing is ever fetched.
FALLBACK_MSL_PA = 101325.0

API_URL = "https://api.open-meteo.com/v1/forecast"

#: Sanity window for a returned mean-sea-level pressure, in hPa. The world
#: record extremes are ~870 (typhoon Tip) and ~1084 (Siberia), so anything
#: outside this is a parse error or a wrong field, not weather.
_MSL_HPA_RANGE = (850.0, 1100.0)

#: Don't hammer the API after a failure: a box with no uplink would otherwise
#: retry on every `maybe_refresh()` call (i.e. at the sensor cadence).
_RETRY_AFTER_FAIL_S = 120.0

#: A cached value older than `refresh_s * _STALE_FACTOR` is reported as
#: "stale" rather than "api" -- i.e. refreshes have been failing for a while,
#: and the reading is drifting with unmeasured weather.
_STALE_FACTOR = 2.0

_TIMEOUT_S = 6.0


def build_url(latitude: float, longitude: float) -> str:
    """The Open-Meteo current-conditions URL for one point.

    `surface_pressure` is requested alongside `pressure_msl` purely so the
    response is self-describing when someone curls it by hand -- only
    `pressure_msl` is parsed.
    """
    query = urllib.parse.urlencode({
        "latitude": f"{float(latitude):.6f}",
        "longitude": f"{float(longitude):.6f}",
        "current": "surface_pressure,pressure_msl",
        "timezone": "auto",
    })
    return f"{API_URL}?{query}"


def parse_msl_pressure(json_text: str) -> float:
    """Open-Meteo response body -> mean-sea-level pressure in **pascals**.

    Pure: no network, no clock, no state. Raises `ValueError` on anything that
    is not a usable reading -- malformed JSON, a missing `current` object, a
    missing/non-numeric `pressure_msl`, or a value outside `_MSL_HPA_RANGE`.
    The caller treats any `ValueError` as "no reading", which is why the range
    check lives here: a plausible-looking 0.0 or a value in the wrong unit
    would otherwise sail through and put the elevation kilometres off.
    """
    try:
        data = json.loads(json_text)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"not JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"expected a JSON object, got {type(data).__name__}")
    current = data.get("current")
    if not isinstance(current, dict):
        raise ValueError("response has no `current` object")
    raw = current.get("pressure_msl")
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise ValueError(f"`current.pressure_msl` is not a number: {raw!r}")
    hpa = float(raw)
    lo, hi = _MSL_HPA_RANGE
    if not (lo <= hpa <= hi):
        raise ValueError(f"`current.pressure_msl` = {hpa} hPa is outside {lo}-{hi}")
    return hpa * 100.0


def fetch_msl_pressure(latitude: float, longitude: float, *, timeout: float = _TIMEOUT_S,
                       opener=None) -> float:
    """Blocking GET + parse -> pascals. Raises on any failure.

    Never call this on the event loop; `MslPressure` runs it in a thread.
    `opener` is injectable for tests (called with `(url, timeout)` -> text).
    """
    url = build_url(latitude, longitude)
    if opener is not None:
        return parse_msl_pressure(opener(url, timeout))
    with urllib.request.urlopen(url, timeout=timeout) as resp:   # noqa: S310 (fixed https host)
        body = resp.read().decode("utf-8", "replace")
    return parse_msl_pressure(body)


class MslPressure:
    """Cached sea-level reference pressure, refreshed in the background.

    Read side (`msl_pa` / `source` / `age_s` / `snapshot`) is a plain attribute
    read, safe to call from the broadcaster every tick. Write side is
    `maybe_refresh()`, which schedules at most one in-flight `asyncio` task and
    returns immediately.

    `source` is deliberately on the wire:
      * `"api"`   -- a real reading, fetched within `refresh_s * 2`
      * `"stale"` -- we have a reading, but refreshes have been failing
      * `"fallback"` -- never got one; the elevation is against 101325 Pa
    """

    def __init__(self, latitude: float, longitude: float, *, refresh_s: float = 1800.0,
                 fetch=None, clock=time.monotonic):
        self.latitude = float(latitude)
        self.longitude = float(longitude)
        self.refresh_s = max(60.0, float(refresh_s))
        self._fetch = fetch if fetch is not None else fetch_msl_pressure
        self._clock = clock
        self._value: float | None = None
        self._fetched_at: float | None = None
        self._last_attempt: float | None = None
        self._task = None

    # ---- read side (broadcaster; never blocks, never raises) ----------------

    @property
    def msl_pa(self) -> float:
        return self._value if self._value is not None else FALLBACK_MSL_PA

    @property
    def age_s(self) -> float | None:
        if self._fetched_at is None:
            return None
        return max(0.0, self._clock() - self._fetched_at)

    @property
    def source(self) -> str:
        age = self.age_s
        if self._value is None or age is None:
            return "fallback"
        return "api" if age <= self.refresh_s * _STALE_FACTOR else "stale"

    def snapshot(self) -> dict:
        """The three wire fields, as `build_sensor_message` wants them."""
        age = self.age_s
        return {"msl_pa": round(self.msl_pa, 1),
                "msl_source": self.source,
                "msl_age_s": None if age is None else round(age, 1)}

    # ---- write side ---------------------------------------------------------

    def due(self) -> bool:
        """Whether a refresh should be attempted now. False while a recent
        attempt (successful or not) is still inside its cooldown."""
        now = self._clock()
        if self._last_attempt is None:
            return True
        if self._value is None:
            return (now - self._last_attempt) >= _RETRY_AFTER_FAIL_S
        return (now - self._last_attempt) >= self.refresh_s

    def refresh_now(self) -> bool:
        """Blocking refresh. Returns True on success. Never raises -- a failure
        just leaves the previous value (and moves `source` toward stale)."""
        self._last_attempt = self._clock()
        try:
            value = float(self._fetch(self.latitude, self.longitude))
        except Exception as exc:                        # network, DNS, parse, ...
            log.warning("sea-level pressure lookup failed (%s); using %s",
                        exc, "the last reading" if self._value is not None else "101325 Pa")
            return False
        self._value = value
        self._fetched_at = self._clock()
        return True

    async def refresh(self) -> bool:
        """`refresh_now` off the event loop."""
        return await asyncio.to_thread(self.refresh_now)

    def maybe_refresh(self) -> None:
        """Fire-and-forget: schedule a refresh if one is due and none is in
        flight. Safe to call at the broadcaster's rate; a no-op outside a
        running event loop (tests calling `build_sensor_message` directly)."""
        if self._task is not None and not self._task.done():
            return
        if not self.due():
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return                  # no running loop: caller drives refresh_now itself
        self._task = loop.create_task(self.refresh())
