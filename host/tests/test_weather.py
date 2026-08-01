"""weather.py — the sea-level pressure reference for the elevation readout.

Every test here runs with **no network access at all**: the parse is a pure
function over a string, and `MslPressure` takes its fetch callable by injection.
That is the whole reason `parse_msl_pressure` is split out of the I/O -- the
wire format is exactly the part that can change under us, and it must be
testable on a box with no uplink (which this rig frequently is, sitting behind
a battery Wi-Fi bridge).
"""

import pytest

from roomscan import weather


# --- parse_msl_pressure: real-shaped input ---------------------------------

REAL_RESPONSE = """{
  "latitude": 45.0,
  "longitude": -93.25,
  "generationtime_ms": 0.0349,
  "utc_offset_seconds": -18000,
  "timezone": "America/Chicago",
  "timezone_abbreviation": "CDT",
  "elevation": 264.0,
  "current_units": {"time": "iso8601", "interval": "seconds",
                    "surface_pressure": "hPa", "pressure_msl": "hPa"},
  "current": {"time": "2026-07-31T15:00", "interval": 900,
              "surface_pressure": 982.4, "pressure_msl": 1013.6}
}"""


def test_parse_real_shaped_response_returns_pascals():
    """Open-Meteo reports hPa; everything downstream is pascals."""
    assert weather.parse_msl_pressure(REAL_RESPONSE) == pytest.approx(101360.0)


def test_parse_ignores_surface_pressure():
    """`surface_pressure` is requested only so a hand-curled response is
    self-describing. Parsing it instead would put the elevation ~260 m off
    here -- and would look perfectly plausible."""
    assert weather.parse_msl_pressure(REAL_RESPONSE) != pytest.approx(98240.0)


def test_parse_accepts_an_integer_reading():
    assert weather.parse_msl_pressure('{"current": {"pressure_msl": 1013}}') == pytest.approx(101300.0)


# --- parse_msl_pressure: adversarial input ---------------------------------

@pytest.mark.parametrize("body, why", [
    ("", "empty body"),
    ("not json at all", "not JSON"),
    ("[1, 2, 3]", "JSON array, not an object"),
    ('"a string"', "JSON string, not an object"),
    ("{}", "no `current`"),
    ('{"current": null}', "`current` is null"),
    ('{"current": []}', "`current` is not an object"),
    ('{"current": {}}', "no `pressure_msl`"),
    ('{"current": {"pressure_msl": null}}', "null reading"),
    ('{"current": {"pressure_msl": "1013.2"}}', "string reading"),
    ('{"current": {"pressure_msl": true}}', "bool is not a pressure"),
    ('{"current": {"pressure_msl": 0}}', "0 hPa: out of range"),
    ('{"current": {"pressure_msl": 101325}}', "pascals in the hPa field"),
    ('{"current": {"pressure_msl": 1.0132}}', "bar in the hPa field"),
    ('{"current": {"pressure_msl": -1013.2}}', "negative"),
])
def test_parse_rejects_malformed_input(body, why):
    """Every failure is a ValueError, never a silently plausible number.

    The range check is load-bearing, not belt-and-braces: a wrong-unit value
    (pascals or bar in the hPa field) parses fine as a float and would put the
    reported elevation kilometres out with nothing to notice it by.
    """
    with pytest.raises(ValueError):
        weather.parse_msl_pressure(body)


def test_build_url_carries_the_coordinates_and_the_field():
    url = weather.build_url(45.014060, -93.245526)
    assert url.startswith(weather.API_URL + "?")
    assert "latitude=45.014060" in url and "longitude=-93.245526" in url
    assert "pressure_msl" in url


# --- MslPressure: caching, staleness, failure ------------------------------

class _Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def test_starts_on_the_fallback_and_says_so():
    """Before any fetch the reference is the standard atmosphere, and
    `source` reports that rather than pretending the number was measured."""
    msl = weather.MslPressure(45.0, -93.0, fetch=lambda *a, **k: 101300.0, clock=_Clock())
    assert msl.msl_pa == weather.FALLBACK_MSL_PA
    assert msl.source == "fallback"
    assert msl.age_s is None
    assert msl.snapshot() == {"msl_pa": 101325.0, "msl_source": "fallback", "msl_age_s": None}


def test_successful_refresh_caches_and_reports_api():
    clock = _Clock()
    calls = []

    def fetch(lat, lon, **kw):
        calls.append((lat, lon))
        return 100900.0

    msl = weather.MslPressure(45.0, -93.0, refresh_s=1800.0, fetch=fetch, clock=clock)
    assert msl.refresh_now() is True
    assert msl.msl_pa == pytest.approx(100900.0)
    assert msl.source == "api"
    assert msl.age_s == pytest.approx(0.0)
    assert calls == [(45.0, -93.0)]

    # Not due again until refresh_s has passed -> no second call.
    clock.t = 100.0
    assert msl.due() is False
    clock.t = 1801.0
    assert msl.due() is True


def test_a_failed_refresh_keeps_the_last_value_and_never_raises():
    clock = _Clock()
    state = {"fail": False}

    def fetch(lat, lon, **kw):
        if state["fail"]:
            raise OSError("no route to host")
        return 100900.0

    msl = weather.MslPressure(45.0, -93.0, refresh_s=1800.0, fetch=fetch, clock=clock)
    msl.refresh_now()
    state["fail"] = True
    clock.t = 1801.0
    assert msl.refresh_now() is False            # no exception escapes
    assert msl.msl_pa == pytest.approx(100900.0)  # last good reading survives
    assert msl.source == "api"                    # still inside the stale window

    # Keep failing long enough and it is reported as stale, not as fresh.
    clock.t = 1801.0 + 3600.0 * 2
    assert msl.source == "stale"


def test_never_fetched_falls_back_and_backs_off():
    """A box with no uplink must not retry on every broadcaster tick."""
    clock = _Clock()
    attempts = []

    def fetch(lat, lon, **kw):
        attempts.append(clock.t)
        raise OSError("dns failure")

    msl = weather.MslPressure(45.0, -93.0, fetch=fetch, clock=clock)
    assert msl.due() is True
    msl.refresh_now()
    assert msl.msl_pa == weather.FALLBACK_MSL_PA
    assert msl.source == "fallback"
    clock.t = 10.0
    assert msl.due() is False                     # inside the failure cooldown
    clock.t = 10.0 + weather._RETRY_AFTER_FAIL_S
    assert msl.due() is True


def test_a_bad_body_is_a_failed_refresh_not_a_crash():
    """The parse's ValueError must be caught by the same path as a socket
    error -- a schema change at the far end cannot be allowed to take the
    broadcaster down."""
    def fetch(lat, lon, **kw):
        return weather.parse_msl_pressure('{"current": {"pressure_msl": "nope"}}')

    msl = weather.MslPressure(45.0, -93.0, fetch=fetch, clock=_Clock())
    assert msl.refresh_now() is False
    assert msl.source == "fallback"


def test_fetch_msl_pressure_uses_the_injected_opener():
    """`fetch_msl_pressure` is url-build + GET + parse; with an opener injected
    it exercises everything except the socket."""
    seen = {}

    def opener(url, timeout):
        seen["url"] = url
        seen["timeout"] = timeout
        return REAL_RESPONSE

    got = weather.fetch_msl_pressure(45.0, -93.0, timeout=1.5, opener=opener)
    assert got == pytest.approx(101360.0)
    assert "latitude=45.000000" in seen["url"] and seen["timeout"] == 1.5


def test_maybe_refresh_outside_an_event_loop_is_a_no_op():
    """The broadcaster calls this every sensor tick; a synchronous caller
    (tests, the desktop panel) must not get a RuntimeError for it."""
    msl = weather.MslPressure(45.0, -93.0, fetch=lambda *a, **k: 101300.0, clock=_Clock())
    msl.maybe_refresh()                 # no running loop -> silently does nothing
    assert msl.source == "fallback"


def test_maybe_refresh_inside_a_loop_fetches_once():
    import asyncio

    calls = []

    def fetch(lat, lon, **kw):
        calls.append(1)
        return 100800.0

    msl = weather.MslPressure(45.0, -93.0, fetch=fetch)

    async def drive():
        msl.maybe_refresh()
        msl.maybe_refresh()          # in-flight -> must not start a second
        await asyncio.sleep(0.2)

    asyncio.run(drive())
    assert calls == [1]
    assert msl.msl_pa == pytest.approx(100800.0)
    assert msl.source == "api"
