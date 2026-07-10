from pathlib import Path

from roomscan.decoder import StreamDecoder   # existing frame decoder; .feed(bytes) -> list[Frame]
from roomscan.sensors import SensorState

FIX = Path(__file__).parent / "fixtures" / "golden_sensors_snippet.bin"


def test_sensor_state_populates_from_capture():
    data = FIX.read_bytes()
    frames = StreamDecoder().feed(data)   # same API host/tests/golden.py uses on golden_pairs_snippet
    st = SensorState()
    for frame in frames:
        st.feed(frame)
    assert st.latest_quat() is not None
    env = st.latest_env()
    assert env is not None
    assert 100000.0 < env.pressure_pa < 103000.0
    assert len(st.pressure_history()) == 8
