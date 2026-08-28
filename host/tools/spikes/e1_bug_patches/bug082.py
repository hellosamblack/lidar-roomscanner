"""BUG-082: a 359 -> 0 heading crossing was read as 359 degrees of motion
because the heading delta was a plain subtraction. NOT REJECTED: Bearing is a
NewType over float, so `Bearing - Bearing` is an ordinary float and the diffs
dict accepts it. Only `bearing_delta` discipline guards this in Python."""
from roomscan.domain import Bearing


def heading_motion(prev: Bearing, cur: Bearing) -> dict[str, float]:
    return {"heading": abs(cur - prev)}
