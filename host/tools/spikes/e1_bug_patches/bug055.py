"""BUG-055: the SLAM map drew the scanner marker from DEVICE_DIMS -- magcal3d's
unitless shell scale -- as if it were metres (4.44x oversize). The JS site has
no Python twin, so this is the same mistake at the two Python mm/m boundaries
plus the literal shape of the JS bug (an unlabelled constant as Metres)."""
from roomscan.deproject import Deprojector
from roomscan.domain import Metres, Millimetres
from roomscan.slam.tsdf import TsdfMap

DEVICE_DIMS_X = 0.62   # shell units (unitless), as in devicemodel.js


def marker_size() -> Metres:
    return DEVICE_DIMS_X                           # bare float where Metres is required


def make_deprojector() -> Deprojector:
    return Deprojector(54, 42, max_range_mm=Metres(10.0))  # Metres where Millimetres is required


def make_map() -> TsdfMap:
    return TsdfMap(depth_max=Millimetres(5000.0))  # Millimetres where Metres is required
