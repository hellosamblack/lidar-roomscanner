"""BUG-044: eight call sites called _state_message(ui) without the optional
controller/detailed/calibration_available args, dropping capability context.
NOT REJECTED: Python defaults make the omission legal; Pyright cannot flag it."""
from roomscan.server.session import UiState, _state_message


def state(ui: UiState) -> dict:
    return _state_message(ui)
