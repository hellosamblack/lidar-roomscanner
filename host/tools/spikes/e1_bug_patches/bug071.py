"""BUG-071: resolve_command's name -> CommandCode map never gained a "standby"
arm, so a generic cmd request for it returned None ("unknown"). Against the
exhaustive CmdName match, dropping the arm is a type error at assert_never."""
from typing import assert_never

from roomscan.protocol import CommandCode
from roomscan.server.session import CmdName


def resolve_known_command(cmd: CmdName, param) -> tuple[CommandCode, int, str]:
    match cmd:
        case CmdName.PING:
            return CommandCode.PING, 0, "ping"
        case CmdName.CALIB:
            return CommandCode.SEND_CALIB, 0, "calib"
        case CmdName.REINIT:
            return CommandCode.REINIT, 0, "reinit"
        case CmdName.USECASE:
            return CommandCode.SET_USECASE, int(param), f"usecase {int(param)}"
        case CmdName.PERIOD:
            return CommandCode.SET_FRAME_PERIOD_US, int(param), f"period {int(param)}"
        case CmdName.EXPOSURE:
            return CommandCode.SET_EXPOSURE_MS, int(param), f"exposure {int(param)}"
        case _:
            assert_never(cmd)   # CmdName.STANDBY is unhandled
