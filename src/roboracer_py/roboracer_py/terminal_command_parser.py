import math
import re
from dataclasses import dataclass
from typing import Optional, Union


@dataclass(frozen=True)
class ParsedCommand:
    kind: str
    value: Optional[Union[str, float]] = None
    raw: str = ''


class CommandKind:
    AUTO_TOGGLE = 'auto_toggle'
    CONTROLLER_TOGGLE = 'controller_toggle'
    AUTO_HOLD = 'auto_hold'
    AUTO_ACTIVE = 'auto_active'
    AUTO_NOW = 'auto_now'
    AUTO_PRESET = 'auto_preset'
    NUMBER_ACTION = 'number_action'
    SOFT_STOP = 'soft_stop'
    STOP = 'stop'
    MANUAL_HOLD = 'manual_hold'
    MANUAL_PRESET = 'manual_preset'
    SPEED = 'speed'
    TURN_SPEED = 'turn_speed'
    TURN_SPEED_FOLLOW = 'turn_speed_follow'
    STEERING_ANGLE = 'steering_angle'
    DISTANCE = 'distance'
    AVOIDANCE = 'avoidance'
    AVOIDANCE_DISTANCE = 'avoidance_distance'
    AVOIDANCE_TURN_TIME = 'avoidance_turn_time'
    SETTINGS = 'settings'
    TRAJECTORY = 'trajectory'
    CANCEL = 'cancel'
    HELP = 'help'
    QUIT = 'quit'
    UNKNOWN = 'unknown'


NUMBER_ACTION_MAP = {
    '1': 'move', 'move': 'move', 'forward': 'move',
    '2': 'left', 'left': 'left',
    '3': 'right', 'right': 'right',
    '4': 'back', 'back': 'back', 'reverse': 'back',
}

MANUAL_PRESET_MAP = {
    'm1': 'move', 'manual_1': 'move',
    'm2': 'left', 'manual_2': 'left',
    'm3': 'right', 'manual_3': 'right',
    'm4': 'back', 'manual_4': 'back',
}

AUTO_PRESET_MAP = {
    'a1': 'move', 'auto_1': 'move',
    'a2': 'left', 'auto_2': 'left',
    'a3': 'right', 'auto_3': 'right',
    'a4': 'back', 'auto_4': 'back',
}

_FLOAT = r'([+-]?(?:\d+(?:\.\d*)?|\.\d+))'


def parse_terminal_command(raw_command: str) -> ParsedCommand:
    raw = raw_command
    command = raw_command.strip().lower()
    compact = re.sub(r'\s+', '', command)

    if not command:
        return ParsedCommand(CommandKind.UNKNOWN, raw=raw)
    if command in ['h', 'help', '?']:
        return ParsedCommand(CommandKind.HELP, raw=raw)
    if command in ['q', 'quit', 'exit']:
        return ParsedCommand(CommandKind.QUIT, raw=raw)
    if command in ['p', 'param', 'params', 'setting', 'settings', 'status']:
        return ParsedCommand(CommandKind.SETTINGS, raw=raw)

    if compact in ['tsauto', 'tsfollow', 'turnspeedauto', 'turnspeedfollow']:
        return ParsedCommand(CommandKind.TURN_SPEED_FOLLOW, raw=raw)

    match = re.fullmatch(rf'(?:ts|tv|turnspeed){_FLOAT}', compact)
    if match:
        return ParsedCommand(
            CommandKind.TURN_SPEED,
            abs(float(match.group(1))),
            raw=raw,
        )

    match = re.fullmatch(rf'(?:sad|steerdeg){_FLOAT}', compact)
    if match:
        return ParsedCommand(
            CommandKind.STEERING_ANGLE,
            math.radians(abs(float(match.group(1)))),
            raw=raw,
        )

    match = re.fullmatch(rf'(?:sa|steer){_FLOAT}d', compact)
    if match:
        return ParsedCommand(
            CommandKind.STEERING_ANGLE,
            math.radians(abs(float(match.group(1)))),
            raw=raw,
        )

    match = re.fullmatch(rf'(?:sa|steer){_FLOAT}', compact)
    if match:
        return ParsedCommand(
            CommandKind.STEERING_ANGLE,
            abs(float(match.group(1))),
            raw=raw,
        )

    # Configuration commands for the scripted avoidance maneuver.
    # Both directions share one forward-distance setting. Turn completion
    # remains time based so it can be calibrated for each platform.
    match = re.fullmatch(rf'avd{_FLOAT}', compact)
    if match:
        return ParsedCommand(
            CommandKind.AVOIDANCE_DISTANCE,
            abs(float(match.group(1))),
            raw=raw,
        )

    match = re.fullmatch(rf'avt{_FLOAT}', compact)
    if match:
        return ParsedCommand(
            CommandKind.AVOIDANCE_TURN_TIME,
            abs(float(match.group(1))),
            raw=raw,
        )

    if compact in ['l', 'avl', 'avoidleft', 'avoid_left']:
        return ParsedCommand(CommandKind.AVOIDANCE, 'left', raw=raw)
    if compact in ['r', 'avr', 'avoidright', 'avoid_right']:
        return ParsedCommand(CommandKind.AVOIDANCE, 'right', raw=raw)

    # Distance commands require an explicit unit suffix:
    #   2m, 0.5m, 10ft, -1m
    # Positive values command forward travel; negative values command reverse.
    match = re.fullmatch(rf'{_FLOAT}(m|ft)', compact)
    if match:
        distance_value = float(match.group(1))
        unit = match.group(2)
        distance_m = distance_value if unit == 'm' else distance_value * 0.3048
        return ParsedCommand(CommandKind.DISTANCE, distance_m, raw=raw)

    if command == '0':
        return ParsedCommand(CommandKind.SOFT_STOP, raw=raw)
    if command in ['s', '5', 'stop', 'estop', 'e_stop', 'emergency_stop', 'disarm']:
        return ParsedCommand(CommandKind.STOP, raw=raw)
    if command in ['j', 'joy', 'controller', 'gamepad']:
        return ParsedCommand(CommandKind.CONTROLLER_TOGGLE, raw=raw)
    if command in ['a', 'auto']:
        return ParsedCommand(CommandKind.AUTO_TOGGLE, raw=raw)
    if command in ['ah', 'auto_hold']:
        return ParsedCommand(CommandKind.AUTO_HOLD, raw=raw)
    if command in ['aa', 'auto_now', 'auto_active_now', 'rescue_auto', 'auto_rescue']:
        return ParsedCommand(CommandKind.AUTO_NOW, raw=raw)
    if command in ['auto_active', 'auto_start', 'start_auto']:
        return ParsedCommand(CommandKind.AUTO_ACTIVE, raw=raw)
    if compact in AUTO_PRESET_MAP:
        return ParsedCommand(
            CommandKind.AUTO_PRESET,
            AUTO_PRESET_MAP[compact],
            raw=raw,
        )
    if command in ['m', 'manual', 'manual_hold']:
        return ParsedCommand(CommandKind.MANUAL_HOLD, raw=raw)
    if compact in MANUAL_PRESET_MAP:
        return ParsedCommand(
            CommandKind.MANUAL_PRESET,
            MANUAL_PRESET_MAP[compact],
            raw=raw,
        )
    if compact in NUMBER_ACTION_MAP:
        return ParsedCommand(
            CommandKind.NUMBER_ACTION,
            NUMBER_ACTION_MAP[compact],
            raw=raw,
        )

    match = re.fullmatch(rf'v{_FLOAT}', compact)
    if match:
        return ParsedCommand(CommandKind.SPEED, float(match.group(1)), raw=raw)

    if compact in ['lc', 'leftcircle', 'left_circle']:
        return ParsedCommand(CommandKind.TRAJECTORY, 'lc', raw=raw)
    if compact in ['rc', 'rightcircle', 'right_circle']:
        return ParsedCommand(CommandKind.TRAJECTORY, 'rc', raw=raw)
    if compact in ['f8', 'figure8', 'figure_eight']:
        return ParsedCommand(CommandKind.TRAJECTORY, 'f8', raw=raw)
    if compact in ['c', 'cancel']:
        return ParsedCommand(CommandKind.CANCEL, raw=raw)

    return ParsedCommand(CommandKind.UNKNOWN, raw=raw)
