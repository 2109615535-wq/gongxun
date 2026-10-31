"""Read current four-motor or legacy UART5 telemetry; optional GET only."""
import json
import math
from pathlib import Path
import time


MOTOR_FIELDS = tuple(key for index in range(1, 5) for key in (f"speed{index}", f"target{index}"))
LEGACY_FIELDS = ("target", "output", "speed", "kp", "ki", "kd", "enable")


def parse_motor_status(line):
    """Preserve firmware field names/units; reject incomplete or invalid samples."""
    try:
        value = json.loads(line)
    except (ValueError, UnicodeError) as exc:
        raise ValueError("Not a JSON motor status") from exc
    if not isinstance(value, dict):
        raise ValueError("Expected JSON object")
    current = any(key in value for key in (*MOTOR_FIELDS, "y_body"))
    fields = (*MOTOR_FIELDS, "y_body") if current else LEGACY_FIELDS
    for key in fields:
        item = value.get(key)
        try:
            finite = isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(item)
        except OverflowError:
            finite = False
        if not finite:
            raise ValueError(f"Invalid status field: {key}")
    speed_fields = MOTOR_FIELDS if current else ("target", "output", "speed")
    if any(not -1000 <= value[k] <= 1000 for k in speed_fields):
        raise ValueError("Motor value outside normalized scale")
    if not current and (any(not 0 <= value[k] <= 100 for k in ("kp", "ki", "kd"))
                        or value["enable"] not in (0, 1)):
        raise ValueError("Invalid gains/enable")
    return {key: value[key] for key in fields}


def parse_motor2_status(line):
    """Compatibility API for callers explicitly expecting the old single-motor format."""
    value = parse_motor_status(line)
    if "target" not in value:
        raise ValueError("Expected legacy motor-2 status")
    return value


def monitor_status(port_name, duration=5, output=None, query=False):
    import serial
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be positive")
    sink = None
    samples = errors = discarded = replies = 0
    error_replies = 0
    last_reply = None
    formats = {"four_motor": 0, "legacy_motor2": 0}
    pending = bytearray()
    dropping = False
    if output:
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        sink = path.open("x", encoding="utf-8")
    try:
        with serial.Serial(port_name, 115200, timeout=.05, write_timeout=.05) as port:
            if query and port.write(b"GET\n") != 4:
                raise RuntimeError("Partial GET write")
            end = time.monotonic()+duration
            while time.monotonic() < end:
                for byte in port.read(256):
                    if byte in (10, 13):
                        if pending and not dropping:
                            try:
                                line = pending.decode("ascii")
                                if line.startswith(("OK,", "ERR,")):
                                    replies += 1
                                    error_replies += int(line.startswith("ERR,"))
                                    last_reply = line
                                    pending.clear()
                                    continue
                                status = parse_motor_status(line)
                            except (ValueError, UnicodeError):
                                errors += 1
                            else:
                                samples += 1
                                kind = "four_motor" if "y_body" in status else "legacy_motor2"
                                formats[kind] += 1
                                row = dict(received_monotonic=time.monotonic(), telemetry_format=kind,
                                           speed_unit="normalized_minus1000_to1000", **status)
                                if kind == "four_motor":
                                    row["y_body_unit"] = "cm"
                                text = json.dumps(row, ensure_ascii=False)
                                if sink:
                                    sink.write(text+"\n")
                                    sink.flush()
                                else:
                                    print(text)
                        pending.clear()
                        dropping = False
                    elif not dropping:
                        if len(pending) >= 512:
                            pending.clear()
                            dropping = True
                            discarded += 1
                        else:
                            pending.append(byte)
    finally:
        if sink:
            sink.close()
    return dict(samples=samples, invalid_lines=errors, oversized_lines=discarded,
                reply_lines=replies, error_replies=error_replies, last_reply=last_reply,
                telemetry_formats=formats,
                baudrate=115200, interface="motor_debug_ascii", sent_command="GET" if query else None,
                chassis_control_available=False)
