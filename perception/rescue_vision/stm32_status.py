"""The current faraway.c interface: read motor-2 telemetry; optional GET only."""
import json
import math
from pathlib import Path
import time


def parse_motor2_status(line):
    try:
        value = json.loads(line)
    except (ValueError, UnicodeError) as exc:
        raise ValueError("Not a JSON motor status") from exc
    if not isinstance(value, dict):
        raise ValueError("Expected JSON object")
    for key in ("target", "output", "speed", "kp", "ki", "kd", "enable"):
        item = value.get(key)
        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
            raise ValueError(f"Invalid status field: {key}")
    if any(not -1000 <= value[k] <= 1000 for k in ("target", "output", "speed")):
        raise ValueError("Motor value outside normalized scale")
    if any(not 0 <= value[k] <= 100 for k in ("kp", "ki", "kd")) or value["enable"] not in (0, 1):
        raise ValueError("Invalid gains/enable")
    return {key: value[key] for key in ("target", "output", "speed", "kp", "ki", "kd", "enable")}


def monitor_status(port_name, duration=5, output=None, query=False):
    import serial
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be positive")
    sink = None
    samples = errors = discarded = 0
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
                                status = parse_motor2_status(pending.decode("ascii"))
                            except (ValueError, UnicodeError):
                                errors += 1
                            else:
                                samples += 1
                                row = dict(received_monotonic=time.monotonic(), **status)
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
                baudrate=115200, interface="motor2_debug_ascii", sent_command="GET" if query else None,
                chassis_control_available=False)
