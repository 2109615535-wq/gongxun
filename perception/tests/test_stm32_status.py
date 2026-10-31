import json

import pytest

from rescue_vision.config import load_config
from rescue_vision.stm32_status import monitor_status, parse_motor_status, parse_motor2_status


@pytest.fixture
def current_status():
    # Field names and units from project/Core/Src/faraway.c at d915341.
    return dict(speed1=49, target1=50, speed2=48, target2=50,
                speed3=51, target3=50, speed4=50, target4=50, y_body=49.25)


@pytest.fixture
def legacy_status():
    return dict(target=5, output=4, speed=5, kp=1.2, ki=.3, kd=0, enable=1)


def test_current_fields_and_centimeter_odometry_preserved(current_status):
    assert parse_motor_status(json.dumps(current_status)) == current_status
    current_status.update(y_body=-2.5, speed1=-1000, target4=1000)
    assert parse_motor_status(json.dumps(current_status)) == current_status
    with pytest.raises(ValueError, match="legacy"):
        parse_motor2_status(json.dumps(current_status))


@pytest.mark.parametrize("field", ["speed1", "target1", "speed2", "target2", "speed3",
                                  "target3", "speed4", "target4", "y_body"])
def test_incomplete_four_motor_status_cannot_fall_back_to_legacy(current_status, legacy_status, field):
    current_status.pop(field)
    with pytest.raises(ValueError, match="Invalid status field"):
        parse_motor_status(json.dumps(current_status))
    with pytest.raises(ValueError, match="Invalid status field"):
        parse_motor_status(json.dumps(dict(legacy_status, **current_status)))


@pytest.mark.parametrize("changes", [
    {"speed1": 1001}, {"target4": -1001}, {"target2": True}, {"speed3": "50"},
    {"y_body": None}, {"y_body": False}, {"y_body": float("nan")},
    {"y_body": float("inf")}, {"speed4": 10**400}, {"y_body": 10**400},
])
def test_current_invalid_values_are_rejected_without_crashing(current_status, changes):
    with pytest.raises(ValueError):
        parse_motor_status(json.dumps(dict(current_status, **changes)))


def test_legacy_compatibility_and_unknown_fields(legacy_status, current_status):
    assert parse_motor_status(json.dumps(legacy_status)) == legacy_status
    assert parse_motor2_status(json.dumps(legacy_status)) == legacy_status
    assert parse_motor_status(json.dumps(dict(current_status, future_field=123))) == current_status


@pytest.mark.parametrize("line", ["[]", "null", "42", "{}", "not json"])
def test_non_telemetry_is_not_a_valid_sample(line):
    with pytest.raises(ValueError):
        parse_motor_status(line)


@pytest.mark.parametrize("query", [False, True])
def test_stream_recovers_and_never_sends_control_commands(monkeypatch, tmp_path,
                                                        current_status, legacy_status, query):
    import rescue_vision.stm32_status as module
    import serial

    # Acknowledgements, malformed input, oversized input and mixed firmware samples
    # are deliberately split across reads. CRLF must not join a reply to telemetry.
    data = (b"OK,GET\r\nERR,COMMAND\nnot-json\n" + b"x"*600 + b"\n" +
            json.dumps(current_status).encode()+b"\r\n" +
            json.dumps(legacy_status).encode()+b"\n")
    pending = bytearray(data)
    clock = {"now": 0.0}
    writes = []

    class Port:
        def __init__(self, name, baud, **kwargs):
            assert name == "fake-uart" and baud == 115200

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def write(self, packet):
            writes.append(packet)
            return len(packet)

        def read(self, size):
            clock["now"] += .001
            chunk = bytes(pending[:17])
            del pending[:17]
            return chunk

    monkeypatch.setattr(serial, "Serial", Port)
    monkeypatch.setattr(module.time, "monotonic", lambda: clock["now"])
    output = tmp_path/"four_motor.jsonl"
    result = monitor_status("fake-uart", .2, output, query)
    rows = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
    assert writes == ([b"GET\n"] if query else [])
    assert result["samples"] == 2
    assert result["reply_lines"] == 2 and result["invalid_lines"] == 1
    assert result["error_replies"] == 1 and result["last_reply"] == "ERR,COMMAND"
    assert result["oversized_lines"] == 1
    assert result["telemetry_formats"] == {"four_motor": 1, "legacy_motor2": 1}
    assert result["interface"] == "motor_debug_ascii"
    assert not result["chassis_control_available"]
    assert rows[0]["telemetry_format"] == "four_motor"
    assert rows[0]["y_body"] == 49.25 and rows[0]["y_body_unit"] == "cm"
    assert rows[0]["speed_unit"] == "normalized_minus1000_to1000"
    assert rows[1]["telemetry_format"] == "legacy_motor2" and "y_body" not in rows[1]
    assert not pending


@pytest.mark.parametrize("protocol", [None, "motor_debug_ascii", "motor2_debug_ascii", "vision_v1"])
def test_debug_protocol_default_and_legacy_alias(tmp_path, config, protocol):
    if protocol is None:
        config.pop("serial_protocol", None)
    else:
        config["serial_protocol"] = protocol
    path = tmp_path/"vision.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    expected = "vision_v1" if protocol == "vision_v1" else "motor_debug_ascii"
    assert load_config(path)["serial_protocol"] == expected


@pytest.mark.parametrize("protocol", ["motor_debug_ascii", "motor2_debug_ascii"])
def test_debug_port_rejects_observations_before_opening_any_hardware(monkeypatch, tmp_path, config, protocol):
    import serial
    import rescue_vision.runtime as runtime

    def forbidden(*args, **kwargs):
        pytest.fail("A debug port must be rejected before model loading or serial opening")

    monkeypatch.setattr(serial, "Serial", forbidden)
    monkeypatch.setattr(runtime, "create_detector", forbidden)
    config["serial_protocol"] = protocol
    with pytest.raises(ValueError, match="ASCII debug"):
        runtime.run(config, tmp_path/"recording", serial_port="fake-uart")
    assert not (tmp_path/"recording").exists()
