import shutil
import struct
import subprocess
from pathlib import Path

import pytest

from rescue_vision.protocol import StreamDecoder, crc16, decode, encode
from rescue_vision.types import Observation, Status, Target


def observation(tick=100, distance=800, angle=0, status=Status.OK, flags=3, targets=None):
    if targets is None:
        targets = (Target(1, 90, angle, distance, 40, 40, 0, 48),)
    return Observation(tick, flags, status, tuple(targets))


@pytest.fixture(scope="session")
def c_runner(tmp_path_factory):
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.fail("gcc is required for firmware interoperability verification")
    root = Path(__file__).parents[1]
    executable = tmp_path_factory.mktemp("firmware")/"receiver.exe"
    compiled = subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", "-O2",
                    "-I", str(root/"stm32"), str(root/"stm32/rescue_vision.c"),
                    str(root/"tests/c_receiver_harness.c"), "-lm", "-o", str(executable)],
                   capture_output=True)
    assert compiled.returncode == 0, compiled.stderr.decode(errors="replace")
    def run(records, forward=True):
        data = b"".join(struct.pack("<IH", now & 0xFFFFFFFF, len(packet))+packet for now, packet in records)
        result = subprocess.run([str(executable), "120", "0", "1" if forward else "0"],
                                input=data, capture_output=True, check=True)
        rows = [[float(v) for v in line.split()] for line in result.stdout.decode().splitlines()]
        assert len(rows) == len(records), "Harness must consume every binary input record"
        return rows
    return run


def test_crc_reference_and_roundtrip():
    assert crc16(b"123456789") == 0x29B1
    source = observation(angle=-1234)
    assert decode(encode(source, 255)) == (255, source)
    assert decode(encode(source, 256))[0] == 0


def test_stream_fragmentation_noise_and_corruption():
    packet = encode(observation(), 0)
    corrupt = bytearray(packet)
    corrupt[15] ^= 1
    stream = StreamDecoder()
    data = b"junk\xaa" + bytes(corrupt) + packet + packet
    results = []
    for value in data:
        results += stream.feed(bytes([value]))
    assert len(results) == 2
    assert stream.errors > 0
    assert len(stream.buffer) <= 192


def test_c_python_interop_and_motion_limit(c_runner):
    rows = c_runner([(100, encode(observation(), 0))])
    assert rows[0][0] == 2
    assert 0 < rows[0][2] <= 100
    assert rows[0][4] == pytest.approx(680)


@pytest.mark.parametrize("angle,sign", [(1500, 1), (-1500, -1)])
def test_turn_sign_and_no_forward_when_misaligned(c_runner, angle, sign):
    row = c_runner([(100, encode(observation(angle=angle), 0))])[0]
    assert row[0] == 1 and row[2] == 0
    assert row[3]*sign > 0 and abs(row[3]) <= .4


def test_turn_only_trial(c_runner):
    row = c_runner([(100, encode(observation(angle=200), 0))], forward=False)[0]
    assert row[0] == 1 and row[2] == 0


def test_arrival_needs_three_new_frames_and_latches(c_runner):
    a = encode(observation(distance=320), 0)
    rows = c_runner([(100, a), (110, a), (120, b""),
                     (167, encode(observation(167, 320), 1)),
                     (234, encode(observation(234, 320), 2)), (600, b"")])
    assert all(r[0] != 3 for r in rows[:4])
    assert rows[4][0] == rows[5][0] == 3
    assert all(r[2] == r[3] == 0 for r in rows)
    assert rows[1][6] == 1


@pytest.mark.parametrize("status", [Status.NO_TARGET, Status.CAMERA_FAULT,
                                     Status.CALIBRATION_INVALID, Status.TARGET_INVALID,
                                     Status.TARGET_LOST, Status.STALE, Status.INFERENCE_FAULT])
def test_faults_stop_immediately(c_runner, status):
    rows = c_runner([(100, encode(observation(), 0)),
                     (150, encode(observation(150, status=status, flags=0, targets=[]), 1))])
    assert rows[0][2] > 0
    assert rows[1][2] == rows[1][3] == 0


def test_timeout_and_duplicate_cannot_refresh_after_timeout(c_runner):
    packet = encode(observation(), 0)
    rows = c_runner([(100, packet), (399, b""), (400, packet), (500, packet)])
    assert rows[1][2] > 0
    assert rows[2][2] == rows[3][2] == 0
    assert rows[3][6] == 2


def test_intermediate_fault_heartbeat_does_not_reject_new_capture(c_runner):
    rows = c_runner([(100, encode(observation(50), 0)),
                     (150, encode(observation(150, flags=0, status=Status.STALE, targets=[]), 1)),
                     (170, encode(observation(120), 2))])
    assert rows[1][2] == 0
    assert rows[2][2] > 0


@pytest.mark.parametrize("hazard", [Target(4, 80, 0, 600, 40, 40, 0, 33),
                                     Target(4, 80, 0, 0, 40, 40, 0, 1),
                                     Target(255, 80, 0, 600, 40, 40, 0, 32)])
def test_receiver_independently_blocks_hazard(c_runner, hazard):
    targets = [Target(1, 90, 0, 800, 40, 40, 0, 48), hazard]
    row = c_runner([(100, encode(observation(targets=targets), 0))])[0]
    assert row[1] == 3 and row[2] == row[3] == 0


def test_crc_recovery_and_bad_frame_cannot_drive(c_runner):
    bad = bytearray(encode(observation(), 0))
    bad[-1] ^= 1
    rows = c_runner([(100, bytes(bad)), (167, encode(observation(167), 1))])
    assert rows[0][2] == 0
    assert rows[1][2] > 0 and rows[1][7] == 1


def test_clock_wrap(c_runner):
    rows = c_runner([(0xFFFFFFF0, encode(observation(0xFFFFFFE0), 255)),
                     (51, encode(observation(35), 0)), (351, b"")])
    assert rows[0][2] > 0 and rows[1][2] > 0 and rows[2][2] == 0


def test_too_close_never_advances(c_runner):
    row = c_runner([(100, encode(observation(distance=280), 0))])[0]
    assert row[1] == 5 and row[2] == row[3] == 0


def test_overflow_is_zero_command(c_runner):
    row = c_runner([(100, b"x"*600)])[0]
    assert row[1] == 7 and row[2] == row[3] == 0


def test_image_to_ground_to_wire_to_c_arrival(c_runner, calibration, config):
    import numpy as np
    from rescue_vision.pipeline import Perception
    from rescue_vision.types import Detection
    pipeline = Perception(calibration, config)
    records = []
    # Synthetic frames exercise the complete software chain; no claim of real motion.
    for i, distance in enumerate(list(range(1100, 319, -20))+[320]*6):
        bottom = round((1400-distance)/3)
        image = np.full((480, 640, 3), 220, np.uint8)
        image[bottom-40:bottom+1, 300:340] = (0, 200, 0)
        detection = Detection("normal_material", .9, (300, bottom-40, 340, bottom+1))
        now = 100+i*67
        obs = pipeline.process(image, [detection], now)
        records.append((now, encode(obs, i)))
    rows = c_runner(records)
    assert rows[0][2] == rows[1][2] == 0
    assert any(row[2] > 0 for row in rows)
    assert all(0 <= row[2] <= 100 for row in rows)
    assert rows[-1][0] == 3
    assert rows[-1][2] == rows[-1][3] == 0
    assert 170 <= rows[-1][4] <= 230 and abs(rows[-1][5]) <= 20
