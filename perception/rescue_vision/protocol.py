"""Wire-compatible layout from the K230 draft; explicit status in reserved byte."""
import binascii
import struct

from .types import Observation, Status, Target

HEADER = b"\xaa\x55"
VERSION = 1
MAX_TARGETS = 12
MAX_ZONES = 4
MAX_PAYLOAD = 8 + MAX_TARGETS * 12 + MAX_ZONES * 8
TARGET = struct.Struct("<BBhHHHBB")


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def encode(observation: Observation, sequence: int) -> bytes:
    if len(observation.targets) > MAX_TARGETS:
        raise ValueError("More than 12 targets")
    payload = struct.pack("<IBBBB", observation.tick_ms & 0xFFFFFFFF,
                          observation.flags, len(observation.targets), 0,
                          int(observation.status))
    payload += b"".join(TARGET.pack(t.kind, t.confidence, t.angle_cdeg,
                                  t.distance_mm, t.width_px, t.height_px,
                                  t.posture, t.flags) for t in observation.targets)
    data = struct.pack("<BBH", VERSION, sequence & 255, len(payload)) + payload
    return HEADER + data + struct.pack("<H", crc16(data))


def decode(frame: bytes) -> tuple[int, Observation]:
    if len(frame) < 16 or frame[:2] != HEADER:
        raise ValueError("Invalid frame header/size")
    version, sequence, length = struct.unpack_from("<BBH", frame, 2)
    if version != VERSION or not 8 <= length <= MAX_PAYLOAD or len(frame) != length + 8:
        raise ValueError("Invalid version/length")
    if crc16(frame[2:-2]) != struct.unpack_from("<H", frame, len(frame)-2)[0]:
        raise ValueError("CRC mismatch")
    tick, flags, nt, nz, status = struct.unpack_from("<IBBBB", frame, 6)
    if nt > MAX_TARGETS or nz > MAX_ZONES or length != 8 + nt*12 + nz*8:
        raise ValueError("Invalid item counts")
    try:
        state = Status(status)
    except ValueError as exc:
        raise ValueError("Unknown status") from exc
    targets = tuple(Target(*TARGET.unpack_from(frame, 14+i*12)) for i in range(nt))
    if any(t.confidence > 100 or t.kind not in (1, 2, 3, 4, 255)
           or abs(t.angle_cdeg) > 18000 for t in targets):
        raise ValueError("Invalid target fields")
    return sequence, Observation(tick, flags, state, targets)


class StreamDecoder:
    """Bounded stream parser; advance one byte after corruption, recover next header."""

    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data: bytes) -> list[tuple[int, Observation]]:
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer[:] = b"\xaa" if self.buffer.endswith(b"\xaa") else b""
                break
            del self.buffer[:start]
            if len(self.buffer) < 6:
                break
            length = int.from_bytes(self.buffer[4:6], "little")
            if self.buffer[2] != VERSION or not 8 <= length <= MAX_PAYLOAD:
                del self.buffer[0]
                self.errors += 1
                continue
            if len(self.buffer) < length+8:
                break
            try:
                frames.append(decode(bytes(self.buffer[:length+8])))
            except ValueError:
                self.errors += 1
                del self.buffer[0]
            else:
                del self.buffer[:length+8]
        return frames
