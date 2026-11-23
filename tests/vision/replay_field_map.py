"""Replay the last session of two V8 or V9 logs using their raw upper-camera XY."""

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile


GATE = 5.991464547
VARIANCE_PER_CM = 0.09
CONFIRM_FRAMES = 3
TRACK_CAPACITY = 48
FAR_CM = 150.0


def radius(distance):
    return 4.0 + 0.05 * distance


def measurement_variance(distance):
    return radius(distance) ** 2 / GATE


@dataclass
class Detection:
    x: float
    y: float
    distance: float
    camera_x_mm: int
    camera_y_mm: int
    distance_mm: int


@dataclass
class Frame:
    tick: int
    frame_id: int | None
    fresh: bool
    aligned: bool
    robot: tuple
    observed: tuple
    travel: float
    is_new: bool
    detections: list
    wire_version: int


@dataclass
class Track:
    x: float
    y: float
    variance: float
    hits: int = 1
    number: int = 0

    def update(self, detection, variance):
        gain = self.variance / (self.variance + variance)
        self.x += gain * (detection.x - self.x)
        self.y += gain * (detection.y - self.y)
        self.variance *= 1.0 - gain


@dataclass
class OldTrack:
    x: float
    y: float
    distance: float
    number: int
    pending_x: float = 0.0
    pending_y: float = 0.0
    pending_count: int = 0

    def update(self, detection):
        threshold = radius(max(self.distance, detection.distance)) * 0.5
        if math.hypot(detection.x - self.x, detection.y - self.y) > threshold:
            if not self.pending_count or math.hypot(
                detection.x - self.pending_x, detection.y - self.pending_y
            ) > threshold:
                self.pending_x, self.pending_y = detection.x, detection.y
                self.pending_count = 0
            self.pending_count += 1
            if self.pending_count < CONFIRM_FRAMES:
                return
        self.x, self.y, self.distance = detection.x, detection.y, detection.distance
        self.pending_count = 0


def read_frames(path):
    """Use the live odometry independently of the camera's frame clock."""
    last_session = []
    with path.open(encoding="utf-8-sig") as stream:
        for line in stream:
            if not line.strip():
                continue
            record = json.loads(line)
            data = record.get("data") or {}
            version = data.get("wire_version")
            source = data.get("source")
            if (record.get("source") != "live" or "error" in record
                    or not ((version == 8 and source == "upper")
                            or (version == 9 and source in ("state", "upper")))):
                continue
            if last_session and ((data["tick"] - last_session[-1]["tick"]) & 0xFFFFFFFF) >= 0x80000000:
                last_session.clear()
            last_session.append(data)
    if not last_session:
        raise ValueError(f"{path}: no parsed live V8/V9 pose records")

    seen = set()
    previous_robot = None
    previous_tick = None
    for data in last_session:
        field = data["field"]
        if data["tick"] == previous_tick:
            robot, travel = previous_robot, 0
        else:
            robot = tuple(field["robot"][axis] for axis in ("x", "y"))
            travel = 0 if previous_robot is None else math.dist(robot, previous_robot)
            previous_robot, previous_tick = robot, data["tick"]
        frame_id = data.get("frame") if data["source"] == "upper" else None
        is_new = frame_id is not None and frame_id not in seen
        if frame_id is not None:
            seen.add(frame_id)
        pose = field.get("observed")
        fresh = bool(data.get("fresh"))
        aligned = bool(field["aligned"] and pose)
        observed = tuple(pose[axis] for axis in ("x", "y", "yaw")) if aligned else (0, 0, 0)
        detections = []
        if is_new and fresh and aligned:
            rx, ry, yaw = observed
            angle = math.radians(yaw)
            cosine, sine = math.cos(angle), math.sin(angle)
            for obj in data["objects"]:
                if data["wire_version"] == 9:
                    if not obj.get("position_present"):
                        continue
                    x, y = obj["raw_x_mm"] * 0.1, obj["raw_y_mm"] * 0.1
                else:
                    x, y = obj.get("x_cm"), obj.get("y_cm")
                if obj["kind"] != 1 or not all(
                    type(value) in (int, float) and math.isfinite(value) for value in (x, y)
                ):
                    continue
                distance_mm = obj["d_mm"]
                distance = distance_mm / 10.0 if distance_mm >= 0 else math.hypot(x, y)
                detections.append(Detection(
                    rx + x * cosine + y * sine,
                    ry - x * sine + y * cosine,
                    distance,
                    round(10 * x), round(10 * y), distance_mm,
                ))
        yield Frame(data["tick"], frame_id, fresh, aligned, robot, observed,
                    travel, is_new, detections, data["wire_version"])


def minimum_assignment(cost):
    """Rectangular Hungarian algorithm with one or more columns per row."""
    nrows, ncols = len(cost), len(cost[0])
    row_potential = [0.0] * (nrows + 1)
    col_potential = [0.0] * (ncols + 1)
    row_for_col = [0] * (ncols + 1)
    predecessor = [0] * (ncols + 1)
    for row in range(1, nrows + 1):
        row_for_col[0] = row
        current = 0
        used = [False] * (ncols + 1)
        slack = [math.inf] * (ncols + 1)
        while True:
            used[current] = True
            current_row = row_for_col[current]
            delta, next_col = math.inf, 0
            for col in range(1, ncols + 1):
                if used[col]:
                    continue
                reduced = (cost[current_row - 1][col - 1]
                           - row_potential[current_row] - col_potential[col])
                if reduced < slack[col]:
                    slack[col], predecessor[col] = reduced, current
                if slack[col] < delta:
                    delta, next_col = slack[col], col
            for col in range(ncols + 1):
                if used[col]:
                    row_potential[row_for_col[col]] += delta
                    col_potential[col] -= delta
                else:
                    slack[col] -= delta
            current = next_col
            if not row_for_col[current]:
                break
        while current:
            prior = predecessor[current]
            row_for_col[current] = row_for_col[prior]
            current = prior
    result = [-1] * nrows
    for col in range(1, ncols + 1):
        if row_for_col[col]:
            result[row_for_col[col] - 1] = col - 1
    return result


def match(detections, variances, indexes, tracks):
    if not indexes or not tracks:
        return {}
    # Losing one match costs more than the sum of all feasible distances.
    unmatched = (len(indexes) + 1) * GATE
    blocked = (len(indexes) + 1) * unmatched
    cost = []
    for i in indexes:
        detection = detections[i]
        row = []
        for track in tracks:
            squared = ((detection.x - track.x) ** 2 + (detection.y - track.y) ** 2)
            value = squared / (track.variance + variances[i])
            row.append(value if value <= GATE else blocked)
        cost.append(row + [unmatched] * len(indexes))
    return {
        indexes[row]: col for row, col in enumerate(minimum_assignment(cost))
        if col < len(tracks) and cost[row][col] <= GATE
    }


class Tracker:
    def __init__(self):
        self.confirmed = []
        self.candidates = []

    def process(self, frame):
        for track in self.confirmed + self.candidates:
            track.variance += VARIANCE_PER_CM * frame.travel
        if not frame.is_new:
            return []

        detections = frame.detections
        variances = [measurement_variance(d.distance) for d in detections]
        indexes = list(range(len(detections)))
        matched = match(detections, variances, indexes, self.confirmed)
        remaining = [i for i in indexes if i not in matched]
        pending = match(detections, variances, remaining, self.candidates)
        result = [0] * len(indexes)
        for i, col in matched.items():
            track = self.confirmed[col]
            track.update(detections[i], variances[i])
            result[i] = track.number
        next_candidates = []
        for i in remaining:
            detection = detections[i]
            if i not in pending:
                next_candidates.append(Track(detection.x, detection.y, variances[i]))
                continue
            track = self.candidates[pending[i]]
            track.update(detection, variances[i])
            track.hits += 1
            if track.hits < CONFIRM_FRAMES:
                next_candidates.append(track)
            elif len(self.confirmed) < TRACK_CAPACITY:
                track.number = len(self.confirmed) + 1
                self.confirmed.append(track)
                result[i] = track.number
        self.candidates = next_candidates
        return result


class OldTracker:
    """Original greedy global nearest pair and accepted-position jump handling."""

    def __init__(self):
        self.tracks = []

    def process(self, frame):
        if not frame.is_new:
            return []
        detections = frame.detections
        matched = {}
        used = set()
        while True:
            pairs = []
            for i, detection in enumerate(detections):
                if i in matched:
                    continue
                for j, track in enumerate(self.tracks):
                    if j in used:
                        continue
                    d2 = (detection.x - track.x) ** 2 + (detection.y - track.y) ** 2
                    if d2 <= radius(max(track.distance, detection.distance)) ** 2:
                        pairs.append((d2, track.number, i, j))
            if not pairs:
                break
            _, _, i, j = min(pairs)
            matched[i] = j
            used.add(j)

        result = []
        for i, detection in enumerate(detections):
            if i in matched:
                j = matched[i]
                track = self.tracks[j]
                track.update(detection)
            elif len(self.tracks) < TRACK_CAPACITY:
                j = len(self.tracks)
                track = OldTrack(detection.x, detection.y, detection.distance, j + 1)
                self.tracks.append(track)
            else:
                result.append(0)
                continue
            used.add(j)
            result.append(track.number)
        for j, track in enumerate(self.tracks):
            if j not in used:
                track.pending_count = 0
        return result


def write_fixtures(path, rows):
    """State-only rows carry `S tick robot_x robot_y`. Camera rows carry tick,
    frame, fresh, aligned, robot XY, observed XY/yaw, count, and each target's
    camera XY and distance in mm plus reference ID (zero if pending).
    """
    with path.open("w", encoding="ascii", newline="\n") as stream:
        for frame, ids in rows:
            if frame.frame_id is None:
                stream.write(f"S {frame.tick} {frame.robot[0]} {frame.robot[1]}\n")
                continue
            raw = [str(v) for v in (
                frame.tick, frame.frame_id, int(frame.fresh), int(frame.aligned),
                *frame.robot, *frame.observed, len(frame.detections),
            )]
            for detection, number in zip(frame.detections, ids):
                raw.extend(str(v) for v in (
                    detection.camera_x_mm, detection.camera_y_mm, detection.distance_mm, number,
                ))
            stream.write(" ".join(raw) + "\n")


def replay(path):
    old, new = OldTracker(), Tracker()
    statistics = {name: {band: Counter() for band in ("near", "far")} for name in ("old", "new")}
    first = {"old": {}, "new": {}}
    rows = []
    for frame in read_frames(path):
        for name, ids in (("old", old.process(frame)), ("new", new.process(frame))):
            assert len(ids) == len(frame.detections)
            for detection, number in zip(frame.detections, ids):
                band = "near" if detection.distance < FAR_CM else "far"
                statistics[name][band][number] += 1
                if number:
                    first[name].setdefault(number, frame.tick)
            if name == "new":
                rows.append((frame, ids))
    row_counts = Counter(frame.wire_version for frame, _ in rows)
    return {
        "v8_rows": row_counts[8],
        **({"v9_rows": row_counts[9], "status_only_rows": sum(
            frame.frame_id is None for frame, _ in rows
        )} if 9 in row_counts else {}),
        "distinct_upper_frames": sum(frame.is_new for frame, _ in rows),
        "first_tick_ms": rows[0][0].tick,
        "last_tick_ms": rows[-1][0].tick,
        "confirmed_ids": {
            name: {band: sum(number != 0 for number in counts)
                   for band, counts in ranges.items()}
            for name, ranges in statistics.items()
        },
        "first_confirmed_ms": first,
        "counts": statistics,
    }, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, epilog=(
        "Use --uart5-bandwidth --help for the UART5 encoding and scheduling benchmark."
    ))
    parser.add_argument("log_a", type=Path)
    parser.add_argument("log_b", type=Path)
    parser.add_argument("--fixtures-dir", type=Path, help="Write per-status ASCII A.txt and B.txt for C replay.")
    args = parser.parse_args()
    report = {}
    for label, path in (("A", args.log_a), ("B", args.log_b)):
        report[label], rows = replay(path)
        if args.fixtures_dir:
            args.fixtures_dir.mkdir(parents=True, exist_ok=True)
            write_fixtures(args.fixtures_dir / f"{label}.txt", rows)
    print(json.dumps(report, indent=2))


ROOT = Path(__file__).resolve().parents[2]
WINDOWS = (
    ("2026-11-22T11:38:05+00:00", "2026-11-22T11:38:45+00:00"),
    ("2026-11-22T11:41:18+00:00", "2026-11-22T11:41:35+00:00"),
)
UNKNOWN_AGE = 0xFFFFFFFF
TYPES = ("", "LIVE", "EVENT", "TRACK", "HISTORY", "INFO", "STATS",
         "STEER", "ACK", "CONTROL", "ERROR", "MAP_SYNC")
STATS_KEYS = ("vision_rx", "vision_bad", "vision_uart_errors", "tx_drops",
              "tx_bytes", "coalesced_live", "event_overflows", "uart5_errors")


def load_capture(path):
    """Read successful V8/M1 wire records and preserve the browser's error counts."""
    valid = []
    errors = Counter()
    with path.open(encoding="utf-8-sig") as source:
        for line in source:
            record = json.loads(line)
            if record["source"] != "live":
                continue
            if record.get("error"):
                errors[record["error"]] += 1
                continue
            raw = record.get("raw")
            if raw and (raw.startswith('["V8",') or raw.startswith('["M1",')):
                array = json.loads(raw.rsplit("*", 1)[0])
                at = datetime.fromisoformat(record["received_at"])
                valid.append((at, array, raw))
    return valid, errors


def nullable_age(value):
    return value if value is not None else UNKNOWN_AGE


def fixture_fields(a):
    if a[0] == "M1":
        _, tick, epoch, revision, count, unmapped, track = a
        row = ["M", tick, epoch, revision, count, unmapped]
        if track is None:
            return [*row, 0]
        i, kind, x, y, radius, seen, pos, observations, visible, review, rev, history = track
        return [*row, i, rev, kind, visible | (review << 1), x, y, radius,
                seen, pos, observations, len(history), *(v for point in history for v in point)]

    _, tick, link, rx, bad, uart, drops, frame, age, status, fresh, fault, objects, servo, lower, mission, field, control, imu = a
    step, reason, inside, outside, held = mission
    robot, aligned, observed, points, origin, extrinsic = field
    command_id, command_status, target_x, target_y, target_speed = control
    ready, imu_age, yaw, heading = imu
    flags = link | (ready << 1) | (heading << 2) | 8 | 16 | (fault << 5)
    row = ["V", tick, step, reason, held, inside | (outside << 4), servo, flags,
           *robot, imu_age or 0, yaw or 0, *origin, *extrinsic, rx, bad, uart, drops,
           int(command_id is not None), command_id or 0, command_status,
           int(target_x is not None), target_x or 0, target_y or 0, round((target_speed or 0) * 10),
           frame, nullable_age(age), status, fresh | (fresh << 1) | (aligned << 2), len(objects),
           *(observed or (0, 0, 0))]
    for o, point in zip(objects, points, strict=True):
        kind, distance, angle, quality, *raw_xy = o
        bits = int(distance >= 0) | ((angle is not None) << 1) | (bool(raw_xy) << 2)
        bits |= (quality << 3) | ((point is not None) << 4)
        row.extend([kind, bits, max(0, distance), angle or 0,
                    *(raw_xy or (0, 0)), *(point or (0, 0, 0, 0))])
    lower_frame, lower_age, lower_status, lower_fresh, lower_objects = lower
    row.extend([lower_frame, nullable_age(lower_age), lower_status,
                lower_fresh | (lower_fresh << 1), len(lower_objects)])
    for o in lower_objects:
        kind, in_roi, *zone = o
        bits = in_roi | (bool(zone) << 1) | ((zone[1] if zone else 0) << 2)
        row.extend([kind, bits, zone[0] if zone else 0])
    return row


def write_fixture(path, rows):
    with path.open("w", encoding="ascii") as output:
        for _, array, _ in rows:
            output.write(" ".join(str(v) for v in fixture_fields(array)) + "\n")


def summarize(counts, sizes, duration):
    return {
        name: {"packets": counts[name], "bytes": sizes[name], "B/s": round(sizes[name] / duration, 3)}
        for name in sorted(sizes)
    }


def baseline(rows, duration):
    counts, sizes = Counter(), Counter()
    for _, array, raw in rows:
        counts[array[0]] += 1
        sizes[array[0]] += len(raw.encode("ascii")) + 2  # CRLF on wire.
    return {"by_type": summarize(counts, sizes, duration),
            "bytes": sum(sizes.values()), "B/s": round(sum(sizes.values()) / duration, 3)}


def unstuff(wire):
    """Inspect actual C-generated frames for timing, types and counters."""
    raw = bytearray()
    pos = 0
    assert wire[-1] == 0
    while pos < len(wire) - 1:
        code = wire[pos]
        assert code
        raw.extend(wire[pos + 1:pos + code])
        pos += code
        if pos < len(wire) - 1 and code != 0xFF:
            raw.append(0)
    assert pos == len(wire) - 1
    assert len(raw) == 12 + int.from_bytes(raw[4:6], "little")
    return TYPES[raw[1]], raw[10:-2]


def run_same(exe, work, rows, duration):
    capture = work / "same.txt"
    encoded = work / "same.hex"
    originals = work / "same.ndjson"
    write_fixture(capture, rows)
    with encoded.open("w", encoding="ascii") as output:
        subprocess.run([str(exe), "same", str(capture)], stdout=output, check=True)
    with originals.open("w", encoding="ascii") as output:
        for _, _, raw in rows:
            output.write(json.dumps(raw) + "\n")
    checked = subprocess.run(["node", str(ROOT / "tests/vision/test_error_sampling.cjs"),
                              "--bandwidth", str(originals), str(encoded)], capture_output=True, text=True, check=False)
    if checked.returncode:
        raise RuntimeError(checked.stderr.strip())
    counts, sizes = Counter(), Counter()
    for line in encoded.read_text().splitlines():
        wire = bytes.fromhex(line)
        name, _ = unstuff(wire)
        counts[name] += 1
        sizes[name] += len(wire)
    return {"by_type": summarize(counts, sizes, duration), "bytes": sum(sizes.values()),
            "B/s": round(sum(sizes.values()) / duration, 3),
            "cross_decode": json.loads(checked.stdout)}


def run_policy(exe, capture, tick, duration_ms, mode):
    result = subprocess.run([str(exe), "window", str(capture), str(tick), str(duration_ms), mode],
                            check=True, capture_output=True, text=True)
    counts, sizes = Counter(), Counter()
    bucket = Counter()
    counters = []
    windows = []
    statuses = []
    ack = []
    sync_batches = {}
    total_bytes = 0
    peak_frame = 0
    for line in result.stdout.splitlines():
        start, completed, hex_wire = line.split()
        start, completed = int(start), int(completed)
        wire = bytes.fromhex(hex_wire)
        name, payload = unstuff(wire)
        if name == "LIVE":
            statuses.append(start)
            name += "_OBS" if payload[6] & 0x18 else "_STATE"
        if name == "STATS":
            values = [int.from_bytes(payload[13 + i * 4:17 + i * 4], "little")
                      for i in range(len(STATS_KEYS))]
            sample = dict(zip(STATS_KEYS, values, strict=True))
            sample["tx_queue_high_water_bytes"] = int.from_bytes(payload[45:47], "little")
            counters.append(sample)
        if name == "ACK":
            # Only the isolated control benchmark has a single known request.
            if mode == "control":
                ack.append(completed - 1000)
        if name == "MAP_SYNC":
            sync_id = int.from_bytes(payload[:2], "little")
            batch = sync_batches.setdefault(sync_id, {"total": payload[4], "indexes": [], "ended": False})
            if payload[2] == 1:
                batch["indexes"].append(payload[3])
            elif payload[2] == 2:
                batch["ended"] = True
        n = len(wire)
        counts[name] += 1
        sizes[name] += n
        total_bytes += n
        bucket[start // 1000] += n
        windows.append((start, n))
        peak_frame = max(peak_frame, n)
    left = span_bytes = peak = 0
    for right, n in windows:
        span_bytes += n
        while windows[left][0] <= right - 1000:
            span_bytes -= windows[left][1]
            left += 1
        peak = max(peak, span_bytes)
    duration = duration_ms / 1000
    delta = ({key: counters[-1][key] - counters[0][key] for key in STATS_KEYS}
             if len(counters) > 1 else {})
    return {
        "by_type": summarize(counts, sizes, duration),
        "bytes": total_bytes,
        "B/s": round(total_bytes / duration, 3),
        "peak_frame_bytes": peak_frame,
        "peak_1s_sliding_bytes": peak,
        "peak_1s_aligned_bytes": max(bucket.values(), default=0),
        "max_live_gap_ms": max((b - a for a, b in zip(statuses, statuses[1:])), default=0),
        "stats_first": counters[0] if counters else {},
        "stats_last": counters[-1] if counters else {},
        "stats_delta_between_reports": delta,
        **({"control_ack_latency_ms": ack} if mode == "control" else {}),
        **({"sync_batches": sync_batches} if sync_batches else {}),
    }


def window_rows(valid, start, end):
    in_window = [r for r in valid if start <= r[0] < end]
    first = next(r for r in in_window if r[1][0] == "V8")
    origin = first[1][1] - round((first[0] - start).total_seconds() * 1000)
    # Select the live session containing this window, allowing small serial arrival jitter.
    session = []
    last = None
    for record in valid:
        at, array, _ = record
        if at >= end:
            break
        mc_tick = array[1]
        if last is not None and mc_tick < last - 1000:
            session.clear()
        last = mc_tick
        session.append(record)
    assert session and session[0][0] < start and first in session
    return in_window, origin, sorted(session, key=lambda r: r[1][1])


def bandwidth_main(argv):
    parser = argparse.ArgumentParser(description="Benchmark the UART5 legacy capture against binary encoding and scheduling.")
    parser.add_argument("log", type=Path)
    parser.add_argument("--cc", default="gcc")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    valid, errors = load_capture(args.log)
    report = {"log": str(args.log), "successful_baseline": {}, "decoder_errors": dict(errors),
              "method": [
                  "Old bytes are successful raw ASCII size plus CRLF. Capture errors are counted separately.",
                  "Same-sample comparison uses the shared C codec and production browser decoder.",
                  "For full-content comparison, old V8 is represented by LIVE, INFO, CONTROL and STATS at each original sample. Old M1 keeps its complete HISTORY.",
                  "Policy replay calls the real faraway.c scheduler and advances simulated UART completion at 115200 8N1 with the production 64-byte chunk and 5ms gap.",
                  "Policy warms from the start of each old MCU session. Window zero is anchored with the first V8 arrival and MCU tick. Map state uses the most recent successfully captured M1 per ID.",
                  "A MAP request can only replay IDs seen by that request time; the old reported total can exceed that reconstructed set.",
                  "Policy bytes are charged when the complete frame starts in the window. Sliding peak is the largest sum of complete frame sizes whose starts fit in 1 s.",
                  "HIST queries a single track. Export asks for all known tracks sequentially. Control sends one unbound KEY2. Steer renews the subscription every 4 s.",
                  "This is a logged-input host replay; over-air packet losses, Zigbee timing and real vehicle response were not measured.",
              ],
              "windows": []}
    for name in ("V8", "M1"):
        samples = [len(raw.encode("ascii")) + 2 for _, array, raw in valid if array[0] == name]
        report["successful_baseline"][name] = {
            "packets": len(samples), "bytes": sum(samples), "mean_bytes": round(sum(samples) / len(samples), 2)
        }
    with tempfile.TemporaryDirectory(prefix="uart5-bandwidth-") as directory:
        work = Path(directory)
        exe = work / "benchmark.exe"
        subprocess.run([args.cc, "-std=c99", "-Wall", "-W", "-Werror",
                        "-I", str(ROOT / "tests/vision/stubs"), "-I", str(ROOT / "project/Core/Inc"),
                        "-DUART5_BANDWIDTH_REPLAY", str(ROOT / "tests/vision/test_map_memory_transport.c"), "-lm", "-o", str(exe)], check=True)
        for i, (from_time, to_time) in enumerate(WINDOWS):
            start, end = datetime.fromisoformat(from_time), datetime.fromisoformat(to_time)
            duration = (end - start).total_seconds()
            duration_ms = round(duration * 1000)
            samples, tick, session = window_rows(valid, start, end)
            window = {"start_utc": from_time, "end_utc": to_time, "duration_s": duration,
                      "mcu_start_tick": tick, "old": baseline(samples, duration),
                      "same_samples": run_same(exe, work, samples, duration)}
            capture = work / "window.txt"
            write_fixture(capture, session)
            window["default"] = run_policy(exe, capture, tick, duration_ms, "default")
            window["same_sample_encoding_B/s_saved"] = round(
                window["old"]["B/s"] - window["same_samples"]["B/s"], 3)
            window["policy_B/s_saved_vs_same_binary_content"] = round(
                window["same_samples"]["B/s"] - window["default"]["B/s"], 3)
            if i == 1:
                window["optional"] = {}
                optional_types = {"history": ("HISTORY",), "export": ("HISTORY",),
                                  "sync": ("MAP_SYNC",), "steer": ("STEER", "ACK", "CONTROL"),
                                  "control": ("ACK", "CONTROL", "EVENT", "ERROR")}
                for mode, types in optional_types.items():
                    comparison = run_policy(exe, capture, tick, duration_ms, mode)
                    direct = {name: comparison["by_type"][name] for name in types
                              if name in comparison["by_type"]}
                    comparison["direct_optional_traffic"] = {
                        "by_type": direct, "bytes": sum(detail["bytes"] for detail in direct.values())
                    }
                    comparison["incremental_bytes_vs_default"] = comparison["bytes"] - window["default"]["bytes"]
                    window["optional"][mode] = comparison
            report["windows"].append(window)
    if report["windows"][-1]["default"]["B/s"] >= 1500:
        raise ValueError("Default 19:41 window exceeds 1500 B/s")
    output = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(output, encoding="utf-8")
    print(output, end="")


if __name__ == "__main__":
    if sys.argv[1:2] == ["--uart5-bandwidth"]:
        bandwidth_main(sys.argv[2:])
    else:
        main()
