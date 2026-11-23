"""Run STM32 control and UART5 wire regressions with fake HAL; no hardware is opened."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "perception"))
from rescue_vision.simple_protocol import SimpleObject, SimpleObservation, encode

# Captured from the previous firmware for legacy page regressions.
LEGACY_WIRE = {
    "legacy_v8.txt": (
        '["V8",37,1,0,0,0,0,42,40,0,1,0,[[1,500,3000,0,251,432],[1,500,-3000,0],[1,500,null,0]],120,[42,40,0,1,[[1,1,6,1],[1,0]]],[0,0,0,0,0],[[2850,745,0],1,[2850,725,0],[[3101,1156,0,65],[2600,1158,0,65],null],[2850,695,0],[0,0,0]],[null,0,null,null,null],[1,0,2500,1]]*C4BF\n'
    ),
    "legacy_m1.ndjson": (
        '["M1",1306,1,13,1,0,[1,1,2858,1195,65,1300,1300,13,1,0,13,[[1160,2867,1195,1],[1180,2850,1195,1],[1200,2858,1195,1],[1220,2867,1195,1],[1240,2850,1195,1],[1260,2858,1195,1],[1280,2867,1195,1],[1300,2850,1195,1]]]]*D73E\n'
        '["M1",2347,1,14,1,0,[1,1,2858,1195,65,1300,1300,13,0,0,14,[[1160,2867,1195,1],[1180,2850,1195,1],[1200,2858,1195,1],[1220,2867,1195,1],[1240,2850,1195,1],[1260,2858,1195,1],[1280,2867,1195,1],[1300,2850,1195,1]]]]*A476\n'
        '["M1",12388,1,14,1,0,[1,1,2858,1195,65,1300,1300,13,0,0,14,[[1160,2867,1195,1],[1180,2850,1195,1],[1200,2858,1195,1],[1220,2867,1195,1],[1240,2850,1195,1],[1260,2858,1195,1],[1280,2867,1195,1],[1300,2850,1195,1]]]]*A227\n'
    ),
    "legacy_s1.txt": (
        '["S1",1068,1,21,0,-10,10,30,25,5,0,2.5,0.5,0.12]*15F6\n'
    ),
}

def terminal_alignment_fixture(log, output):
    rows = [json.loads(line) for line in log.read_text(encoding="utf-8-sig").splitlines() if line.strip()]
    start = next(row for row in rows if row["type"] == "START")
    steps = [row for row in rows if row["type"] == "STEP" and row["to"] == 5]
    anchors = [(turn, stop) for turn, stop in zip(steps, steps[1:])
               if turn["action"] == 2 and stop["action"] == 0 and turn["frame"] == stop["frame"]
               and turn["age_ms"] < start["obs_max_ms"] <= stop["age_ms"]
               and stop["tick"] - turn["tick"] == 20 and abs(abs(turn["bearing"]) - 7.0) <= 0.2]
    if not anchors:
        raise ValueError(f"{log}: no terminal ~7-degree turn interrupted by the 200 ms frame boundary")
    turn, stop = anchors[-1]
    fixes = [row for row in rows if row["type"] == "FIX" and row["tick"] >= steps[0]["tick"]]
    # The stationary replay cannot reproduce translation after exposure during
    # entry into the terminal position band. Keep the continuous in-band tail.
    first = 0
    for i, row in enumerate(fixes):
        yaw = math.radians(row["yaw"])
        u, v = row["x_mm"] * 0.1, row["y_mm"] * 0.1
        x = start["zx"] - (u * math.cos(yaw) + v * math.sin(yaw))
        y = start["zy"] - (-u * math.sin(yaw) + v * math.cos(yaw))
        if max(abs(x - start["gx"]), abs(y - start["gy"])) > start["pos_tol"]:
            first = i + 1
    fixes = fixes[first:]
    if not any(row["frame"] == turn["frame"] for row in fixes):
        raise ValueError(f"{log}: missing recorded FIX for the interrupted turn")
    with output.open("w", encoding="ascii") as fixture:
        print(len(fixes), start["tick"], turn["tick"], stop["tick"], turn["frame"], file=fixture)
        for row in fixes:
            print(*(row[name] for name in ("frame", "obs_tick", "tick", "yaw", "angle", "x_mm", "y_mm")),
                  file=fixture)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", default="gcc")
    parser.add_argument("--skip-trajectory", action="store_true", help="Run functional regressions separately from the nonconvergent trajectory diagnostic")
    parser.add_argument("--field-replay", nargs=2, type=Path, metavar=("LOG_A", "LOG_B"),
                        help="Replay two raw V8 logs and compare STM32 mapping against the Python reference")
    parser.add_argument("--key2-output", type=Path, metavar="DIR",
                        help="Keep the KEY2 simulated UART5 fixture and decoded example JSONL")
    parser.add_argument("--key2-align-log", type=Path, metavar="JSONL",
                        help="Replay recorded terminal fixes on the original exposure/application timeline")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="vision-rv-") as tmp:
        binary = Path(tmp) / "test_vision.exe"
        command = [args.cc, "-std=c99", "-Wall", "-W", "-Werror",
                   "-I", str(ROOT / "tests/vision/stubs"),
                   "-I", str(ROOT / "project/Core/Inc"),
                   str(ROOT / "tests/vision/test_vision.c"), "-o", str(binary)]
        subprocess.run(command, check=True)
        fixture = Path(tmp) / "vision_uart6.txt"
        kinds = ["green", "black", "orange", "blue", "red_casualty_zone",
                 "red_material_zone", "blue_casualty_zone", "blue_material_zone",
                 "red_material_zone", "blue_material_zone", "red_casualty_zone", "blue_casualty_zone"]
        zones = {"green": "red_material_zone", "black": "blue_material_zone",
                 "orange": "blue_casualty_zone"}
        with fixture.open("wb") as stream:
            for version in (3, 4):
                objects = tuple(SimpleObject(kind, 65535, False, False, -135.0, -46340, -46340,
                                             zones.get(kind), kind in zones, kind in zones,
                                             in_safety_zone=version == 4 and kind in zones)
                                for kind in kinds)
                obs = SimpleObservation("upper", 1000 + version, 10, "ok", objects)
                packet = encode(obs, version=version)
                assert 512 < len(packet.rstrip()) <= 1024
                stream.write(packet)
        subprocess.run([str(binary), str(fixture)], check=True)
        map_binary = Path(tmp) / "test_field_map.exe"
        subprocess.run([args.cc,"-std=c99","-Wall","-W","-Werror","-I",str(ROOT/"tests/vision/stubs"),"-I",str(ROOT/"project/Core/Inc"),str(ROOT/"tests/vision/test_field_map.c"),"-lm","-o",str(map_binary)],check=True)
        subprocess.run([str(map_binary)],check=True)
        if args.field_replay:
            subprocess.run([sys.executable, "-B", str(ROOT / "tests/vision/replay_field_map.py"),
                            *(str(log) for log in args.field_replay), "--fixtures-dir", tmp], check=True)
            replay_binary = Path(tmp) / "test_field_map_replay.exe"
            subprocess.run([args.cc, "-std=c99", "-Wall", "-W", "-Werror",
                            "-I", str(ROOT / "tests/vision/stubs"), "-I", str(ROOT / "project/Core/Inc"),
                            str(ROOT / "tests/vision/test_field_map_replay.c"), "-lm", "-o", str(replay_binary)],
                           check=True)
            subprocess.run([str(replay_binary), str(Path(tmp) / "A.txt"), str(Path(tmp) / "B.txt")], check=True)
        imu_binary = Path(tmp) / "test_imu.exe"
        subprocess.run([args.cc,"-std=c99","-Wall","-W","-Werror","-I",str(ROOT/"tests/vision/stubs"),"-I",str(ROOT/"project/Core/Inc"),str(ROOT/"tests/vision/test_imu.c"),"-lm","-o",str(imu_binary)],check=True)
        subprocess.run([str(imu_binary)],check=True)
        tests = ROOT / "tests/vision"
        flags = [args.cc, "-std=c99", "-Wall", "-W", "-Werror",
                 "-I", str(tests / "stubs"), "-I", str(ROOT / "project/Core/Inc")]

        def build_with_uart5(name):
            executable = Path(tmp) / (name + ".exe")
            subprocess.run([*flags, str(tests / (name + ".c")),
                            "-lm", "-o", str(executable)], check=True)
            return executable

        vectors = []
        for name in ("test_static", "test_map_memory_transport", "test_steering"):
            executable = build_with_uart5(name)
            result = subprocess.run([str(executable)], capture_output=True, text=True, errors="replace")
            for line in result.stdout.splitlines():
                if line.startswith("WIRE "):
                    vectors.append(line)
                elif line.strip():
                    print(line, flush=True)
            if result.returncode:
                print(result.stderr)
                result.check_returncode()
        fixture = Path(tmp) / "binary_vectors.txt"
        fixture.write_text("\n".join(vectors) + "\n", encoding="ascii")
        subprocess.run(["node", str(tests / "test_steering.cjs"), "--firmware", str(fixture)], check=True)
        legacy = Path(tmp)
        for name, content in LEGACY_WIRE.items():
            (legacy / name).write_text(content, encoding="ascii")
        subprocess.run(["node", str(tests / "test_viewer.cjs"), str(legacy / "legacy_v8.txt")], check=True)
        subprocess.run(["node", str(ROOT / "tests/vision/test_error_sampling.cjs")], check=True)
        subprocess.run(["node", str(tests / "test_map_memory.cjs"), str(legacy / "legacy_m1.ndjson")], check=True)
        subprocess.run(["node", str(tests / "test_steering.cjs"), str(legacy / "legacy_s1.txt")], check=True)
        subprocess.run([str(build_with_uart5("test_near_alignment"))], check=True)
        key2_output = args.key2_output or Path(tmp)
        key2_output.mkdir(parents=True, exist_ok=True)
        key2_fixture = key2_output / "KEY2-fixture.txt"
        key2_command = [str(build_with_uart5("test_key2"))]
        if args.key2_align_log:
            alignment_fixture = key2_output / "KEY2-terminal-replay.txt"
            terminal_alignment_fixture(args.key2_align_log, alignment_fixture)
            key2_command.append(str(alignment_fixture))
        key2_run = subprocess.run(key2_command, capture_output=True, text=True, errors="replace")
        key2_lines = key2_run.stdout.splitlines()
        key2_fixture.write_text("\n".join(line for line in key2_lines if line.startswith("WIRE ")) + "\n",
                                encoding="ascii")
        for line in key2_lines:
            if not line.startswith("WIRE "):
                print(line, flush=True)
        if key2_run.returncode:
            print(key2_run.stderr, flush=True)
            key2_run.check_returncode()
        red_key2 = Path(tmp) / "test_key2_red.exe"
        subprocess.run([*flags, "-DRETURN_TEAM=VISION_RED", str(tests / "test_key2.c"),
                        "-lm", "-o", str(red_key2)], check=True)
        subprocess.run([str(red_key2)], check=True)
        subprocess.run(["node", str(tests / "test_key2_page.cjs"), "--firmware", str(key2_fixture),
                        "--example", str(key2_output / "KEY2-example.jsonl")], check=True)
        if args.skip_trajectory:
            return
        subprocess.run([str(build_with_uart5("test_goto_trajectory"))], check=True)




if __name__ == "__main__":
    main()
