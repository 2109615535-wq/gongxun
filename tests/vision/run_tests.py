"""Compile actual RV2 UART6 receiver and static-test telemetry; no hardware is opened."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "perception"))
from rescue_vision.simple_protocol import SimpleObject, SimpleObservation, encode

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", default="gcc")
    parser.add_argument("--skip-trajectory", action="store_true", help="Run functional regressions separately from the nonconvergent trajectory diagnostic")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="vision-rv2-") as tmp:
        binary = Path(tmp) / "test_vision.exe"
        command = [args.cc, "-std=c99", "-Wall", "-W", "-Werror",
                   "-I", str(ROOT / "tests/vision/stubs"),
                   "-I", str(ROOT / "project/Core/Inc"),
                   str(ROOT / "tests/vision/test_vision.c"), "-o", str(binary)]
        subprocess.run(command, check=True)
        fixture = Path(tmp) / "rv2.txt"
        kinds = ["green","black","orange","blue"] * 3
        obs = SimpleObservation("upper", 1000, 10, "ok",
              tuple(SimpleObject(k,500,False,False,30.0) for k in kinds))
        fixture.write_bytes(encode(obs))
        subprocess.run([str(binary), str(fixture)], check=True)
        map_binary = Path(tmp) / "test_field_map.exe"
        subprocess.run([args.cc,"-std=c99","-Wall","-W","-Werror","-I",str(ROOT/"tests/vision/stubs"),"-I",str(ROOT/"project/Core/Inc"),str(ROOT/"tests/vision/test_field_map.c"),"-lm","-o",str(map_binary)],check=True)
        subprocess.run([str(map_binary)],check=True)
        imu_binary = Path(tmp) / "test_imu.exe"
        subprocess.run([args.cc,"-std=c99","-Wall","-W","-Werror","-I",str(ROOT/"tests/vision/stubs"),"-I",str(ROOT/"project/Core/Inc"),str(ROOT/"tests/vision/test_imu.c"),"-lm","-o",str(imu_binary)],check=True)
        subprocess.run([str(imu_binary)],check=True)
        static_binary = Path(tmp) / "test_static.exe"
        command = [args.cc, "-std=c99", "-Wall", "-W", "-Werror",
                   "-I", str(ROOT / "tests/vision/stubs"), "-I", str(ROOT / "project/Core/Inc"),
                   str(ROOT / "tests/vision/test_static.c"), "-lm", "-o", str(static_binary)]
        subprocess.run(command, check=True)
        result = subprocess.run([str(static_binary)], capture_output=True, text=True, errors="replace")
        if result.returncode:
            print(result.stdout, result.stderr)
            result.check_returncode()
        print(result.stdout.strip())
        telemetry = next(line for line in result.stdout.splitlines() if line.startswith('["V7",'))
        fixture = Path(tmp) / "firmware_telemetry.json"
        fixture.write_text(telemetry, encoding="utf-8")
        subprocess.run(["node", str(ROOT / "tests/vision/test_viewer.cjs"), str(fixture)], check=True)
        subprocess.run(["node", str(ROOT / "tests/vision/test_error_sampling.cjs")], check=True)
        memory_binary = Path(tmp) / "test_map_memory_transport.exe"
        subprocess.run([args.cc,"-std=c99","-Wall","-W","-Werror",
                        "-I",str(ROOT/"tests/vision/stubs"),"-I",str(ROOT/"project/Core/Inc"),
                        str(ROOT/"tests/vision/test_map_memory_transport.c"),"-lm","-o",str(memory_binary)],check=True)
        memory = subprocess.run([str(memory_binary)],capture_output=True,text=True,errors="replace")
        if memory.returncode:
            print(memory.stdout,memory.stderr)
            memory.check_returncode()
        print(next(line for line in memory.stdout.splitlines() if line.startswith("PASS:")))
        memory_fixture=Path(tmp)/"map_memory.ndjson"
        memory_fixture.write_text(memory.stdout,encoding="utf-8")
        subprocess.run(["node",str(ROOT/"tests/vision/test_map_memory.cjs"),str(memory_fixture)],check=True)
        steer_binary = Path(tmp) / "test_steering.exe"
        subprocess.run([args.cc,"-std=c99","-Wall","-W","-Werror",
                        "-I",str(ROOT/"tests/vision/stubs"),"-I",str(ROOT/"project/Core/Inc"),
                        str(ROOT/"tests/vision/test_steering.c"),"-lm","-o",str(steer_binary)],check=True)
        steering = subprocess.run([str(steer_binary)],capture_output=True,text=True,errors="replace")
        if steering.returncode:
            print(steering.stdout,steering.stderr)
            steering.check_returncode()
        print("\n".join(line for line in steering.stdout.splitlines() if line.startswith("PASS:")))
        steer_fixture=Path(tmp)/"steering.txt"
        steer_fixture.write_text(steering.stdout,encoding="utf-8")
        subprocess.run(["node",str(ROOT/"tests/vision/test_steering.cjs"),str(steer_fixture)],check=True)
        near_binary = Path(tmp) / "test_near_alignment.exe"
        subprocess.run([args.cc,"-std=c99","-Wall","-W","-Werror",
                        "-I",str(ROOT/"tests/vision/stubs"),"-I",str(ROOT/"project/Core/Inc"),
                        str(ROOT/"tests/vision/test_near_alignment.c"),"-lm","-o",str(near_binary)],check=True)
        subprocess.run([str(near_binary)],check=True)
        if args.skip_trajectory:
            return
        trajectory_binary = Path(tmp) / "test_goto_trajectory.exe"
        subprocess.run([args.cc,"-std=c99","-Wall","-W","-Werror",
                        "-I",str(ROOT/"tests/vision/stubs"),"-I",str(ROOT/"project/Core/Inc"),
                        str(ROOT/"tests/vision/test_goto_trajectory.c"),"-lm","-o",str(trajectory_binary)],check=True)
        subprocess.run([str(trajectory_binary)],check=True)




if __name__ == "__main__":
    main()
