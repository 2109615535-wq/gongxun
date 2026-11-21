"""Compile the actual UART6 vision receiver and static-test telemetry; no hardware is opened."""
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
        extended_fixture = Path(tmp) / "rv4.txt"
        block = SimpleObject("green", 65535, False, False, -179.99, -65535, -65535,
                             "blue_material_zone", True, True, True)
        zone = SimpleObject("blue_casualty_zone", 65535, False, False, -179.99, -65535, -65535)
        obs = SimpleObservation("upper", 1001, 10, "ok", (block, zone) * 6)
        extended_fixture.write_bytes(encode(obs, version=4))
        subprocess.run([str(binary), str(fixture), str(extended_fixture)], check=True)
        static_binary = Path(tmp) / "test_static.exe"
        command = [args.cc, "-std=c99", "-Wall", "-W", "-Werror",
                   "-I", str(ROOT / "tests/vision/stubs"), "-I", str(ROOT / "project/Core/Inc"),
                   str(ROOT / "tests/vision/test_static.c"), "-lm", "-o", str(static_binary)]
        subprocess.run(command, check=True)
        result = subprocess.run([str(static_binary)], check=True, capture_output=True, text=True)
        print(result.stdout.strip())
        telemetry = next(line for line in result.stdout.splitlines() if line.startswith('["V3",'))
        fixture = Path(tmp) / "firmware_telemetry.json"
        fixture.write_text(telemetry, encoding="utf-8")
        subprocess.run(["node", str(ROOT / "tests/vision/test_viewer.cjs"), str(fixture)], check=True)


if __name__ == "__main__":
    main()
