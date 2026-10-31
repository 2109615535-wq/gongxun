"""Build a handoff zip without caches, recordings or unrelated firmware."""
import argparse
from pathlib import Path
import zipfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=Path("artifacts/rescue_vision_handoff_v3.zip"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    files = [root/"README.md", root/"HANDOFF_V2.md", root/"HANDOFF_V3.md", root/"PROTOCOL.md", root/"pyproject.toml", root/".gitignore"]
    for directory in ("rescue_vision", "config", "stm32", "tests", "tools", "hardware", "examples"):
        files += [p for p in (root/directory).rglob("*") if p.is_file() and
                  "__pycache__" not in p.parts and p.suffix not in (".pyc", ".exe", ".dll")]
    for name in ("best.onnx", "model.json", "best.rknn", "best.json", "onnx_comparison.json", "validation.json"):
        path = root/"artifacts"/name
        if path.is_file():
            files.append(path)
    for name in ("stm32_source_before.json", "stm32_source_verified.json"):
        path = root/"artifacts/hardware_review"/name
        if path.is_file():
            files.append(path)
    evidence = root/"artifacts/remote_review/evidence"
    if evidence.is_dir():
        files += [p for p in evidence.iterdir() if p.is_file() and p.suffix in (".json", ".jsonl", ".bin", ".jpg")]
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(set(files)):
            archive.write(path, Path("rescue_vision_handoff")/path.relative_to(root))
    print(output)


if __name__ == "__main__":
    main()
