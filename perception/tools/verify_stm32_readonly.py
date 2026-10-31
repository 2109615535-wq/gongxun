"""Compare source hashes against a saved baseline; never write inside the project."""
import argparse
import hashlib
import json
from pathlib import Path


def snapshot(root):
    root = Path(root)
    extensions = {".c", ".h", ".s", ".ioc", ".uvprojx", ".uvoptx"}
    return {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob("*")) if p.is_file() and p.suffix.lower() in extensions
            and ".git" not in p.relative_to(root).parts}


def compare(root, baseline):
    before = json.loads(Path(baseline).read_text(encoding="utf-8"))
    after = snapshot(root)
    modified = sorted(name for name in before.keys() & after.keys() if before[name] != after[name])
    added, removed = sorted(after.keys()-before.keys()), sorted(before.keys()-after.keys())
    digest = hashlib.sha256(json.dumps(after, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    return dict(unchanged=not (modified or added or removed), source_files=len(after),
                source_tree_sha256=digest, modified=modified, added=added, removed=removed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", required=True)
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = compare(args.project, args.baseline)
    text = json.dumps(result, ensure_ascii=False, indent=2)+"\n"
    if args.output:
        project = Path(args.project).resolve()
        if args.output.resolve().is_relative_to(project):
            raise ValueError("Verification output must be outside the read-only STM32 project")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text)
    return 0 if result["unchanged"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
