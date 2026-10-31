"""C100绿色物资测距入口：使用YOLO与已检查的离车地面映射。"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rescue_vision.bench_green import main

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("已停止检测。")
    except Exception as error:
        print(f"检测未能继续：{error}", file=sys.stderr)
        raise SystemExit(1)
