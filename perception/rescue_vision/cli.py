import argparse
import json
from pathlib import Path
import sys
import time


def capture_images(args):
    import cv2
    directory = Path(args.output)
    directory.mkdir(parents=True, exist_ok=False)
    camera = int(args.camera) if args.camera.isdigit() else args.camera
    cap = cv2.VideoCapture(camera)
    try:
        if not cap.isOpened():
            raise ValueError("Cannot open camera")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        count, last = 0, time.monotonic()
        while count < args.count:
            ok, image = cap.read()
            if not ok:
                raise ValueError("Camera read failed")
            if image.shape[1::-1] != (args.width, args.height):
                raise ValueError("Camera did not accept requested resolution")
            if args.show:
                cv2.imshow("Move chessboard between captures; q to stop", image)
                if cv2.waitKey(1) & 255 == ord("q"):
                    break
            if time.monotonic()-last >= args.interval:
                if not cv2.imwrite(str(directory/f"{count:04d}.png"), image):
                    raise ValueError("Failed to save camera image")
                last = time.monotonic()
                count += 1
        return dict(images=count, directory=str(directory.resolve()))
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser(description="智能救援：单目感知、标定、模型部署与串口联调")
    commands = parser.add_subparsers(dest="command", required=True)
    command = commands.add_parser("export", help="现有PT权重导出瑞芯微9输出ONNX和类别清单")
    command.add_argument("--weights", required=True)
    command.add_argument("--output", default="artifacts")
    command = commands.add_parser("convert-rknn", help="在支持RKNN-Toolkit2的Linux环境转换")
    command.add_argument("--onnx", required=True)
    command.add_argument("--manifest", required=True)
    command.add_argument("--output", required=True)
    command.add_argument("--dataset", help="仅INT8需要；真实相机预处理后的640x640 RGB校准图片清单")
    command = commands.add_parser("capture", help="采集原始相机标定图")
    command.add_argument("--camera", default="0")
    command.add_argument("--output", required=True)
    command.add_argument("--width", type=int, default=640)
    command.add_argument("--height", type=int, default=480)
    command.add_argument("--count", type=int, default=20)
    command.add_argument("--interval", type=float, default=2)
    command.add_argument("--show", action="store_true")
    command = commands.add_parser("intrinsics")
    command.add_argument("--images", required=True)
    command.add_argument("--pattern", type=int, nargs=2, default=(9, 6), metavar=("COLUMNS", "ROWS"))
    command.add_argument("--square-mm", type=float, required=True)
    command.add_argument("--output", required=True)
    command = commands.add_parser("import-intrinsics", help="导入实测OpenCV YAML/XML或JSON内参与畸变")
    command.add_argument("--input", required=True)
    command.add_argument("--image-size", type=int, nargs=2, metavar=("WIDTH", "HEIGHT"))
    command.add_argument("--output", required=True)
    for name in ("ground", "validate-ground"):
        command = commands.add_parser(name)
        command.add_argument("--calibration", required=True)
        command.add_argument("--points", required=True)
        command.add_argument("--output", required=True)
    command = commands.add_parser("undistort", help="输出用于采集地面标定点的去畸变图")
    command.add_argument("--calibration", required=True)
    command.add_argument("--image", required=True)
    command.add_argument("--output", required=True)
    command = commands.add_parser("compare")
    command.add_argument("--reference", required=True)
    command.add_argument("--candidate", required=True)
    command.add_argument("--manifest", required=True)
    command.add_argument("--images", required=True)
    command.add_argument("--output", required=True)
    command.add_argument("--limit", type=int)
    for name in ("run", "replay"):
        command = commands.add_parser(name)
        command.add_argument("--config", default="config/vision.json")
        command.add_argument("--output", required=True)
        command.add_argument("--model", help="覆盖配置中的模型路径，相对当前工作目录")
        command.add_argument("--manifest", help="覆盖模型清单路径")
        if name == "run":
            command.add_argument("--serial", help="提供端口才发送串口；省略则仅记录")
            command.add_argument("--show", action="store_true")
            command.add_argument("--record-video", action="store_true")
            command.add_argument("--duration", type=float)
        else:
            command.add_argument("--video", required=True)
            command.add_argument("--limit", type=int)
    for name in ("report", "trial-report"):
        command = commands.add_parser(name)
        command.add_argument("--input", required=True)
        command.add_argument("--output", required=True)
    command = commands.add_parser("serial-monitor")
    command.add_argument("--port", required=True)
    command.add_argument("--duration", type=float, default=30)
    command.add_argument("--baudrate", type=int, choices=(115200, 921600), default=115200)
    command = commands.add_parser("stm32-status", help="115200 UART5四电机遥测接收；仅可选发送GET")
    command.add_argument("--port", required=True)
    command.add_argument("--duration", type=float, default=5)
    command.add_argument("--output", help="新的JSONL记录文件；省略则打印遥测")
    command.add_argument("--query", action="store_true", help="只发送一次GET，不改变电机参数")
    args = parser.parse_args(argv)
    try:
        if args.command == "export":
            from .model_tools import export_model
            result = export_model(args.weights, args.output)
        elif args.command == "convert-rknn":
            from .model_tools import convert_rknn
            result = convert_rknn(args.onnx, args.manifest, args.output, args.dataset)
        elif args.command == "capture":
            if args.count < 1 or args.interval <= 0:
                raise ValueError("count/interval must be positive")
            result = capture_images(args)
        elif args.command == "intrinsics":
            from .calibrate import calibrate_intrinsics
            from .validation import image_files
            result = calibrate_intrinsics(image_files(args.images), args.pattern, args.square_mm, args.output)
        elif args.command == "import-intrinsics":
            from .calibrate import import_intrinsics
            result = import_intrinsics(args.input, args.output, args.image_size)
        elif args.command in ("ground", "validate-ground"):
            from .calibrate import fit_ground, validate_ground
            fn = fit_ground if args.command == "ground" else validate_ground
            result = fn(args.calibration, args.points, args.output)
        elif args.command == "undistort":
            import cv2
            from .geometry import GroundCalibration
            image = cv2.imread(args.image)
            if image is None:
                raise ValueError("Cannot read image")
            image = GroundCalibration.load(args.calibration).undistort(image)
            if not cv2.imwrite(args.output, image):
                raise ValueError("Cannot write image")
            result = dict(image=args.output)
        elif args.command == "compare":
            from .validation import compare_models
            result = compare_models(args.reference, args.candidate, args.manifest,
                                    args.images, args.output, args.limit)
        elif args.command in ("run", "replay"):
            from .config import load_config
            config = load_config(args.config)
            for key in ("model", "manifest"):
                if getattr(args, key):
                    config[key] = str(Path(getattr(args, key)).resolve())
            if args.command == "run":
                from .runtime import run
                run(config, args.output, args.serial, args.show, args.record_video, args.duration)
                result = dict(output=args.output, serial_transmitted=args.serial is not None)
            else:
                from .validation import replay
                result = replay(config, args.video, args.output, limit=args.limit)
        elif args.command == "report":
            from .validation import performance_report
            result = performance_report(args.input, args.output)
        elif args.command == "trial-report":
            from .validation import trial_report
            result = trial_report(args.input, args.output)
        elif args.command == "stm32-status":
            from .stm32_status import monitor_status
            result = monitor_status(args.port, args.duration, args.output, args.query)
        else:
            import serial
            from dataclasses import asdict
            from .protocol import StreamDecoder
            decoder = StreamDecoder()
            end = time.monotonic()+args.duration
            with serial.Serial(args.port, args.baudrate, timeout=.05) as port:
                while time.monotonic() < end:
                    for sequence, observation in decoder.feed(port.read(192)):
                        print(json.dumps(dict(sequence=sequence, **asdict(observation)), ensure_ascii=False))
            result = dict(parse_errors=decoder.errors)
        summary = {key: value for key, value in result.items() if key != "results"}
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 2 if result.get("passed") is False else 0
    except (ValueError, RuntimeError, OSError, ImportError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
