"""Export the existing trained detector using Rockchip's cv2/cv3 output convention."""
import hashlib
import json
from pathlib import Path
from types import MethodType


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")


def export_model(weights, output):
    import onnx
    import torch
    import ultralytics
    from ultralytics import YOLO

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(weights))
    if model.task != "detect":
        raise ValueError("Only ordinary YOLO11 detection weights are supported")
    names = [model.names[i] for i in range(len(model.names))]
    net = model.model.cpu().float().eval()
    head = net.model[-1]
    if (getattr(head, "reg_max", None) != 16 or getattr(head, "nl", None) != 3
            or not hasattr(head, "cv2") or not hasattr(head, "cv3")
            or getattr(head, "end2end", False)):
        raise ValueError("Weights do not have the supported three-scale YOLO11 head")

    # Same outputs as airockchip/ultralytics_yolo11 Detect.forward(format='rknn').
    # Reuse the trained layers without changing the source package or checkpoint.
    def rockchip_forward(self, features):
        result = []
        for index in range(self.nl):
            result.append(self.cv2[index](features[index]))
            probability = self.cv3[index](features[index]).sigmoid()
            result.extend((probability, probability.sum(1, keepdim=True).clamp(0, 1)))
        return tuple(result)

    head.forward = MethodType(rockchip_forward, head)
    sample = torch.zeros(1, 3, 640, 640)
    output_names = [f"{label}_{stride}" for stride in (8, 16, 32)
                    for label in ("box_dfl", "class_probability", "score_sum")]
    path = output/"best.onnx"
    with torch.no_grad():
        net(sample)
        torch.onnx.export(net, sample, str(path), input_names=["images"],
                          output_names=output_names, opset_version=12,
                          do_constant_folding=True, dynamo=False)
    graph = onnx.load(str(path))
    onnx.checker.check_model(graph)
    shapes = [[d.dim_value for d in item.type.tensor_type.shape.dim] for item in graph.graph.output]
    expected = [[1, c, 640//s, 640//s] for s in (8, 16, 32) for c in (64, len(names), 1)]
    if shapes != expected:
        raise ValueError(f"Exported graph has unexpected output shapes: {shapes}")
    manifest = dict(layout="rockchip_yolo11_9_outputs", input_size=640,
                    names=names, weights_sha256=file_hash(weights), onnx_sha256=file_hash(path),
                    ultralytics_version=ultralytics.__version__, opset=12,
                    output_names=output_names, output_shapes=shapes,
                    pad_rgb=[114, 114, 114], input_rgb=True,
                    export_reference="https://github.com/airockchip/ultralytics_yolo11/blob/main/ultralytics/nn/modules/head.py")
    write_json(output/"model.json", manifest)
    return manifest


def convert_rknn(onnx_path, manifest_path, output, dataset=None):
    from rknn.api import RKNN
    manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    if file_hash(onnx_path) != manifest.get("onnx_sha256"):
        raise ValueError("ONNX hash mismatch")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    model = RKNN(verbose=False)
    try:
        # RGB uint8 board input is normalized by RKNN, not twice in application code.
        if model.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform="rk3588") != 0:
            raise RuntimeError("RKNN config failed")
        if model.load_onnx(model=str(onnx_path)) != 0:
            raise RuntimeError("RKNN load_onnx failed")
        if model.build(do_quantization=dataset is not None, dataset=dataset) != 0:
            raise RuntimeError("RKNN build failed")
        if model.export_rknn(str(output)) != 0:
            raise RuntimeError("RKNN export failed")
    finally:
        model.release()
    manifest.update(rknn_sha256=file_hash(output), rknn_quantized=dataset is not None)
    write_json(output.with_suffix(".json"), manifest)
    return manifest
