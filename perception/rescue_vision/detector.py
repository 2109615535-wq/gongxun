"""Rockchip nine-output YOLO11 decoding, using NumPy DFL (no board-side torch)."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from .types import Detection


def create_detector(config):
    if config.get("detector_backend", "reference") == "existing_ultralytics_rknn":
        from .board_detector import ExistingUltralyticsDetector
        return ExistingUltralyticsDetector(config)
    return Detector(config["model"], config["manifest"], config["confidence"], config["nms_iou"])


def letterbox(image, size=640):
    h, w = image.shape[:2]
    scale = min(size/w, size/h)
    nw, nh = int(round(w*scale)), int(round(h*scale))
    left, top = (size-nw)//2, (size-nh)//2
    resized = cv2.resize(image, (nw, nh))
    padded = cv2.copyMakeBorder(resized, top, size-nh-top, left, size-nw-left,
                               cv2.BORDER_CONSTANT, value=(114, 114, 114))
    # Store actual rounded x/y resize ratios, not just the ideal scale.
    return padded, (nw/w, nh/h, left, top)


def restore_boxes(boxes, transform, shape):
    sx, sy, left, top = transform
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4).copy()
    boxes[:, [0, 2]] = (boxes[:, [0, 2]]-left)/sx
    boxes[:, [1, 3]] = (boxes[:, [1, 3]]-top)/sy
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, shape[1])
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, shape[0])
    return boxes


def box_iou(a, b):
    a, b = np.asarray(a), np.asarray(b)
    lo, hi = np.maximum(a[:2], b[:2]), np.minimum(a[2:], b[2:])
    intersection = np.prod(np.maximum(0, hi-lo))
    area_a = np.prod(np.maximum(0, a[2:]-a[:2]))
    area_b = np.prod(np.maximum(0, b[2:]-b[:2]))
    return float(intersection/max(area_a+area_b-intersection, 1e-9))


def nms(boxes, scores, threshold):
    order = np.argsort(scores)[::-1]
    kept = []
    areas = np.prod(np.maximum(0, boxes[:, 2:]-boxes[:, :2]), axis=1)
    while order.size:
        i = int(order[0])
        kept.append(i)
        rest = order[1:]
        lo = np.maximum(boxes[i, :2], boxes[rest, :2])
        hi = np.minimum(boxes[i, 2:], boxes[rest, 2:])
        intersection = np.prod(np.maximum(0, hi-lo), axis=1)
        overlap = intersection/np.maximum(areas[i]+areas[rest]-intersection, 1e-9)
        order = rest[overlap <= threshold]
    return kept


def decode_rk_outputs(outputs, class_count, size=640, confidence=.35, nms_iou=.45):
    if len(outputs) != 9:
        raise ValueError("Expected Rockchip-optimized 9-output YOLO11; standard ONNX uses a different decoder")
    all_boxes, all_scores, all_classes = [], [], []
    for i in range(3):
        position, probability, score_sum = map(np.asarray, outputs[i*3:i*3+3])
        if (position.ndim != 4 or position.shape[0] != 1 or position.shape[1] != 64
                or probability.shape != (1, class_count, *position.shape[2:])
                or score_sum.shape != (1, 1, *position.shape[2:])):
            raise ValueError("Output shape/class count does not match model manifest")
        if not all(np.isfinite(v).all() for v in (position, probability, score_sum)):
            raise ValueError("Nonfinite model output")
        h, w = position.shape[2:]
        # DFL: 4 edges, 16 bins, softmax along bins.
        p = position.reshape(1, 4, 16, h, w).astype(np.float32)
        p -= p.max(axis=2, keepdims=True)
        p = np.exp(p)
        p /= p.sum(axis=2, keepdims=True)
        distances = (p*np.arange(16).reshape(1, 1, 16, 1, 1)).sum(axis=2)[0]
        gx, gy = np.meshgrid(np.arange(w)+.5, np.arange(h)+.5)
        boxes = np.stack(((gx-distances[0])*size/w, (gy-distances[1])*size/h,
                          (gx+distances[2])*size/w, (gy+distances[3])*size/h), axis=-1).reshape(-1, 4)
        probs = probability[0].transpose(1, 2, 0).reshape(-1, class_count)
        classes = probs.argmax(axis=1)
        scores = probs.max(axis=1)
        selected = scores >= confidence
        all_boxes.append(boxes[selected])
        all_classes.append(classes[selected])
        all_scores.append(scores[selected])
    boxes, classes, scores = map(np.concatenate, (all_boxes, all_classes, all_scores))
    # Bound CPU work even on corrupt/high-response scenes.
    order = np.argsort(scores)[::-1][:1000]
    boxes, classes, scores = boxes[order], classes[order], scores[order]
    kept = []
    for cls in np.unique(classes):
        subset = np.flatnonzero(classes == cls)
        kept.extend(subset[nms(boxes[subset], scores[subset], nms_iou)])
    kept = sorted(kept, key=lambda k: -scores[k])[:100]
    return boxes[kept], classes[kept], scores[kept]


class Detector:
    def __init__(self, model, manifest, confidence=.35, nms_iou=.45):
        self.model_path = Path(model)
        self.manifest = json.loads(Path(manifest).read_text(encoding="utf-8"))
        self.names = self.manifest["names"]
        if (self.manifest.get("layout") != "rockchip_yolo11_9_outputs"
                or self.manifest.get("input_size") != 640
                or self.manifest.get("pad_rgb") != [114, 114, 114]
                or self.manifest.get("input_rgb") is not True
                or not isinstance(self.names, list) or len(set(self.names)) != len(self.names)
                or not set(("normal_material", "core_material", "casualty", "hazard")) <= set(self.names)):
            raise ValueError("Invalid model manifest")
        self.confidence, self.nms_iou = confidence, nms_iou
        self.backend = self.model_path.suffix.lower()
        hash_key = {".pt": "weights_sha256", ".onnx": "onnx_sha256", ".rknn": "rknn_sha256"}.get(self.backend)
        if hash_key is None:
            raise ValueError("Model must be .pt/.onnx/.rknn")
        expected = self.manifest.get(hash_key)
        if expected is None or hashlib.sha256(self.model_path.read_bytes()).hexdigest() != expected:
            raise ValueError("Model hash missing/mismatched; use the matching manifest")
        if self.backend == ".pt":
            from ultralytics import YOLO
            self.model = YOLO(str(model))
            if [self.model.names[i] for i in range(len(self.model.names))] != self.names:
                raise ValueError("Weight class names differ from manifest")
        elif self.backend == ".onnx":
            import onnxruntime as ort
            self.model = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
        elif self.backend == ".rknn":
            from rknnlite.api import RKNNLite
            self.model = RKNNLite()
            if self.model.load_rknn(str(model)) != 0 or self.model.init_runtime() != 0:
                self.model.release()
                raise RuntimeError("RKNN load/init failed; check board NPU driver/runtime compatibility")

    def predict(self, image):
        padded, transform = letterbox(image)
        if self.backend == ".pt":
            result = self.model.predict(padded, imgsz=640, rect=False, device="cpu",
                                        conf=self.confidence, iou=self.nms_iou, verbose=False)[0]
            boxes = result.boxes.xyxy.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            scores = result.boxes.conf.cpu().numpy()
        else:
            rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
            if self.backend == ".onnx":
                tensor = rgb.transpose(2, 0, 1)[None].astype(np.float32)/255.
                outputs = self.model.run(None, {self.model.get_inputs()[0].name: tensor})
            else:
                outputs = self.model.inference(inputs=[rgb[None]], data_format=["nhwc"])
                if outputs is None:
                    raise RuntimeError("RKNN inference failed")
            boxes, classes, scores = decode_rk_outputs(outputs, len(self.names),
                                                      confidence=self.confidence, nms_iou=self.nms_iou)
        boxes = restore_boxes(boxes, transform, image.shape)
        return [Detection(self.names[int(c)], float(s), tuple(map(float, b)))
                for b, c, s in zip(boxes, classes, scores) if b[2] > b[0] and b[3] > b[1]]

    def close(self):
        if self.backend == ".rknn":
            self.model.release()
