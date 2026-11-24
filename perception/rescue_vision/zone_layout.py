"""Resolve same-colour safety-zone candidates within one camera frame."""
from .detector import box_iou


COLORS = ("red", "blue")
ZONES = {f"{color}_{kind}_zone" for color in COLORS for kind in ("material", "casualty")}
DUPLICATE_IOU = .80
EDGE_MARGIN_PX = 4


def _center_x(box):
    return (box[0] + box[2]) / 2


def _side_by_side(left, right, image_size):
    if image_size:
        width, height = image_size
        # Each zone may touch its own outer edge. A vertical crop or a box
        # reaching the opposite edge cannot establish a left/right pair.
        if (left[2] >= width - EDGE_MARGIN_PX or right[0] <= EDGE_MARGIN_PX
                or min(left[1], right[1]) <= EDGE_MARGIN_PX
                or max(left[3], right[3]) >= height - EDGE_MARGIN_PX):
            return False
    lw, lh = left[2] - left[0], left[3] - left[1]
    rw, rh = right[2] - right[0], right[3] - right[1]
    dx = _center_x(right) - _center_x(left)
    dy = abs(right[1] + right[3] - left[1] - left[3]) / 2
    # Compare separation, vertical alignment and size in units of the two ROIs.
    return (min(lw, rw) >= .60 * max(lw, rw) and min(lh, rh) >= .60 * max(lh, rh)
            and .45 <= dx / ((lw + rw) / 2) <= 1.30
            and dy <= .35 * min(lh, rh) and dy <= .30 * dx
            and box_iou(left, right) <= .22)


def _record(detection):
    return {"name": detection.name, "confidence": detection.confidence, "box": list(detection.box)}


def resolve_zones(detections, *, forward=False, image_size=None):
    """Return (original detection, final name) pairs and per-colour diagnostics."""
    final = {i: d.name for i, d in enumerate(detections) if d.name not in ZONES}
    diagnostics = {}
    for color in COLORS:
        layout_names = (f"{color}_material_zone", f"{color}_casualty_zone")
        indices = [i for i, d in enumerate(detections) if d.name in layout_names]
        if not indices:
            continue
        groups = {}
        for i in sorted(indices, key=lambda i: -detections[i].confidence):
            representative = next((j for j in groups if
                                   box_iou(detections[i].box, detections[j].box) >= DUPLICATE_IOU), None)
            if representative is None:
                groups[i] = []
            else:
                groups[representative].append(i)

        kept = sorted(groups, key=lambda i: _center_x(detections[i].box))
        model_names = tuple(detections[i].name for i in kept)
        conflict = any(detections[j].name != detections[i].name for i in kept for j in groups[i])
        status, reason = "model", "uncontested"
        chosen = model_names
        if len(kept) == 2:
            left, right = (detections[i].box for i in kept)
            if box_iou(left, right) > .45:
                status, reason = "ambiguous", "overlapping_candidates"
            elif forward and _side_by_side(left, right, image_size):
                status, reason, chosen = "paired", "forward_layout", layout_names
            elif forward and model_names == layout_names[::-1] and not conflict:
                reason = "opposite_model_order"
            elif conflict or len(set(model_names)) != 2:
                status, reason = "ambiguous", "no_trusted_pair"
        elif len(kept) > 2:
            status, reason = "ambiguous", "multiple_candidates"
        elif conflict:
            status, reason = "ambiguous", "conflicting_labels"

        if status != "ambiguous":
            final.update(zip(kept, chosen))
        diagnostics[color] = {
            "status": status, "reason": reason, "duplicates_removed": len(indices) - len(kept),
            "candidates": [dict(_record(detections[i]), final_name=final.get(i),
                                duplicates=[_record(detections[j]) for j in groups[i]]) for i in kept],
        }
    return [(d, final[i]) for i, d in enumerate(detections) if i in final], diagnostics
