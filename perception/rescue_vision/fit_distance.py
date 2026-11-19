"""Fit raw static distance errors and generate the dual-camera compensation config.

Run from perception/: python -m rescue_vision.fit_distance samples.json --output fit.json
Optional --config paths update only the distance_compensation section of each config.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from .geometry import RadialDistanceCompensation


def fit_error_samples(data):
    if (data.get("type") != "vision_error_samples" or data.get("version") != 1
            or data.get("units", {}).get("distance") != "cm"
            or data.get("units", {}).get("raw_distance") != "mm"):
        raise ValueError("Expected vision_error_samples v1 with cm truth and mm raw distances")
    positions, excluded = {}, []
    valid_count = 0
    for group in data["groups"]:
        x, y = group.get("x"), group.get("y")
        if not all(type(v) in (int, float) and math.isfinite(v) for v in (x, y)):
            raise ValueError("Ground-truth coordinates must be finite centimetres")
        truth = math.hypot(x, y)
        values = []
        for sample in group.get("samples", []):
            raw = sample.get("d_mm")
            if type(raw) in (int, float) and math.isfinite(raw) and 0 < raw <= 65535:
                values.append(raw / 10)
                valid_count += int(sample.get("valid") == 1)
        if truth <= 0 or not values:
            excluded.append({"group_id": group.get("id"), "truth_cm": truth,
                             "reason": "no_positive_numeric_samples" if not values else "zero_truth"})
            continue
        position = positions.setdefault(truth, {"values": [], "group_ids": []})
        position["values"].extend(values)
        position["group_ids"].append(group.get("id"))
    if len(positions) < 4:
        raise ValueError("At least four distinct sampled distances are required for model comparison")
    truth = np.array(sorted(positions), np.float64)
    samples = [np.asarray(positions[t]["values"], np.float64) for t in truth]
    means = np.array([v.mean() for v in samples])
    errors = means-truth
    candidates = []
    for degree in (1, 2):
        coefficients = np.polyfit(truth, errors, degree)
        residual = np.polyval(coefficients, truth)-errors
        cv = []
        for index in range(len(truth)):
            training = np.arange(len(truth)) != index
            held_out_fit = np.polyfit(truth[training], errors[training], degree)
            cv.append(np.polyval(held_out_fit, truth[index])-errors[index])
        total = float(np.sum((errors-errors.mean())**2))
        candidates.append({
            "degree": degree,
            "error_coefficients_cm": np.pad(coefficients, (3-len(coefficients), 0)).tolist(),
            "r2": float(1-np.sum(residual**2)/total) if total > 0 else None,
            "fit_rmse_cm": float(np.sqrt(np.mean(residual**2))),
            "leave_one_distance_out_rmse_cm": float(np.sqrt(np.mean(np.asarray(cv)**2))),
        })
    # Whole-distance holdouts keep correlated frames at the same position together.
    # Prefer the simpler model if its validation error is equal within numerical precision.
    selected = candidates[0]
    if candidates[1]["leave_one_distance_out_rmse_cm"] < selected["leave_one_distance_out_rmse_cm"]-1e-12:
        selected = candidates[1]
    endpoints = truth[[0, -1]] + np.polyval(selected["error_coefficients_cm"], truth[[0, -1]])
    config = {"enabled": True,
              "error_coefficients_cm": selected["error_coefficients_cm"],
              "true_range_cm": [float(truth[0]), float(truth[-1])],
              # Cover observed endpoint jitter as well as the model's endpoint means.
              "measured_range_cm": [float(min(endpoints.min(), min(v.min() for v in samples))),
                                    float(max(endpoints.max(), max(v.max() for v in samples)))]}
    compensation = RadialDistanceCompensation(config)
    points, all_corrected_errors = [], []
    for real, values, mean in zip(truth, samples, means):
        corrected = np.array([compensation.apply((0., float(v*10)))[0][1]/10 for v in values])
        all_corrected_errors.extend((corrected-real).tolist())
        points.append({"truth_cm": float(real), "group_ids": positions[real]["group_ids"],
                       "sample_count": len(values), "mean_measured_cm": float(mean),
                       "raw_bias_cm": float(mean-real),
                       "raw_sd_cm": float(values.std(ddof=1)) if len(values) > 1 else 0.,
                       "corrected_mean_cm": float(corrected.mean()),
                       "corrected_mean_bias_cm": float(corrected.mean()-real)})
    return {"version": 1, "source": data.get("source"),
            "distance_point_count": len(points), "sample_count": sum(map(len, samples)),
            "control_valid_sample_count": valid_count,
            "weighting": "equal_weight_per_distinct_true_distance",
            "selection": "lowest_leave_one_distance_out_rmse_cm; linear_on_tie",
            "excluded_groups": excluded, "candidates": candidates,
            "selected_degree": selected["degree"], "distance_compensation": config,
            "evaluation": {
                "raw_position_mean_rmse_cm": float(np.sqrt(np.mean(errors**2))),
                "corrected_position_mean_rmse_cm": float(np.sqrt(np.mean(
                    np.array([p["corrected_mean_bias_cm"] for p in points])**2))),
                "corrected_all_samples_rmse_cm": float(np.sqrt(np.mean(np.array(all_corrected_errors)**2))),
                "scope": "in_sample_replay; not an independent post-compensation vehicle test"},
            "points": points}


def main(argv=None):
    parser = argparse.ArgumentParser(description="拟合一次/二次测距误差并生成径向补偿配置")
    parser.add_argument("samples", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, action="append", default=[],
                        help="把选定模型写入指定双摄配置，可重复指定")
    args = parser.parse_args(argv)
    raw = args.samples.read_bytes()
    report = fit_error_samples(json.loads(raw.decode("utf-8-sig")))
    report.update(source_file=args.samples.name, source_sha256=hashlib.sha256(raw).hexdigest())
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    for path in args.config:
        config = json.loads(path.read_text(encoding="utf-8-sig"))
        config["distance_compensation"] = report["distance_compensation"]
        path.write_text(json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"selected_degree": report["selected_degree"],
                      "distance_point_count": report["distance_point_count"],
                      "sample_count": report["sample_count"], "candidates": report["candidates"],
                      "distance_compensation": report["distance_compensation"],
                      "evaluation": report["evaluation"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
