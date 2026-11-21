"""Check a real green-block delivery using the running vision service."""

import argparse
from contextlib import closing
import http.client
import json
import time

from .simple_protocol import encode, observations_from_snapshot


CAMERAS = ("upper", "lower")
STAGES = {
    "start": ("起点", "小车停在起点，夹爪为空，绿块已放在车前 50 cm 内"),
    "grip": ("夹取", "小车已夹住绿块并停稳，下视仍可看到夹爪里的绿块"),
    "zone": ("接近物资区", "小车携带绿块停在本方物资区前，安全区完整可见"),
    "delivery": ("放下确认", "小车已松爪、退开并停稳，绿块和物资区仍在画面中"),
}


def read_status(address, port, source_ip):
    source = (source_ip, 0) if source_ip else None
    with closing(http.client.HTTPConnection(address, port, timeout=3, source_address=source)) as connection:
        connection.request("GET", "/api/status")
        response = connection.getresponse()
        if response.status != 200:
            raise OSError(f"视觉服务返回 HTTP {response.status}")
        return json.load(response)


def has_position(obj):
    return obj.distance_mm >= 0 and obj.angle is not None and obj.x_mm is not None and obj.y_mm is not None


def check_stage(stage, snapshot, zone):
    """Return the observation that supports the current physical milestone."""
    cameras = snapshot["cameras"]
    observations = {sample.source: sample for sample in observations_from_snapshot(snapshot)}
    upper, lower = (observations[role] for role in CAMERAS)
    if snapshot["model"]["status"] != "READY" or snapshot["link"]["status"] != "CONNECTED":
        return None, "视觉模型或主控串口未就绪"
    if any(cameras[role].get("mode") != "camera" or cameras[role]["status"] != "OK" or
           observations[role].status not in ("ok", "empty") or
           not 0 <= observations[role].age_ms < 150 for role in CAMERAS):
        return None, "等待上下视新的实时帧"
    if any(cameras[role].get("hazard_blocked") for role in CAMERAS):
        return None, "画面中有危险物块或未知目标"

    grip = cameras["lower"]["gripper"]
    grip_count = grip.get("roi_count")
    if not snapshot["gripper_config"]["validated"] or grip_count is None:
        return None, "下视夹爪 ROI 未就绪"
    upper_green = [obj for obj in upper.objects if obj.kind == "green"]
    lower_green = [obj for obj in lower.objects if obj.kind == "green"]
    if len(upper_green) > 1 or len(lower_green) > 1:
        return None, "测试场景需要只保留一个绿块"

    if stage == "start":
        if grip_count != 0:
            return None, "起步时夹爪区域应为空"
        if len(upper_green) == 1:
            target = upper_green[0]
            if has_position(target) and target.distance_valid and 0 < target.distance_mm <= 500:
                return upper, f"绿块位置 X={target.x_mm} mm、Y={target.y_mm} mm，距离 {target.distance_mm} mm"
        return None, "上视需要看到 50 cm 内有位置的绿块"

    if stage == "grip":
        row = next((item for item in cameras["lower"]["detections"]
                    if item["name"] == "normal_material" and item.get("inside_gripper_roi")), None)
        if (grip_count == 1 and grip.get("roi_kinds") == ["normal_material"] and
                row and not row.get("roi_clipped") and not row.get("occluded") and
                len(lower_green) == 1 and lower_green[0].in_roi):
            return lower, "下视夹爪 ROI 内有一个完整的绿块"
        return None, "等待下视看到夹爪 ROI 内的单个完整绿块"

    if stage == "zone":
        for observation in (upper, lower):
            for obj in observation.objects:
                if obj.kind == zone and has_position(obj):
                    return observation, f"本方物资区中心 X={obj.x_mm} mm、Y={obj.y_mm} mm"
        return None, "等待本方物资区中心坐标"

    if grip_count != 0:
        return None, "松爪退开后仍有物体位于夹爪 ROI"
    for observation in (upper, lower):
        if any(obj.kind == "green" and obj.zone_kind == zone and obj.inside_valid and
               obj.object_inside for obj in observation.objects):
            return observation, "绿块留在本方物资区，下视夹爪 ROI 为空"
    return None, "等待确认绿块完整留在本方物资区"


def wait_for_stage(args, stage):
    deadline = time.monotonic() + args.timeout
    previous = None
    consecutive = 0
    source = None
    message = "等待两路相机的下一帧"
    while time.monotonic() < deadline:
        try:
            snapshot = read_status(args.address, args.port, args.source_ip)
            frames = tuple(snapshot["cameras"][role]["sequence"] for role in CAMERAS)
            if previous is None:
                previous = frames
            elif all(current != earlier for current, earlier in zip(frames, previous)):
                previous = frames
                observation, message = check_stage(stage, snapshot, f"{args.team}_material_zone")
                if observation is None:
                    consecutive = 0
                else:
                    consecutive = consecutive + 1 if source == observation.source else 1
                    source = observation.source
                    if consecutive == 3:
                        return observation, message
        except (OSError, ValueError, KeyError, http.client.HTTPException) as error:
            consecutive = 0
            message = str(error)
        time.sleep(.06)
    return None, message


def main():
    parser = argparse.ArgumentParser(description="实车绿块投放的视觉验收。操作员执行行驶和夹爪动作。")
    parser.add_argument("--team", choices=("red", "blue"), required=True, help="本方物资安全区颜色")
    parser.add_argument("--address", default="10.163.0.250", help="香橙派地址")
    parser.add_argument("--port", type=int, default=8080, help="视觉监看端口")
    parser.add_argument("--source-ip", help="电脑用于连接香橙派的 WLAN 地址")
    parser.add_argument("--stage", choices=STAGES, help="单独复查一个已完成的阶段")
    parser.add_argument("--timeout", type=float, default=8.0, help="每阶段等待视觉观测的秒数")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout 必须大于零")

    for stage in (args.stage,) if args.stage else STAGES:
        label, instruction = STAGES[stage]
        if args.stage is None:
            input(f"[{label}] {instruction}后按回车采样。")
        observation, detail = wait_for_stage(args, stage)
        if observation is None:
            print(f"[{label}] 未通过 · {detail}", flush=True)
            return 1
        print(f"[{label}] 通过（连续三组新帧）· {detail}", flush=True)
        print("RV3 样帧 " + encode(observation, version=3).decode("ascii").rstrip(), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
