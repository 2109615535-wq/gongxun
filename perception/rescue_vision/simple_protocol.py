"""简易视觉协议，把识别结果整理成主控能够读取的一行串口数据。

1. CLASS_NAMES 定义模型类别与发送名称的对应关系。
2. SimpleObject 保存一个物体，SimpleObservation 保存一路相机的一次观测。
3. observations_from_snapshot() 从视觉程序的当前状态中提取这些数据。
4. encode() 将数据转换成串口发送的 bytes，decode() 用于反向解析和测试。

实际打开串口并发送数据的代码在 dashboard.py 的 ControllerLink 类中。
主控对应的接收代码在 stm32/vision_simple.c，完整格式见 SIMPLE_PROTOCOL.md。
"""
from dataclasses import dataclass
import math
import re

# 前面的点表示从同一个 rescue_vision 包中导入，复用已有的 CRC 校验算法。
from .protocol import crc16

# 字典的左边是 YOLO 模型内部类别，右边是双方约定的串口名称。
# 修改右边的名称时，主控接收代码中的名称列表也需要同步修改。
CLASS_NAMES = {
    "normal_material": "green",  # 普通物块
    "casualty": "orange",        # 伤员
    "hazard": "blue",            # 危险物块
    "core_material": "black",    # 核心物块
    "red_casualty_zone": "red_casualty_zone",
    "red_material_zone": "red_material_zone",
    "blue_casualty_zone": "blue_casualty_zone",
    "blue_material_zone": "blue_material_zone",
}
WIRE_NAMES = tuple(CLASS_NAMES.values())  # 取出字典右边的全部名称，供合法性检查使用。
MAX_OBJECTS = 12  # 一路相机的一帧最多携带12个物体。
MAX_LINE = 512  # 每行最多512字节，行末的换行符另算。
MAX_AGE_MS = 300  # 识别结果距离采集时刻达到300毫秒后，按过期处理。
# 依次表示有目标、空场、过期、故障、目标数量超限。
STATUSES = ("ok", "empty", "stale", "fault", "overflow")


# @dataclass 自动生成初始化等方法，便于把相关数据装在一个对象中。
# frozen=True 表示创建后字段不可重新赋值，减少传输过程中意外改数据的机会。
@dataclass(frozen=True)
class SimpleObject:
    """一个物体的信息，例如 SimpleObject("green", 450, True, False)。"""

    # 冒号后的 str/int/bool 是类型提示，分别表示字符串、整数、真假值。
    # 等号后是默认值，因此 SimpleObject("black") 也能创建一个距离未知的物体。
    kind: str  # 线上类别名，例如 green。
    distance_mm: int = -1  # 地面径向距离，单位毫米，-1表示无法提供距离。
    distance_valid: bool = False  # True表示距离满足现有位置验证及新鲜度条件。
    in_roi: bool = False  # True表示位于下视夹爪核验区域，夹持成功需另外判断。
    angle: float | None = None  # 地面方位角（度），右正左负，None表示无法提供。


@dataclass(frozen=True)
class SimpleObservation:
    """一路相机的一帧结果，可以包含多个 SimpleObject。"""

    source: str  # upper上视、lower下视。
    frame_id: int  # 实际参与检测的相机帧编号，主控用它识别重复旧帧。
    age_ms: int  # 采集到发送快照之间经过的毫秒数，未知为-1。
    status: str  # 取值来自上面的 STATUSES。
    objects: tuple[SimpleObject, ...] = ()  # tuple是元组，()表示当前没有物体。


def _integer(value, low, high):
    """检查数值确实是整数，而且落在闭区间 [low, high] 内。"""
    # Python中bool属于int的子类，使用type(...) is int可避免把True当作数字1。
    return type(value) is int and low <= value <= high


def validate(observation):
    """检查一帧数据是否符合协议，发现问题时用 ValueError 报错。"""
    # 先检查相机、编号、年龄、状态和目标数量。
    # 0xFFFFFFFF是十六进制写法，等于32位无符号整数的最大值4294967295。
    if (observation.source not in ("upper", "lower") or
            not _integer(observation.frame_id, 0, 0xFFFFFFFF) or
            not _integer(observation.age_ms, -1, 65535) or
            observation.status not in STATUSES or len(observation.objects) > MAX_OBJECTS):
        raise ValueError("Invalid simple observation header")
    if observation.status in ("ok", "empty"):
        # 正常结果必须足够新鲜，且ok应有物体，empty应没有物体。
        if not 0 <= observation.age_ms < MAX_AGE_MS:
            raise ValueError("Normal observations must be fresh")
        if bool(observation.objects) != (observation.status == "ok"):
            raise ValueError("Object count differs from status")
    elif observation.objects:
        raise ValueError("Invalid observations cannot carry objects")
    for obj in observation.objects:
        # 再逐个检查物体，距离为-1时不可同时声明距离有效。
        if (obj.kind not in WIRE_NAMES or not _integer(obj.distance_mm, -1, 65535) or
                type(obj.distance_valid) is not bool or type(obj.in_roi) is not bool or
                (obj.angle is not None and
                 (type(obj.angle) not in (int, float) or not math.isfinite(obj.angle) or
                  not -180 <= obj.angle <= 180)) or
                (obj.distance_valid and obj.distance_mm < 0) or
                (observation.source == "upper" and obj.in_roi)):
            raise ValueError("Invalid simple object")


def encode(observation, *, version=2):
    """把 SimpleObservation 转成一条可交给串口 write() 的字节串。"""
    validate(observation)
    if type(version) is not int or version not in (1, 2):
        raise ValueError("Unsupported simple protocol version")
    # RV2追加angle；RV1保持原来的4字段布局，供旧接收端显式选择。
    fields = [f"RV{version}", observation.source, str(observation.frame_id), str(observation.age_ms),
              observation.status, str(len(observation.objects))]
    for obj in observation.objects:
        # int(True)得到1，int(False)得到0，便于C语言接收。
        fields.extend((obj.kind, str(obj.distance_mm), str(int(obj.distance_valid)), str(int(obj.in_roi))))
        if version == 2:
            fields.append("NA" if obj.angle is None else f"{obj.angle:.2f}")
    # join用逗号连接字符串，encode("ascii")将文字转换为实际发送的字节。
    body = ",".join(fields).encode("ascii")
    # CRC用于发现传输中的数据损坏，04X表示补足4位的大写十六进制数。
    packet = body + f"*{crc16(body):04X}".encode("ascii")
    if len(packet) > MAX_LINE:
        raise ValueError("Simple observation line too long")
    return packet + b"\n"  # b表示bytes字节串，\n让接收端知道这一行已经结束。


def _number(text, low, high):
    """把一个数字字段转换成整数，同时检查文字格式和数值范围。"""
    # fullmatch要求整个字符串符合规则，只接受十进制数字或特殊值-1。
    if not re.fullmatch(r"(?:[0-9]+|-1)", text):
        raise ValueError("Invalid integer token")
    number = int(text)
    if not low <= number <= high:
        raise ValueError("Integer outside wire range")
    return number


def _angle(text):
    if text == "NA":
        return None
    if not re.fullmatch(r"-?[0-9]+(?:\.[0-9]{1,2})?", text):
        raise ValueError("Invalid angle token")
    value = float(text)
    if not math.isfinite(value) or not -180 <= value <= 180:
        raise ValueError("Angle outside wire range")
    return value


def decode(packet):
    """把完整的一行串口数据还原成对象，供电脑检查和协议互通测试使用。"""
    # 切片[:-1]去掉最后一个字节，兼容常见的LF和CRLF两种换行方式。
    if packet.endswith(b"\n"):
        packet = packet[:-1]
    if packet.endswith(b"\r"):
        packet = packet[:-1]
    if len(packet) > MAX_LINE or packet.count(b"*") != 1:
        raise ValueError("Invalid simple line")
    body, checksum = packet.split(b"*")
    # 收到的CRC必须与重新计算的CRC相同，随后才继续解析字段。
    if not re.fullmatch(b"[0-9a-fA-F]{4}", checksum) or crc16(body) != int(checksum, 16):
        raise ValueError("CRC mismatch")
    try:
        fields = body.decode("ascii").split(",")
    except UnicodeDecodeError as exc:
        raise ValueError("Expected ASCII") from exc
    if len(fields) < 6 or fields[0] not in ("RV1", "RV2"):
        raise ValueError("Invalid simple header")
    item_fields = 5 if fields[0] == "RV2" else 4
    count = _number(fields[5], 0, MAX_OBJECTS)
    if len(fields) != 6 + item_fields * count:
        raise ValueError("Object count differs from line length")
    # RV2每物体5项；旧RV1每物体4项且angle为None。
    objects = tuple(SimpleObject(fields[i], _number(fields[i+1], -1, 65535),
                                 bool(_number(fields[i+2], 0, 1)), bool(_number(fields[i+3], 0, 1)),
                                 _angle(fields[i+4]) if item_fields == 5 else None)
                    for i in range(6, len(fields), item_fields))
    result = SimpleObservation(fields[1], _number(fields[2], 0, 0xFFFFFFFF),
                               _number(fields[3], -1, 65535), fields[4], objects)
    validate(result)
    return result


def observations_from_snapshot(snapshot):
    """从视觉状态快照中取出上下视结果，返回两条准备发送的观测。

    snapshot是DashboardState.snapshot()返回的字典，其中保存两路相机的当前状态。
    这里读取已有的检测结果和距离，整理成协议对象，供encode()随后打包。
    """
    observations = []
    for source in ("upper", "lower"):
        camera = snapshot["cameras"][source]
        # get("字段", 默认值)在字段缺失时返回默认值，适用于相机尚未出图的情况。
        sequence = camera.get("sequence", 0)
        # 按位与0xFFFFFFFF保留低32位，让编号超过上限后回绕到0。
        frame_id = (sequence & 0xFFFFFFFF) if type(sequence) is int and sequence >= 0 else 0
        age = camera.get("age_ms")
        # isfinite排除无穷大和NaN等异常数值。ceil向上取整，避免把结果年龄算小。
        age_ms = min(65535, math.ceil(age)) if isinstance(age, (int, float)) and math.isfinite(age) and age >= 0 else -1
        # 先判断本路结果是否可用，过期、未知暗区、故障和回放都有独立处理。
        if camera.get("status") == "STALE" or age_ms >= MAX_AGE_MS:
            status = "stale"
        elif camera.get("status") != "OK" or camera.get("gripper", {}).get("unknown_regions"):
            status = "fault"
        elif age_ms < 0 or camera.get("mode") == "video":
            status = "fault"
        else:
            status = "ok"
        objects = []
        if status == "ok":
            for row in camera.get("detections", []):
                name = row.get("name")
                # continue跳过当前物体，继续看下一个，夹爪本体无需作为物资发送。
                if name in ("gripper", "gripper_closed"):
                    continue
                if name not in CLASS_NAMES:
                    status = "fault"
                    # break结束本路物体循环，遇到未知类别时将整路结果标为不可用。
                    break
                point = row.get("position_mm") or row.get("display_position_mm") or {}
                # ground_range由现有测距模块计算，等于sqrt(x_right² + y_forward²)。
                distance = point.get("ground_range")
                distance_mm = (round(distance) if type(distance) in (int, float) and
                               math.isfinite(distance) and 0 <= distance <= 65535 else -1)
                distance_valid = bool(distance_mm >= 0 and row.get("position_mm") and
                                      row.get("position_valid", False) and
                                      age_ms < 150)
                # 使用已有地面坐标，不能用图像框中心偏移代替地面方位。
                x, y = point.get("x_right"), point.get("y_forward")
                angle = None
                if (type(x) in (int, float) and type(y) in (int, float) and
                        math.isfinite(x) and math.isfinite(y) and (x != 0 or y != 0)):
                    angle = round(math.degrees(math.atan2(x, y)), 2)
                # append在列表末尾加入一个物体，同类多个物体也会分别保留。
                objects.append(SimpleObject(CLASS_NAMES[name], distance_mm, distance_valid,
                                            source == "lower" and bool(row.get("inside_gripper_roi", False)), angle))
            if len(objects) > MAX_OBJECTS:
                # 数量超限时整帧标记overflow，避免截断列表时遗漏危险物体。
                status = "overflow"
            elif status == "ok" and not objects:
                status = "empty"
        # 正常结果携带物体列表，其他状态携带空元组，主控据此清除旧结果。
        observations.append(SimpleObservation(source, frame_id, age_ms, status,
                                               tuple(objects) if status in ("ok", "empty") else ()))
    return tuple(observations)
