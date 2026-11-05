"""单相机连续帧一对一关联；歧义/丢失即换号，不假定跨相机或全局身份。"""
from .detector import box_iou


class TargetTracker:
    def __init__(self, role, iou):
        self.next_id = 1 if role == "upper" else 2
        self.iou = iou
        self.rows = []

    def reset(self):
        self.rows = []  # 保留计数器，禁止旧锁定编号被新目标复用。

    def update(self, rows, *, locked_target_id=0):
        matches = [[old for old in self.rows if old["name"] == row["name"] and
                    box_iou(old["box"], row["box"]) >= self.iou] for row in rows]
        for row, candidates in zip(rows, matches):
            candidate = candidates[0] if len(candidates) == 1 else None
            unique = candidate is not None and sum(any(old is candidate for old in group)
                                                   for group in matches) == 1
            if unique:
                row["target_id"] = candidate["target_id"]
            else:
                if self.next_id == locked_target_id:
                    self.next_id += 2  # 未在上一帧出现的外部锁号不能绑定新目标。
                if self.next_id > 0xFFFFFFFF:
                    raise ValueError("Target IDs exhausted; restart vision/controller session")
                row["target_id"] = self.next_id
                self.next_id += 2
        self.rows = [dict(row) for row in rows]
