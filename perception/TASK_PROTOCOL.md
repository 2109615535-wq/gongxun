# 双向视觉任务协议 rescue_json_v2

用于STM32下发任务与实际状态，视觉返回观测和核验。本次只实现视觉侧；现有 `stm32/vision_simple.c/.h` 只能解析RV1/RV2，不能解析本协议，主控完成适配后再启用。

## 线格式

115200/8N1、3.3V TTL、TX/RX交叉、GND共地。一条ASCII消息一行：

```text
{"msg_type":"task",...}*XXXX\n
```

`crc16` 位于星号后，采用已有CRC16/CCITT-FALSE：poly=0x1021、init=0xFFFF、不反射、xorout=0；`123456789`为`29B1`。校验覆盖原始JSON正文字节，不含星号、CRC和换行，接收端不应重新序列化后校验。CRC为四位十六进制，支持LF/CRLF，不含换行最多4096字节。正文不重复放 `crc16` 键；Python `decode()` 将行尾CRC还原为该字段。拒绝重复JSON键、无CRC、格式错误和超长行。

两端共用 `msg_type/session_id/task_id/seq/crc16`。视觉结果另带 `state_id/phase`。会话/任务编号为1～64个字母、数字、下划线或连字符；seq为u32，两方向独立递增并允许回绕。重复、倒序或前跳达到半个u32范围的任务拒绝且不刷新心跳；旧会话未超时则拒绝不同会话。错误行使任务失效，后续合法新序号恢复后需重新采图与连续帧确认。

## STM32 → 香橙派

以下为完整正文示例，实际发送必须附CRC：

```json
{"msg_type":"task","session_id":"boot-1","task_id":"trip-1","seq":1,"state_id":1,"phase":"search","team_color":"red","allowed_kinds":["green"],"locked_target_id":0,"expected_kind":"green","expected_count":1,"mechanism_state":"open","yaw_deg":15.0}
```

- `state_id`：0启动、1首个普通物资、2普通/核心物资、3伤员。视觉只接收此状态，不自行切换。
- `phase`：`idle`等待、`scan`扫描、`search`搜索、`align`对准、`approach`接近、`verify_before_transfer`转运前核验、`verify_after_transfer`转运后核验、`near_dock`近仓、`deliver`投放、`release`释放、`retreat`后退、`delivery_check`投放确认。
- `team_color`：red或blue。
- `allowed_kinds`：复用现有线上名称green普通物资、black核心物资、orange伤员。状态0只能为空；状态1只能选green；状态2允许green/black；状态3只能选orange。禁止选择危险物或区域。
- `locked_target_id`：视觉返回的u32编号，0表示未锁定；非0时只能选择此编号，丢失后不自动改选同类目标。
- `expected_kind`：必须属于allowed_kinds。等待/扫描/搜索/对准/接近时可为null；核验、近仓、投放相关阶段必须明确。未指定时搜索允许类别。
- `expected_count`：当前固定为1，其他值拒绝。
- `mechanism_state`：实际机构状态，使用 `unknown/open/closed/opening/closing/at_position/fault`。未知、故障或运动中不能核验放行，视觉检测不能代替此实际状态。
- `yaw_deg`：-180～180度，未知可为null，用于扫描记录关联。视觉只在该航向消息不晚于图像采集时附上它；不承诺精确姿态同步。

上述字段均需发送。建议每100ms用新seq发送完整心跳，当前超时为1000ms。只有seq/yaw改变时保留连续确认；任务、状态、阶段、本方颜色、锁号、期望/允许类别或机构变化时重新累计。阶段切换前采集的图像不参与新阶段。实时接近沿用车体坐标，不增加robot_x_mm/robot_y_mm或全局地图。

## 香橙派 → STM32

网页 `/api/status` 的 `task_results` 提供未封包结果，串口发送时增加独立seq和CRC。msg_type分为以下五种，观测按上下视分别发送。

### status：接收确认与运行状态

`ack_seq`为最后接受的主控序号；`accepted_state_id/accepted_phase`为已接受阶段；尚无任务时均为null。`vision_ready=1`要求双路实时图像、当前任务帧、模型与正式标定满足要求。`fault_code`取值：`none/controller_lost/model_fault/calibration_fault/calibration_unverified/processing_fault/camera_fault/replay_input/task_frame_pending/result_stale`。`danger_flag`汇总当前阶段是否禁止继续，1为禁止。ACK不表示允许动作。

### observation：目标观测

包含 `source`（upper/lower）、`frame_id`、`age_ms`、`status`、`objects`、`yaw_deg`、`danger_flag`。status复用 `ok/empty/stale/fault/overflow`，未知年龄为-1，每帧最多12个目标；无效或超限不截断后冒充正常结果。

每个objects条目包含：

- `target_id`：关联编号。
- `kind`：green、black、orange、blue及四类红蓝物资/伤员区，模型内部类别不改。
- `distance_mm/distance_valid`：沿用地面径向距离与位置有效条件，无距离为-1；有效距离要求小于150ms。
- `angle`：地面方位角，单位度，右正左负，无角度为字符串NA。0度是合法正前方角度。
- `in_roi`：是否满足下视机构ROI覆盖阈值，上视恒为0；不代表核验成功。

不发送angle_valid，由主控根据角度非NA、距离有效和累计年龄计算。

关联使用同类别、一对一IoU连续帧匹配，上视奇数、下视偶数。目标消失、间隔超时、故障、歧义或切换转运任务时清空关联，同进程不复用旧编号。阶段改变保留连续目标编号，但重置核验确认。未知外部锁号不能分给新出现目标。

**目前是单相机连续关联，尚无跨相机/全局身份识别。** 上视锁号不能直接认作下视物体；无法证明锁号关联时，下视返回locked_target_lost并禁止继续。不能仅凭同类就自动替换锁号。视觉进程重启后需要重建会话、观测和锁定，不能沿用旧编号。

### verification：转运区域核验

包含 `source=lower/frame_id/age_ms/result_valid/roi_count/roi_kinds/danger_flag/danger_reason`。roi_count是实际物体数量，区别于网页内部连续帧count；同类多物体分别计数。与ROI相交者也计数，部分越界或遮挡时禁止继续；夹爪与区域标识不算物资。未知暗区不能准确计数，此时结果无效。

只有当前核验阶段、ROI已验证、机构实际状态明确且不在运动中、两路图像新鲜并属于当前任务、至少连续3帧单物体类别相符、锁号符合要求且无危险/未知目标时，输出 `result_valid=1,danger_flag=0`。多物体、错类或空区可形成有效的禁止判断，即result_valid=1但danger_flag=1；无效结果始终danger_flag=1。

danger_reason包含 `multiple_objects/kind_mismatch/hazard/unknown_object/occluded/roi_unverified/empty_roi/confirming/expected_kind_unknown/locked_target_lost/mechanism_unknown/wrong_phase/hazard_or_unknown_object` 及运行故障码，none表示本阶段核验通过。

### dock：近仓对准

包含 `zone_kind/dock_valid/dock_lateral_mm/dock_gap_mm/dock_yaw_deg/danger_flag/danger_reason`。zone_kind由本方颜色与物资/伤员类别确定，是任务要求，不是已看见正确区域的证明。

dock_lateral_mm是库口横向误差，dock_gap_mm是到内侧参考边界的前向间隙，dock_yaw_deg是相对库口法线的角度误差，**不能用目标angle或车体yaw替代**。

当前尚无经过验证的库口边界、法线和机构参考点解算。本版发送dock_valid=0，三项姿态值null，danger_flag=1，原因dock_geometry_unavailable。

### delivery：入仓与投放确认

包含 `zone_kind/entry_allowed/object_inside/delivery_confirmed/result_valid/danger_flag/danger_reason`。当前缺少完整物体入区、释放退出后留存的可靠几何与时序核验，因此result_valid与三个放行/成功标志始终为0，danger_flag=1，原因delivery_geometry_unavailable。接口已接入，成功判断算法尚未实现。

后续需以同任务的完整物体轮廓、正确区域边界、释放与退出后的新鲜连续图像判断，不能由检测到区域推断投放成功。视觉返回delivery_confirmed，由STM32核对任务后决定解除首轮限制和进入状态2，没有额外“进入状态2”命令。

## 时效与当前限制

未知、失联、ROI未验证、回放、标定未验证、阶段不符或过期时禁止继续，放行/成功标志为0。失联不恢复网页联调任务；主控必须核对task_id/state_id/phase。

发送时重新取快照，轮流发送状态、上视、下视和当前阶段结果，并按字节预算限速；115200下12目标长JSON不能保证10Hz。age_ms为发送前软件采集年龄，**不含曝光、相机缓存和串口传输时间**。主控必须累计串口传输与接收后的时间：达到150ms撤销距离/角度有效，达到300ms撤销核验结果，自身通信超时也应禁止继续。

现有配置ROI validated=false，上视为局部检查、下视为拟合标定，因此实际运行会报告未验证，本次不自动改验证标志。库口/入仓确认和跨相机交接仍需实车数据和算法补充。

离线回归，在perception目录运行：

```text
python -m unittest discover -s tests -v
```
