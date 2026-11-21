# 智能救援视觉端（RV1 至 RV4）
***本文规定智能救援项目中，香橙派视觉端与 STM32 主控端之间的 UART 通信接口。香橙派负责识别和核验，STM32 负责运动控制与任务状态切换。本文不包含电机驱动和底层运动控制。***

## 1. 职责分配与相关文件
### STM32 负责下发任务与实际机构状态，检查视觉结果是否有效，再决定转向、前进、机构动作和状态切换

#### STM32端负责接受与发送信息的文件
- [stm32/vision_simple.h](stm32/vision_simple.h)：类别、观测结构及 C 接收接口。
- [stm32/vision_simple.c](stm32/vision_simple.c)：CRC校验、逐行解析、双路独立的新帧检查和超时处理。
- [stm32/SIMPLE_PROTOCOL.md](stm32/SIMPLE_PROTOCOL.md)：各版本字段、校验样例和接入说明。

### 香橙派负责接收任务要求，返回目标类别、距离、角度，以及当前阶段的核验结果。

#### 香橙派端接收与发送信息的文件

- [rescue_vision/dashboard.py](rescue_vision/dashboard.py) 启动双视网页、相机、推理和串口服务。
- [rescue_vision/dual.py](rescue_vision/dual.py) 处理双视目标、地面坐标、夹爪 ROI 核验和安全区判断。
- [rescue_vision/simple_protocol.py](rescue_vision/simple_protocol.py) 构造 RV1 至 RV4 观测。
- [rescue_vision/task_protocol.py](rescue_vision/task_protocol.py) 处理双向任务与核验报文。

## 2. 硬件连接与UART参数（双端）

采用3.3V TTL，视觉TX接主控所选UART的RX，双方GND共地。RV1 至 RV4 由视觉端单向发送观测。使用双向任务协议时再接入主控 TX 到香橙派 RX。

### 硬件连接
| 香橙派 (Orange Pi) | 连线方向 | STM32 (UART6) |
| :--- | :---: | :--- |
| 物理13脚 (`UART6_TX`) | `→` | `RX` |
| 物理11脚 (`UART6_RX`) | `←` | `TX` |
| 6脚 (`GND`) | `—` | `GND` |

### uart参数
```md
包头：RV4
波特率：115200
数据位：8
校验位：无
停止位：1
硬件流控：无
完整数据实例：RV4,upper,42,86,ok,1,green,450,1,0,30.00,225,390,red_material_zone,0,0,1*4AE2\n
```

## 3. 主控端UART初始化参数

统一采用115200波特率、8位数据、1位停止位、无校验、无硬件流控。UART6 联调用 STM32 的 PC6（TX）和 PC7（RX）。USART3（PC10 TX、PC11 RX）可按实际引脚分配使用。UART5 已用于电机调试，USART2 已用于 IMU。

以下仅以USART3展示参数，不是覆盖真实主控的补丁：

```c
huart3.Instance = USART3;
huart3.Init.BaudRate = 115200;
huart3.Init.WordLength = UART_WORDLENGTH_8B;
huart3.Init.StopBits = UART_STOPBITS_1;
huart3.Init.Parity = UART_PARITY_NONE;
huart3.Init.Mode = UART_MODE_TX_RX;
huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart3.Init.OverSampling = UART_OVERSAMPLING_16;
if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
}
```

GPIO选择对应USART复用功能，开启接收中断或DMA。复用现有HAL接收/错误回调增加串口分发，不能重复定义同名全局回调。接收出错和观测失效时的运动撤销由主控集成代码处理。

## 4. 双向通信字段与任务约定

以下约定用于任务、实际执行状态与视觉结果的双向传输。具体线上格式和当前实现情况见 [TASK_PROTOCOL.md](TASK_PROTOCOL.md)。

### 两端消息共用

- `msg_type`：消息类型，区分状态、目标观测、核验和投放结果。
- `session_id`：本次启动会话。
- `task_id`：本次转运任务编号。
- `seq`：消息序号。
- `crc16`：校验值。

### STM32 → 香橙派

发送任务要求和实际执行状态。

- `state_id`：大阶段，0启动、1首个普通物资、2普通/核心物资、3伤员。
- `phase`：动作子阶段，例如扫描、搜索、对准、接近、转运前核验、转运后核验、近仓、投放、释放、后退、投放确认。
- `team_color`：本方红色或蓝色。
- `allowed_kinds`：当前允许选择的物资类别。
- `locked_target_id`：已选定并锁定的目标。
- `expected_kind`：本次核验应当看到的类别。
- `expected_count`：本次允许的物体数量，当前方案均为1。
- `mechanism_state`：机构实际开合、到位或故障状态。
- `yaw_deg`：当前车体航向，供扫描记录关联使用。

如果后续确实需要视觉建立全局目标地图，再增加 `robot_x_mm`、`robot_y_mm` 和对应姿态时间，实时接近阶段不必为了建图而增加这些数据。

### 香橙派 → STM32

发送观测和核验结果。

1. **接收确认与运行状态**
   - `ack_seq`：已接受的主控消息序号。
   - `accepted_state_id`、`accepted_phase`：视觉当前执行的阶段。
   - `vision_ready`：视觉是否就绪。
   - `fault_code`：相机、模型、标定或处理故障。
2. **目标观测**
   - 帧头包含 `source`（`upper` 上视或 `lower` 下视）、`frame_id`（图像帧号）、`age_ms`（观测年龄）、`status`（`ok`、`empty`、`stale`、`fault`、`overflow`）。简易协议的 `count` 为本帧目标数，最多 12 个。
   - 每个目标包含 `kind`（类别）、`distance_mm`（径向距离，未知为 `-1`）、`distance_valid`（位置有效位）和 `in_roi`（下视夹爪 ROI 标记，上视为 `0`）。
   - RV2 增加 `angle`。方位角单位为度，右正左负，未知为 `NA`。主控根据角度是否存在、距离有效位和观测年龄计算 `angle_valid`。
   - RV3 增加 `x_mm`、`y_mm`、`zone_kind`、`inside_valid`、`object_inside`。X 向右、Y 向前，单位 mm。入仓结果按安全区类别和连续三帧核验。
   - RV4 在每个目标的末尾增加 `in_safety_zone`。同一相机当前帧中，任一红蓝物资区或伤员区 ROI 与物块 ROI 的重叠面积达到物块 ROI 面积的 80% 时为 `1`，其余为 `0`。各安全区独立计算，安全区目标本身填 `0`。主控搜索、追踪和夹取时跳过标记为 `1` 的物块。
   - 双向任务 JSON `rescue_json_v2` 在 `objects` 中回传上述目标数据，并用 `target_id` 关联同一相机连续帧中的目标。

3. **转运区域核验**
   - `result_valid`：本次判断是否有效。
   - `roi_count`：核验区域内物体数量。
   - `roi_kinds`：区域内各物体类别。
   - `danger_flag`：0允许继续，1禁止继续。
   - `danger_reason`：多物体、类别不符、危险目标、未知物体、遮挡、ROI未验证或结果过期等原因。
4. **近仓对准**
   - `zone_kind`：本方物资区或伤员区的具体类别。
   - `dock_valid`：库口姿态是否可靠。
   - `dock_lateral_mm`：库口横向对准误差。
   - `dock_gap_mm`：到库口内侧参考边界的前向间隙。
   - `dock_yaw_deg`：车体相对库口法线的角度误差。

   `dock_yaw_deg` 与目标方位 `angle` 含义不同，不能混用。

5. **入仓与投放确认**
   - `entry_allowed`：是否满足继续推送入库的条件。
   - `object_inside`：本次物体是否已经全部进入正确区域。
   - `delivery_confirmed`：释放、退出后，是否确认本次物体稳定留在正确区域。
   - `result_valid`：上述判断是否有效。

所有视觉结果都应关联当前 `task_id` 和动作阶段。**未知、失联或结果无效时，`danger_flag=1`，其余放行、成功标志均为0。**

不需要另外设计“进入状态2指令”。香橙派返回 `delivery_confirmed`，由STM32核对任务后决定是否解除首轮限制并切换状态。

### 物体类别字段 `kind`

物块使用四种线上名称，括号内为 weitiao2 模型名称与 ID。

- 普通物块为 `green`（`normal_material`，0）。
- 核心物块为 `black`（`core_material`，1）。
- 伤员物块为 `orange`（`casualty`，2）。
- 危险物块为 `blue`（`hazard`，3）。

四类安全区的线上名称和模型名称相同。

- 红方物资安全区为 `red_material_zone`（5）。
- 蓝方物资安全区为 `blue_material_zone`（6）。
- 红方伤员安全区为 `red_casualty_zone`（7）。
- 蓝方伤员安全区为 `blue_casualty_zone`（8）。

双视配置过滤 `gripper`（4）和 `gripper_closed`（9）。同类多个目标各占一个条目。类别顺序和字段示例见[简易协议](stm32/SIMPLE_PROTOCOL.md)。

## 5. 双视启动与 UART6 / OLED 联调

板端服务当前使用 `vision_simple_v3`。兼容模式对区内普通、核心和伤员物块隐藏抓取定位和夹爪 ROI 命中，并继续发送类别及入仓结果。危险物块继续参与危险判断。主控更新至支持 RV4 的固件后，可在香橙派选择以下启动参数。

```bash
sh tools/run_dual_orangepi.sh --controller-port /dev/ttyS6 --controller-protocol vision_simple_v4
```

Windows 端在 `perception/` 运行，替换相机编号以匹配实际设备。

```powershell
python -m rescue_vision.dashboard --cameras --upper-source 1 --lower-source 2
```

从电脑访问香橙派地址的 8080 端口。此次部署验证使用 `10.163.0.250:8080`。网页显示上下视画面、模型和串口状态，`/api/status` 返回观测详情。

- 香橙派物理 13 脚为 `UART6_M1_TX / GPIO1_A1`，接 STM32 的 `PC7 / USART6_RX`。物理 6 脚接 GND。
- 板端通过 `/boot/orangepiEnv.txt` 的 `overlays=uart6-m1` 启用 `/dev/ttyS6`。
- `rescue-vision.service` 已在板端启用。串口按服务配置或启动参数选择。板端运行环境、相机和校验说明见 [START_HERE.md](START_HERE.md)。

```bash
systemctl is-enabled rescue-vision.service
systemctl is-active rescue-vision.service
journalctl -u rescue-vision.service -n 50 --no-pager
```

### STM32 接收与 OLED 联调

1. 初始化时调用 `VSP_Init` 并启动 USART6 接收。
2. UART 中断将字节和到达时间放入队列，主循环或调度任务调用 `VSP_Feed`。
3. 周期调用 `VSP_Get` 读取 `VSP_UPPER` 与 `VSP_LOWER`。UART6 联调固件按上视和下视分屏显示 green 的 X、Y、angle 和 distance。X 向右为正，Y 向前为正，距离单位为 mm，角度单位为度。
4. 暂停视觉发送，核对 OLED 超时清除行为，再进行断电上电测试。

OLED 联调固件和 systemd 配置示例见 [codex/uart6-vision-oled](https://github.com/2109615535-wq/gongxun/tree/codex/uart6-vision-oled)。安全区抓取过滤的行为和验证记录见[问题003](../process/第02阶段_多个普通物块夹取与放回物资安全区/02_遇到的问题/问题003_2026-11-21_搜索误抓安全区物块.md)。

## 6. weitiao2 模型与板端运行

- 电脑端的 [PT 权重](models/best.pt) 由 [dual_camera.json](config/dual_camera.json) 加载。
- 香橙派的 [RK3588 RKNN 权重和元数据](models/best_rknn_model/) 由 [dual_orangepi.json](config/dual_orangepi.json) 加载。板端使用 Toolkit-Lite2 和 Runtime 2.3.2，仓库模型按固定 640×640、batch 1、非 INT8 导出。
- 两份配置记录了对应模型的 SHA-256，并按名称过滤夹爪和关闭的夹爪。下视 ROI 继续核验其中的物体，机构开合状态来自主控。

训练集按视频各自划分，合计 1,982 张训练图、559 张验证图和 279 张测试图。新模型在置信度阈值 0.35、同类框 IoU ≥ 0.50 的条件下，正确检出 063 号同视频留出片段的 14 个蓝色物资区和 14 个蓝色伤员区。该结果用于记录录像测试。

仓库中的 RKNN 从同一份 weitiao2 ONNX 使用 RKNN-Toolkit2 2.3.2 在 Ubuntu 重新导出。部署时按设备配置核对相机、标定、运行库和模型校验值。

## 7. 坐标、测距补偿和视觉核验

安全区使用完整检测框的中心，物块使用地面接触点。按现有标定计算 X、Y、径向距离和方位角。截边安全区保留检测框。径向补偿先反解真实距离，再等比例缩放 X、Y。方位角保持原方向。已拟合的真实距离范围为 15～150 cm，实际控制沿用标定区域、位置质量和新鲜度检查。[测距拟合结果](config/dual_calibration/distance_compensation_fit.json)

目标完整进入同类安全区、同一目标与区域连续三帧成立时，视觉报告 `object_inside`。主控报告松爪并退开后，可在 `delivery_check` 阶段核对锁定物体留在区内、已验证的下视 ROI 为空以及两路观测的新鲜度。执行条件和状态字段见 [TASK_PROTOCOL.md](TASK_PROTOCOL.md)。

RV2 发送距离和方位角。RV3 另带 X/Y、区域类别及入区有效位。RV4 追加安全区抓取过滤位。完整布局见 [SIMPLE_PROTOCOL.md](stm32/SIMPLE_PROTOCOL.md)。

## 8. 数据集录制与离线检查

先在 Windows 的 `perception/` 目录启动接收端，参数按实际设备和数据集目录填写。

```powershell
tools\run_dataset_receiver.cmd --ssh-target orangepi@10.163.0.250 --dataset-root "F:\Learning coding\deep learning\make_dataset"
```

在网页点击录制，结束时再点击一次。上视原始图像保存为录像，经 SSH 接收到 `videos/`，按现有数字顺延命名。每秒随机抽取 3 帧，按 `视频编号_照片序号.jpg` 保存到 `images2/`。接收端可用 `--identity` 指定本机 SSH 密钥。

在 `perception/` 运行视觉端离线回归。

```powershell
python -m unittest discover -s tests -v
```
