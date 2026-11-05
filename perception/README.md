# 视觉端串口接入（含 angle）

## 1. 主控端使用哪些文件

- [stm32/vision_simple.h](stm32/vision_simple.h)：类别、观测结构、`angle/angle_valid`及C接收接口。
- [stm32/vision_simple.c](stm32/vision_simple.c)：CRC校验、逐行解析、双路独立的新帧检查和超时处理。
- [stm32/SIMPLE_PROTOCOL.md](stm32/SIMPLE_PROTOCOL.md)：RV2完整字段、校验样例和接入说明。

将两个C文件一起加入主控工程并重新编译。初始化调用`VSP_Init`，主循环从现有UART队列取出字节及到达时间后调用`VSP_Feed`，周期调用`VSP_Get`分别读取上下视。中断/DMA只入队，模块接口在同一任务上下文调用；无新数据时仍需周期读取，以处理超时。

`angle`单位为度，前方0、右正左负。主控必须检查`angle_valid`，不能把未知角度当成0°。旧RV1仍可接收，但没有angle；本版无需旧二进制C接收器。

## 2. 视觉端串口连接配置

采用3.3V TTL，视觉TX接主控所选UART的RX，双方GND共地。当前只发送观测；预留双向时再交叉连接另一对TX/RX。

香橙派在`perception/`根目录执行，设备名替换为实际串口：

```bash
sh tools/run_dual_orangepi.sh --controller-port /dev/ttyUSB0 --controller-protocol vision_simple_v2
```

电脑端示例，COM3和相机编号按实际设备替换：

```powershell
python -m rescue_vision.dashboard --cameras --upper-source 1 --lower-source 2 --controller-port COM3 --controller-protocol vision_simple_v2
```

`vision_simple_v2`发送带angle的RV2；`vision_simple_v1`保留旧RV1四字段布局。两路分别约每100ms尝试发送，空场、过期或故障也发送状态。默认`motor_debug_ascii`只接收原遥测，需在主控接收器集成后显式启用新协议。

## 3. 主控端UART初始化参数

统一采用115200波特率、8位数据、1位停止位、无校验、无硬件流控。仓库已知候选为USART3（PC10 TX、PC11 RX）或USART6（PC6 TX、PC7 RX），实际引脚占用须确认。UART5已有电机调试，USART2已有IMU业务。

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
   - `source`：上视或下视。
   - `frame_id`：实际图像帧编号。
   - `age_ms`：观测年龄。
   - `status`：正常、空场、过期、故障等状态。
   - `target_id`：目标关联编号，**需要新增跟踪/关联能力**。
   - `kind`：物体类别。
   - `distance_mm`、`distance_valid`：距离及其有效性。
   - `angle`：方位角，单位度，右正左负，无角度时为 `NA`。
   - `in_roi`：是否位于下视机构核验区域。

   当前接收器中的 `angle_valid` 可根据角度是否存在、距离有效性和年龄计算，不必重复在线传输。

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
