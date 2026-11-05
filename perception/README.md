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

## 5. 上电发送与OLED联调

2026-11-05已在Orange Pi 5 Ultra板端配置并重启核验，当前IP为 `10.50.6.250`。部署目录为 `/home/orangepi/rescue_vision_dual`，使用 `/home/orangepi/venvs/yolo-rknn-board/bin/python`。上下视相机正常，串口协议为 `vision_simple_v2`。

### 接线与串口

- 香橙派40针排针物理13脚为 `UART6_M1_TX / GPIO1_A1`，接STM32的 `PC7 / USART6_RX`。
- 香橙派物理6脚为GND，与STM32的GND相连，信号使用3.3V TTL。
- 板端 `/boot/orangepiEnv.txt` 已启用 `overlays=uart6-m1`，对应设备为 `/dev/ttyS6`。
- 双方使用115200、8N1、无流控。上述物理脚号依据本地5 Ultra用户手册第200页和原理图第27页核对。

香橙派启动脚本默认开启双摄和RV2串口发送。需要更换实际串口时设置 `VISION_CONTROLLER_PORT`，也可显式传入 `--controller-port`。

### 开机自启

服务源文件位于 [tools/rescue-vision.service](tools/rescue-vision.service)，板端已安装到 `/etc/systemd/system/rescue-vision.service` 并启用。重启后服务自动运行，已观察到UART6发送字节计数持续增加。

在香橙派终端检查服务和日志：

```bash
systemctl is-enabled rescue-vision.service
systemctl is-active rescue-vision.service
journalctl -u rescue-vision.service -n 50 --no-pager
```

电脑可访问 `http://10.50.6.250:8080` 查看双摄画面和通信状态。

### STM32接入与验收顺序

1. 接入本目录 `stm32/vision_simple.c/.h`，在初始化后调用 `VSP_Init` 并启动USART6接收。
2. 在现有UART接收和错误回调中增加USART6分发，中断只将字节及到达时间放入队列，主循环或调度任务调用 `VSP_Feed`。
3. 周期调用 `VSP_Get` 分别读取 `VSP_UPPER` 和 `VSP_LOWER`。OLED只显示green，上下视各四行，依次为来源/green编号、X和Y、angle（A）、distance（D）。多个green每两秒轮播，没有green时显示NO GREEN。
4. X向右为正，Y向前为正，X、Y、distance单位为mm，angle单位为度。X和Y由距离与角度计算，有数值但有效标志为0时加 `~` 前缀。缺失数值显示NA，故障或观测年龄达到300ms时清除旧目标。
5. OLED页面由 `vision_uart6_oled_task` 统一绘制和刷新，上下视各占半屏。
6. 两路分别放入已知类别物体，核对来源、类别和角度正负。中断视觉发送后检查OLED能进入超时状态，再进行整车断电上电测试。

主控工程已接入UART6接收和green专用OLED页面，上视占前四行、下视占后四行。接收解析直接编译本目录的vision_simple.c。联调固件上电禁用电机，目标速度为0。

green专用固件已重新烧录并通过Flash校验，实机已确认仅显示green，X、Y、A、D四项数据正常出现。

网页上下视坐标区也可显示green的X/Y估计，未选定目标时自动显示可估算的green，估计值带 `≈` 标记。每个green识别标签附有对应坐标，故障或观测过期时清除坐标。网页坐标回归检查在perception目录运行 `node --test tests/test_web_coordinates.js`。

green显示估计沿用旧标定的单应矩阵，尝试覆盖图像中可投影的前方地面，扩大到原100～800mm、0～300mm显示区域之外。优先使用绿色分割的可见底边，必要时使用检测框底边中心。范围外推只提供显示估计，原位置有效性和控制判断继续使用原标定条件。机构ROI内的物体、投影奇点、车后点或超出RV2数值上限的结果显示NA，外推精度需要实测确认。

在仓库根目录使用已安装的ARM GCC编译并通过DAPLink烧录：

```powershell
python project/tools/build_vision.py
powershell -ExecutionPolicy Bypass -File project/tools/flash_vision.ps1
```

本机脚本自动查找STM32CubeIDE自带的ARM GCC与OpenOCD。固件输出为 `project/build/project.hex`，该目录只保存本地构建产物。Keil工程也已添加接收源文件和包含路径，有ARM工具链的环境可使用原工程编译。

2026-11-05已通过DAPLink完成STM32F407烧录，Flash校验返回Verified OK，复位后正常接收。主控UART6接收字节计数持续增加，采样期间UART错误、队列溢出、协议错误均为0，OLED上下视内容和类别更新已由实机确认。整车完全断电后重新上电，程序自动恢复并更新类别，冷启动验收通过。当前使用局部检查和拟合的旧标定估计，有效标志仍按验证状态和150ms窗口计算，OLED以 `~` 标记未验证或已超出150ms窗口的数值。无法提供估计的目标显示NA。

`perception` 和 `project` 的改动提交到GitHub时，先创建独立分支，再由用户确认提交内容。
