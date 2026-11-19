# 视觉端串口接入（含 angle）

## 径向测距误差补偿（2026-11-16）

当前双摄入口 `dashboard` 在地面投影后、生成观测和串口数据前补偿径向距离。按本次确认，上下视共用模型；电脑和香橙派配置均已启用 `distance_compensation`。X、Y 等比例缩放，方位角保持不变，RV1/RV2 和 JSON 观测中的 `distance_mm` 都取补偿后的距离。主控接收协议不需要改动。

源文件 `vision-error-samples-2026-11-16T08-20-45-186Z.json` 标记为 `upper_raw_serial`。重新计算了 15 个距离点、1821 条原始测距的均值，每个真实距离等权；180 cm 两组无样本，未参与拟合。按完整距离点留一验证选择一次或二次模型，二次更优：一次 R²=0.959203、拟合 RMSE=2.210668 cm、留一 RMSE=2.704644 cm；二次 R²=0.995330、拟合 RMSE=0.747972 cm、留一 RMSE=0.988183 cm。

令 `r` 为真实径向距离、`m` 为原始测距，全部以 **cm** 计算，新的误差函数为：

```text
e(r) = 0.001316478178168962 r² + 0.03766521943003619 r - 2.0773531735933064
m = r + e(r)
r = 2(m + 2.0773531735933064) /
    [1.03766521943003619 + sqrt(1.03766521943003619²
       + 4 × 0.001316478178168962 × (m + 2.0773531735933064))]
```

不能直接用 `m - e(m)`，因为拟合自变量是真实距离。程序将原始 mm 转为 cm，反解后再转回 mm。标定真实距离范围为 15～150 cm；配置中的原始测距适用范围为约 13.7838～186 cm，包含模型端点和本次样本的端点波动，范围外保留原值。原有投影区域、各相机输出范围和有效性条件仍生效；本次拟合不会把原有几何标定扩展到 150 cm。

每条有位置的检测增加 `distance_compensation` 诊断，包含 `enabled`、`applied`、`raw_distance_mm` 和 `corrected_distance_mm`。网页、记录和发送数据使用同一份补偿后位置。`enabled: false` 可关闭补偿。

与香橙派现有版本合并后，保留绿色物块的 `display_position_mm` 估算发送路径；该路径也在生成观测前补偿一次，并继续保持 `distance_valid=false`。因此超出原有效投影区域、但原本能够显示和发送的绿色估算距离也使用新模型，不会被提升为已验证位置。

本批原始样本的 `valid` 全部为 0，因此静态拟合使用数值有效的原始测距，不据此提高 `distance_valid` 或其他运动有效位。逐条回放后，各距离点均值的 RMSE 从 16.1255 cm 降至 0.5830 cm，全部样本 RMSE 为 1.2705 cm；这是原数据回放结果，尚需补偿后的独立实车复测。150 cm 仅有 12 条样本。

完整比较、逐点结果和源文件 SHA-256 见 [distance_compensation_fit.json](config/dual_calibration/distance_compensation_fit.json)。重新拟合并更新两份配置，在 `perception/` 执行：

```powershell
python -m rescue_vision.fit_distance "E:\vision-error-samples-2026-11-16T08-20-45-186Z.json" --output config/dual_calibration/distance_compensation_fit.json --config config/dual_camera.json --config config/dual_orangepi.json
```

部署更新后的模块和配置后，重启原视觉服务即可使用；无需向主控再次补偿。

## 安全区 ROI 中心坐标及入仓判定（2026-11-16）

双摄入口 `dashboard` 已支持红蓝双方的物资安全区、伤员安全区坐标：`red_material_zone`、`blue_material_zone`、`red_casualty_zone`、`blue_casualty_zone`。四类均取检测框 ROI 中心 `((x1+x2)/2, (y1+y2)/2)`，先去畸变，再沿用现有地面投影和径向测距补偿；物块仍沿用原来的贴地点取点方式。

适用前提是主控在投放时使小车正对安全区并直行接近，此时检测 ROI 与实际安全区基本贴合，中心可作为对准和接近的目标点。安全区是地面区域，即使与下视夹爪 ROI 重叠，也按地面中心计算位置。图像边缘截断的安全区框不计算中心坐标；现有标定区域、相机输出范围和观测有效期仍生效。

检测结果复用 `position_mm`，包含 `x_right`、`y_forward`、`ground_range`（均为 mm）和 `bearing_deg`（度），`position_quality=zone_roi_center`，近处外推仍标记为 `near_estimate`。新版 `vision_simple_v3` 在物块和安全区的公共字段中直接传 `x_mm/y_mm`，并保留距离、方位角和有效位；任务 JSON 同步增加这些字段。旧 RV2 保持原布局，可继续由距离和角度还原 X、Y。

按本阶段确认口径，物体整个检测 ROI 位于对应物资/伤员安全区 ROI 内，且同一物体、同一安全区连续 3 帧满足时，输出 `zone_entry.result_valid=true, object_inside=true`。结果包含实际 `zone_kind`，供主控核对本方颜色；普通/核心物资对应物资区，伤员对应伤员区。越界、遮挡、区域歧义、丢失或超过 150 ms 时不保留已入仓结果。该判断使用图像 ROI，独立于测距标定；它不表示已释放、退出后留存或完成整次投放。

RV3 每个目标依次为 `kind,distance_mm,distance_valid,in_roi,angle,x_mm,y_mm,zone_kind,inside_valid,object_inside`，完整格式见 [SIMPLE_PROTOCOL.md](stm32/SIMPLE_PROTOCOL.md)。当前按用户要求先部署香橙派、稍后烧录主控：保持实际服务的 RV2 启动参数，网页/API 提供完整坐标和入仓结果；主控更新接收器后再启用 `vision_simple_v3`。本次视觉提交不包含 C 接收器修改，具体改动见[主控 RV3 接入清单](MCU_RV3_HANDOFF.md)。

未完成独立验证的标定仍可返回范围内的数值，但 `distance_valid=0`；补偿不改变这一标志。中心坐标不包含库口朝向或投放成功判断。取点、使用前提和实车验收要求见[第01阶段目标](../process/第01阶段_50cm内绿色物块夹取与放回物资安全区/01_该阶段目标/阶段目标.md)。

离线检查在 `perception/` 执行 `python -m unittest discover -s tests -v`，覆盖四类安全区中心、三帧入仓、RV3/JSON 回传和过期处理。主控 C 代码、对应测试和构建工具另行保留在本机工作区，未纳入此次视觉提交。部署状态见[本次部署记录](../docs/vision_zone_entry_deployment_2026-11-16.md)。

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
