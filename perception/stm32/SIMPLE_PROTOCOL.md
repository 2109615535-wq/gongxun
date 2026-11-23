# 简易视觉双端协议 vision_simple_v1/v2/v3/v4

## RV4 增加安全区抓取过滤（2026-11-21）

```text
RV4,source,frame_id,age_ms,status,count[,kind,distance_mm,distance_valid,in_roi,angle,x_mm,y_mm,zone_kind,inside_valid,object_inside,in_safety_zone ...]*CRC16\n
```

每个目标 11 个字段。`in_safety_zone` 随当前帧计算。同一画面中，任一安全区 ROI 与物块 ROI 的重叠面积达到物块 ROI 面积的 80% 时为 `1`，其余为 `0`。各安全区独立计算。安全区目标该位为 `0`。STM32 的 `VSP_Object` 和 `VisionObject` 通过 `in_safety_zone` 提供该位，抓取逻辑在新鲜观测中检查它。`object_inside` 继续按连续三帧和任务所需的区域类别核验入仓。

RV4 每行含 CRC、不含换行最多 1024 字节。当前 STM32 接收器校验 RV3/RV4 的坐标与入仓字段。安全区中心可从现有的径向距离和方位角获取，主控继续检查 `distance_valid`、`angle_valid` 和帧新鲜度。RV1 至 RV3 的目标布局保持原版本。香橙派使用旧版串口协议时，区内普通、核心和伤员物块发送距离 `-1`、方位 `NA`、`distance_valid=0`、`in_roi=0`，RV3 中 X/Y 也为 `NA`，类别及入仓字段仍可读取。区内物块占据下视夹爪 ROI 时，兼容模式同时屏蔽该帧其他可抓取物块的夹爪 ROI 命中。

主控固件支持 RV4 后，视觉服务使用 `--controller-protocol vision_simple_v4` 即可发送新增位。

## RV3：完整位置与视觉入仓（2026-11-16）

RV3 在 RV2 的五个目标字段后追加 X、Y 和入仓字段。直接使用 X/Y 或入仓信息的业务接入点见[主控 RV3 接入清单](../MCU_RV3_HANDOFF.md)。

```text
RV3,source,frame_id,age_ms,status,count[,kind,distance_mm,distance_valid,in_roi,angle,x_mm,y_mm,zone_kind,inside_valid,object_inside ...]*CRC16\n
```

每目标十个字段。X 向右、Y 向前，单位 mm，有符号整数；未知的 X、Y 必须同时为 `NA`。`angle` 为地面方位角，单位度，正前方为 0、右正左负，不是安全区自身朝向。距离仍为径向距离。有效位置必须同时具备 X、Y、距离和角度。四类物块的新鲜估算位置也可为有效。

`zone_kind` 是当前物体核验的实际安全区名称，未知为 `NA`。`inside_valid` 是视觉 ROI 入仓判断有效位，`object_inside` 只有在完整物体 ROI 处于对应安全区 ROI 内且同一物体/区域连续 3 帧满足时为 1。物资对应物资区，伤员对应伤员区；主控仍需核对区域颜色。该结果独立于测距有效位，不表示夹爪已释放或投放完成。累计年龄达到 150 ms 时，视觉端撤销入仓有效位和已入仓标志。读取这两个标志的主控还需计入本地经过的时间。

RV3 每行含 CRC、不含换行最多 1024 字节，最多 12 个目标。RV1/RV2 每行最多 512 字节。发送端将下视等待上视发送的时间计入年龄。

以下保留 RV1/RV2 的具体接入说明。

本文件位于`stm32/`，下文命令均在`perception/`根目录执行。

双向任务/状态/核验使用新增 `rescue_json_v2`，详见 [TASK_PROTOCOL.md](../TASK_PROTOCOL.md)。本目录 C 文件提供 RV1 至 RV4 的主控参考接收器。

2026-11-04 在简易协议中增加地面方位参数`angle`，新版标识为`RV2`，串口选项为`vision_simple_v2`。视觉发送识别结果，主控接收物体列表；不包含任务下发、状态机回传或动作指令。实际主控工程、板端运行服务和GitHub工作副本均未在本次修改。

旧`vision_simple_v1`仍发送原来的`RV1`四字段格式，不发送angle。新版C接收器接受RV1和RV2；主控须同时更新`vision_simple.c/.h`并重新编译，旧接收器不能解析RV2。原二进制`vision_v1`和JSON `rescue_json_v1`也是独立选项，不能交叉使用接收器。Python `encode()`默认生成RV2，旧端显式调用`encode(observation, version=1)`。

## 八类线上名称

- `green`：普通物块，模型内部名 `normal_material`。
- `orange`：伤员，模型内部名 `casualty`。
- `blue`：危险物块，模型内部名 `hazard`。
- `black`：核心物块，模型内部名 `core_material`。
- `red_casualty_zone`：红色伤员区。
- `red_material_zone`：红色物资区。
- `blue_casualty_zone`：蓝色伤员区。
- `blue_material_zone`：蓝色物资区。

区域名采用用户确认的上述规范名称。模型权重及内部类别不重命名，只在发送时映射。夹爪本体 `gripper/gripper_closed` 不作为物资发送。同类多个物体保留多个条目，不按类别合并。

## 一行一帧

115200/8N1、3.3V TTL、共地。格式为ASCII，逗号分隔，以LF (`\n`) 结尾，接收端也支持CRLF。

```text
RV2,source,frame_id,age_ms,status,count[,kind,distance_mm,distance_valid,in_roi,angle ...]*CRC16\n
```

固定头6项，后面每个物体5项，顺序固定；旧RV1仍为每物体4项：

- `source`：`upper` 上视、`lower` 下视。两路分别发帧，不把同一物块在两路的检测自动合并。
- `frame_id`：该相机实际检测帧编号，u32回绕。重复发送同一图像时编号不变；两路编号分别维护。
- `age_ms`：发送快照时该检测结果的软件年龄，0～65535，未知为-1。有效识别必须小于300ms；曝光、相机内部缓存和串口传输时间不在此数字中。
- `status`：`ok` 有目标，`empty` 本帧无上述八类目标，`stale` 观测过期，`fault` 相机/推理/未知类或未分类暗区等不可用，`overflow` 超过12个目标。只有 `ok` 携带1～12个目标，其他状态count=0；`empty` 仍属于正常的新帧。
- `count`：本行物体数，0～12。没有目标或故障也持续发帧，不能只停发。
- `kind`：上述八个英文名称之一。
- `distance_mm`：目标参考点相对当前标定原点的地面径向距离，单位mm，0～65535；无距离为-1。物块使用可见贴地边缘，四类安全区使用检测 ROI 中心经地面投影后的位置。它不是车头间隙或前向Y。沿用已有 `ground_range=hypot(x_right,y_forward)`，并在发送前应用已配置的径向测距补偿。
- `distance_valid`：发送时年龄小于 150 ms、位置和方位可用时，四类物块及四类安全区均置 1，包括估算位置。主控接收模块按本地经过的时间继续收紧 150 ms 限制。
- `in_roi`：下视物体是否满足现有夹爪区域覆盖阈值。1只表示位于区域内，不表示已夹紧、类别相符、连续帧确认或区域已验证。上视恒为0。ROI未配置时下视全部为0。
- `angle`：地面方位角，单位**度**，范围-180～180，正前方0、右侧正、左侧负，发送保留两位小数，接收最多两位小数。没有地面方位时为`NA`，不是0或-1；0°和-1°本身都是合法方向。它来自已有的`atan2(x_right, y_forward)`，不是图像框中心像素偏移。下视ROI中的离地物体、没有地面位置的区域等发送`NA`。

有估算角度但`distance_valid=0`时，角度也不能用于控制。C端增加`float angle`及`angle_valid`：有数值且位置/距离有效时才置位，累计年龄达到150ms后与`distance_valid`一起清零。旧RV1或`NA`结果的`angle_valid=0`；不能仅因`angle==0`就认为目标位于正前方。

`CRC16` 为4位十六进制CCITT-FALSE校验，poly=0x1021、init=0xFFFF、不反射、xorout=0，标准 `123456789` 为0x29B1。校验范围从 `RV2`（旧版为`RV1`）的R至星号前最后一个字符，星号与换行不参与。每行不含换行最多512字节。

完整可解析的合成样例（含真实校验值，每行末尾需加LF）：

```text
RV2,upper,42,86,ok,1,green,450,1,0,30.00*CBDC
RV2,lower,108,92,ok,2,green,-1,0,1,NA,black,-1,0,1,NA*A7DB
RV2,upper,43,83,empty,0*0284
RV2,lower,0,-1,fault,0*86E0
```

第一行表示上视一个普通物块、地面径向450mm、方位右偏30°、位置有效。第二行表示下视夹爪区同时检测到普通与核心物块、两者距离及地面方位均未知，主控不能按单物体相符处理。这些是编解码样例，不是当前实测标定结论。

## 香橙派发送端

发送代码为 `rescue_vision/simple_protocol.py`，复用双摄现有检测和地面映射，发送全部八类检测，不依赖网页选择了什么任务。主控根据类别和当前状态机自行选择目标。

串口部分从 `rescue_vision/dashboard.py` 的 `ControllerLink.run()` 开始阅读，它负责连接和出错后重连。`send_simple()` 的循环依次执行取得双摄观测、逐路编码发送、等待0.1秒三个步骤。`self.stop` 是停止开关，`port.write(data)` 发送字节，`self.stop.wait(.1)` 在等待期间也能响应停止操作。

明确接好一个已接入本版C接收器的视觉串口后，替换实际设备路径启动：

```bash
cd /home/orangepi/rescue_vision_dual
./tools/run_dual_orangepi.sh --controller-port /dev/实际视觉串口 --controller-protocol vision_simple_v2
```

电脑端也可在本工程目录运行：

```powershell
python -m rescue_vision.dashboard --cameras --upper-source 1 --lower-source 2 --controller-port COM3 --controller-protocol vision_simple_v2
```

两路各约每100ms尝试发一行，实际速率受串口、物体数和调度影响，故障帧也按此节奏发送。不接串口时双视网页照常运行；默认 `motor_debug_ascii` 仍只收不发，不会自动向现有电机调试口发送新协议。简单协议不接受任务消息，也不将串口连接视为主控状态机已接入。旧JSON任务/网页联调可按原入口继续使用。

普通物资使用绿色贴地点，其余物块使用完整检测框的底边中部，四类共用地面投影和径向补偿。四类安全区使用检测 ROI 中心，经去畸变、地面投影和测距补偿后随当帧回传类型、距离和方位，RV3/RV4 还携带 X/Y。

主控在投放时使用安全区中心定位。超出常规投影范围时上报地面单应外推结果，截边时上报可见框中心的地面估算。识别到安全区且能够解算位置和方位的当前新鲜帧填 `distance_valid=1`，包含外推、截边和未验证标定的结果。无法投影时继续上报类别，距离为 `-1`，角度及 X/Y 为 `NA`，有效位为 0。库口姿态和投放结果使用独立字段。

## STM32接收端

将 `stm32/vision_simple.c/.h` 加入真实主控工程，不需要旧版 `rescue_vision.c`、HAL或数学库，无动态内存。`VSP_Receiver` 放静态区。该模块只解析观测，不计算速度或操作电机。

```c
#include "vision_simple.h"
static VSP_Receiver vision;
static VSP_Observation upper, lower;

/* 初始化一次，开启接收前调用。 */
/* VSP_Init(&vision); */

/* 主循环中，从现有UART接收队列逐字节取出： */
/* VSP_Feed(&vision, received_byte, byte_arrival_ms); */

/* 周期任务中，即使没有新串口数据，也持续调用： */
/* uint8_t upper_available = VSP_Get(&vision, VSP_UPPER, HAL_GetTick(), &upper); */
/* uint8_t lower_available = VSP_Get(&vision, VSP_LOWER, HAL_GetTick(), &lower); */
```

这是三个调用位置示例，不是替换 `main.c` 或HAL回调的补丁。模块全部函数只能在同一主循环/任务上下文调用；中断或DMA用原驱动队列记录字节及本地到达时间，再交给本模块。主循环排队延迟不能伪装成新的字节到达时间。UART出错时应撤销旧观测，可在相同任务上下文重新 `VSP_Init`；不能与解析并发复位。

`VSP_Get` 返回1表示该相机有未过期的正常观测，`VSP_EMPTY` 时物体数为0；返回0表示过期、故障、溢出或没有数据，输出物体数清零。两路独立维护，禁止使用上视心跳保活下视结果。

接收器用主控本地时间判断有效期并累计发送侧年龄。本工程实际主控现有期限为累计大于400ms失效、恰好400ms仍保留正常观测；本目录旧参考 C 实现为300ms，接入时应核对实际使用版本。距离/角度有效位，以及适配 RV3 后的入仓有效位，在150ms时清除。重复或短时倒序编号不刷新期限，相同编号即使过期也不会保活；超时后不同编号可重新建立序列，支持发送端重启。u32编号和本地时钟回绕均支持。故障帧立即清除本路可用性，但不修改正常图像序列。CRC或格式错误不更新旧观测，旧观测按期限失效；超长行丢弃至换行后恢复。串口线路与排队时延需要实测并在主控使用期限中额外留量。

对象类型在C内按 `VSP_GREEN` 等枚举读取，线上仍传完整英文字符串。主控只遍历 `observation.count` 个条目，不能读取旧数组尾部或把 `in_roi` 当作夹持成功。危险 `blue` 即使距离无效也必须纳入避让/阻断决策；未知暗区和多物体也不能直接转成单物体抓取成功。

## 建议保留和后续补充

本版已保留相机来源、新帧编号、年龄/故障状态、距离有效性、夹爪区域归属与CRC，分别解决双摄混淆、旧结果重复保活、断流、无效测距、夹爪外目标和串口错误问题。置信度、检测框、姿态、模型元数据和状态机任务不随此版发送，网页仍可显示原诊断数据。

本版已补充`angle`。主控仅在`distance_valid`和`angle_valid`均有效时，才可还原方向和位置：

```text
angle_rad = angle * pi / 180
x_right_mm = distance_mm * sin(angle_rad)
y_forward_mm = distance_mm * cos(angle_rad)
gap_mm = y_forward_mm - bumper_forward_mm
```

安装原点、车头与机构偏移须双方按实车约定。坐标还原不等于已实现底盘速度换算、接近停车或完整转运状态机，失效时撤销运动命令仍由主控集成代码负责。

## 最少交付与安装文件

主控接收只需`stm32/vision_simple.c`、`stm32/vision_simple.h`和本文；再明确实际UART引脚、115200/8N1、3.3V TTL、共地及接收任务调用位置即可。主控仍须接入现有UART字节/到达时刻队列、周期调用`VSP_Get`及失效停车，不需要把Python程序或`pyproject.toml`编译进STM32。

`pyproject.toml`用于Python包安装元数据、依赖声明、命令入口和网页资源打包。直接从源码目录运行`python -m rescue_vision.dashboard`且所需依赖已安装时，它不是启动必需文件；使用`pip install -e .`等项目安装流程时需保留对应项目配置。视觉设备仍需实际模型、标定和运行依赖，不能只复制几个Python文件就假设环境完整。

仍需确认实际视觉串口、引脚和电平，集成C模块并完成串口实机测试、当前ROI重验、各类别定位验证和主控状态机闭环。本版未宣布车辆可自主救援，也未更新GitHub。2026-11-04新增angle后，主机全套242项测试通过，其中56项简易协议测试实际编译运行C接收器；主机软件测试不代表实机停车验收。
