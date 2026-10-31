# 主控端视觉接入 README（观测协议 v1）

本文供STM32主控负责人实现视觉接收和接近控制。2026-10-31核对基线：仓库`main`的`6926a14`，其中主控功能基线为`d915341`。视觉包v0.3.2只支持此版四电机新格式；本次不修改主控源码或v1线上字节布局。

## 1. 先回答：上传内容是否足够主控开发

**足够实现“视觉到主控”的v1观测接收、字段校验、坐标还原、失效处理，以及单个普通物资的对准和接近停车。**已有Python编码器、C接收器、合成报文和跨语言测试，不需要主控重新运行YOLO，也不需要等待外参测完才开始写接收代码。

**尚不足以当作完整的双向救援任务协议或已经完成的整车固件。**当前缺少主控到视觉的任务命令、应答、目标锁定ID、可靠的全场定位/安全区导航和完整转运任务状态机。实际串口接线、底盘速度适配、实车标定及验收仍需完成。

本次上传包含的PT/ONNX模型用于视觉处理，主控不需要加载它们。原香橙派已部署RKNN不在仓库内，需要视觉负责人在板端保留配套环境。

先按下面路径阅读：

- [PROTOCOL.md](PROTOCOL.md)：v1线上协议与失效语义。
- [rescue_vision/protocol.py](rescue_vision/protocol.py)：Python编码/解码，线上字节格式的实际实现。
- [rescue_vision/types.py](rescue_vision/types.py)：状态、类别和标志位。
- [stm32/rescue_vision.h](stm32/rescue_vision.h)、[stm32/rescue_vision.c](stm32/rescue_vision.c)：可复用C接收器及接近参考控制。
- [tests/test_protocol_stm32.py](tests/test_protocol_stm32.py)：Python/C互通与异常处理验收样例。
- [START_HERE.md](START_HERE.md)：视觉运行环境、模型及标定文件说明。

## 2. 职责与实际发送入口

视觉负责图像、YOLO类别映射、目标贴地点提取、位置解算、有效性判断和观测打包。主控负责解析、失效检查、目标使用策略、底盘运动学、速度闭环及机构动作。硬件负责端口引出、电平、接线、供电和相机安装尺寸。

这里有两个不同入口：

- `tools/detect_green_distance.py`：电脑端离车测距，输出画面和文件，**不发送串口**；画面清空坐标不代表主控已收到无目标帧。
- `python -m rescue_vision run ... --serial <设备路径>`：由`runtime.py`打包并发送v1二进制观测；没有`--serial`时只运行和记录。`session.py`只生成观测，也不自行打开串口。

主控实际支持v1、且硬件端口已确认后，视觉负责人才能在自己的配置副本中将`serial_protocol`设为`vision_v1`。当前配置默认是`motor_debug_ascii`，运行器会在打开串口前拒绝向该接口发送二进制帧；旧配置名称已不再接受。

v1发送方向为视觉TX到主控RX，共地。无需主控请求、ACK或重传命令。实时观测使用新帧，不重发旧图像维持运动；反向任务消息尚未实现。

## 3. 当前仓库主控接入点

以下依据当前`project/`源码，不按旧交接记录推断：

- UART5：PC12 TX、PD2 RX，115200/8N1；`faraway.c`处理ASCII命令和四电机JSON遥测，不是视觉二进制接收器。`TARGET`当前更新四个电机目标，`ENABLE`路径仍针对电机1；不能假设所有命令的作用范围一致。
- USART2：PA2/PA3，用于HDZ601 IMU，保持原业务。
- USART3：PC10/PC11；USART6：PC6/PC7。两者均初始化为115200，已有HAL IRQ转发，但尚无视觉业务接收。可作为候选，实际引脚可用性和视觉板端口须由硬件确认。
- `project/Core/Src/hdz601.c`已经定义`HAL_UART_RxCpltCallback`和`HAL_UART_ErrorCallback`，当前分发UART5和USART2。接视觉时应在现有回调中新增分支，不再定义第二个同名全局回调。
- `main.c`使用合作式调度，`motor_process`和`position_encoder`当前为20ms。视觉任务建议10ms，需复用`scheduler_add_task`，并检查注册返回值、任务耗时及优先级。
- 当前开机使能四电机并把目标设为50，`position_encoder`在`y_body>=50`时置四电机目标为0。开展接收验收前，电控须建立明确的运动使能和命令仲裁，避免开机目标、50cm试验逻辑、ASCII调试和视觉任务相互覆盖。
- `faraway.c`目前上报`speed1/target1`到`speed4/target4`及`y_body`。视觉包v0.3.2的`stm32-status`只支持此格式，旧单电机数据作为无效行丢弃。默认只接收，`--query`只发一次`GET`，不发送电机控制命令。

从`perception/`运行`python -m rescue_vision stm32-status --port COM3 --duration 5 --output recordings/stm32_status.jsonl`可只读记录，端口按实际设备修改。记录保持主控原始字段，标注`telemetry_format=four_motor`、速度归一化单位及`y_body_unit=cm`；后者是主控里程计，不是视觉目标距离。汇总分别报告有效样本、无效行、超长行和应答数量，错误应答还报告`error_replies/last_reply`。不从这些字段推断电机已停稳、已使能或视觉控制已具备。

历史`stm32/README.md`、`hardware/*.json`和`HANDOFF_V*.md`中“只控制电机2”等描述只代表当时版本；接当前主控优先核对上述源码。

## 4. UART与字节约定

- 3.3V TTL，TX/RX交叉、共地；不是RS232或RS485电平。
- 联调115200bps，8数据位、无校验、1停止位，无硬件流控。v1也允许921600，但双方必须同步配置。
- 所有多字节整数采用小端；有符号整数为二进制补码。
- 二进制包不是ASCII文本，也不是JSON；不能调用按行解析器、`strlen`或用换行符分包。
- 串口可能拆包、粘包、插入噪声；按帧头、长度和CRC恢复边界。

一个目标的总帧长28字节；空观测16字节；最多192字节。115200/8N1发送192字节约16.7ms。当前有效观测频率并未作为固定保证：此前香橙派实测约12.7～12.9Hz，项目目标≥15Hz；故障或无新结果时，运行器约每50ms尝试发无效心跳，实际受线程调度及串口影响。

## 5. 精确字节布局

所有偏移从0开始，下面前两组偏移相对于整帧。

```text
AA 55 | version:u8 | sequence:u8 | payload_length:u16 | payload | CRC16:u16
```

外层头：

- 偏移0～1：固定`AA 55`。
- 偏移2：`version`，当前必须为1。
- 偏移3：`sequence`，0～255，每次发送加1，255后回0；用于诊断，不用它代替图像新鲜性。
- 偏移4～5：`payload_length`，只计payload，不含外层头和CRC。

payload固定部分，共8字节：

- 整帧偏移6～9：`tick_ms:u32`。图像观测为视觉读取图像完成时的本地单调毫秒数，故障心跳为发送侧当前时间；允许32位回绕。
- 整帧偏移10：`frame_flags:u8`。
- 整帧偏移11：`target_num:u8`，0～12。
- 整帧偏移12：`zone_num:u8`，当前发送0，接收器允许0～4但不使用场地项控制运动。
- 整帧偏移13：`status:u8`，0～7。

目标数组从整帧偏移14开始，每个目标12字节。以下偏移相对于单个目标起点：

- 偏移0：`type:u8`。
- 偏移1：`confidence:u8`，0～100，不是0～1浮点数。
- 偏移2～3：`angle_cdeg:i16`，单位0.01°，有效编码范围-18000～18000。
- 偏移4～5：`distance_mm:u16`，0表示无效，非0值还必须结合有效标志检查。
- 偏移6～7：`width_px:u16`，原检测分辨率下的框宽。
- 偏移8～9：`height_px:u16`，原检测分辨率下的框高。
- 偏移10：`posture:u8`，当前发送0，表示未知，不据此控制机构。
- 偏移11：`target_flags:u8`。

目标后面是每项8字节的场地预留区；本阶段只按长度跳过，不假设其内部字段。最后2字节是CRC。

```text
payload_length = 8 + 12 * target_num + 8 * zone_num
total_length   = payload_length + 8
```

payload最大184字节，总帧最大192字节。不要把`target_num`误当作payload长度，也不要直接把字节缓冲区强转为`RV_Target*`或发送C结构体的`sizeof`结果。

CRC为CRC16/CCITT-FALSE：poly=0x1021、init=0xFFFF、不反射、xorout=0。计算范围为整帧偏移2开始，到payload最后一个字节；不包含`AA 55`和CRC自身。CRC结果低字节先发。标准串`123456789`应得到`0x29B1`，已有`RV_Crc16`可复用。

## 6. 类别、状态与标志

协议类别：1普通物资、2核心物资、3伤员、4危险目标、255未知。视觉侧按模型类别名称映射，主控不能直接使用YOLO原始class_id推断协议类别。

`status`：

- 0：`OK`，当前普通目标流程已确认；仍须检查所有有效标志和危险条件。
- 1：`NO_TARGET`，无当前可用普通目标；不保证整个画面没有其他类别。
- 2：`CAMERA_FAULT`，相机故障。
- 3：`CALIBRATION_INVALID`，标定无效。
- 4：`TARGET_INVALID`，目标或位置不可信，或连续帧确认尚未完成。
- 5：`TARGET_LOST`，此前确认的目标丢失。
- 6：`STALE`，图像/推理结果过期或没有及时获得新结果。
- 7：`INFERENCE_FAULT`，推理异常。

`frame_flags`按位检查：bit0/0x01观测有效；bit1/0x02标定有效；bit4/0x10接近通道阻断。bit2、bit3为预留，当前发送端不置位，不据此推断安全区或近场能力。

`target_flags`：bit0/0x01危险；bit2/0x04遮挡或截断；bit4/0x10连续帧确认；bit5/0x20位置有效。bit1姿态可信、bit3进入铲入区尚未实现，主控不可依赖。

注意`0x10`在帧flags中表示阻断，在目标flags中表示连续确认，不能混用。置信度高也不能替代标定、位置和连续帧有效性。

## 7. 坐标、安装参数和里程计的关系

线上不直接传实验的`X right / Y forward`，而是传**可见贴地边缘中点相对底盘原点的地面径向距离与方位角**。方位向右为正。

```text
angle_rad = angle_cdeg * pi / 18000
x_forward_mm = distance_mm * cos(angle_rad)
y_left_mm    = -distance_mm * sin(angle_rad)

gap_mm     = x_forward_mm - bumper_forward_mm
lateral_mm = y_left_mm - collection_center_left_mm
```

正前方目标角度为0；右侧目标角度为正、还原后`y_left_mm`为负。宽高只用于诊断，主控不再次用框宽高猜测距离。

双方需要共同固定底盘原点，并实测`bumper_forward_mm`（原点到车头前缘的前向距离）和`collection_center_left_mm`（机构中心相对原点向左的偏移，可为负）。这两个值与视觉标定一致；测试中出现的120mm等数值只是合成夹具参数，不能直接作为实车安装值。

当前`decode_move.h`注释使用右向+x、前向+y、单位cm，`x_body/y_body`对应自初始化起的里程计量；视觉协议则是当前车体相对坐标、单位mm。需要融合时，必须区分实验坐标、当前车体坐标和里程计参考坐标；涉及运动旋转还需实测航向变换。不能只交换轴名就把视觉相对坐标当作里程计绝对位置，也不能把当前50cm停车阈值直接当成视觉停车距离。

内外参及地面映射留在视觉端配置；不需要随每帧传给主控。上传的离车映射和配置仍为未验证实车状态，不能为使车辆运动而改有效位。未标定时主控仍可完成收发和显示，运动输出应为0。

## 8. 推荐直接复用C接收器

将`stm32/rescue_vision.c`加入主控构建目标，并添加头文件搜索路径。模块只依赖标准C、`stdint.h`、`string.h`、`math.h`，没有HAL和电机依赖；需要链接数学函数。含接收环和缓冲区的`RV_Receiver`建议放静态区，约5KB，不放在中断栈上。

真实接口只有以下几个，参数名称以头文件为准：

```c
void RV_Init(RV_Receiver *receiver, RV_Config config);
void RV_OnByte(RV_Receiver *receiver, uint8_t byte, uint32_t received_ms);
RV_Command RV_Step(RV_Receiver *receiver, uint32_t now_ms);
void RV_ResetTrial(RV_Receiver *receiver);
uint16_t RV_Crc16(const uint8_t *data, uint16_t length);
```

集成顺序：

1. 分配静态`RV_Receiver`，在开启视觉接收IRQ之前调用`RV_Init`；填入实测安装偏移。`allow_forward=0`只禁止前进，仍可能返回转向请求，不等于禁止运动。
2. 对确认分配的UART启动接收。在已有HAL接收回调的视觉分支中逐字节调用`RV_OnByte`，时间取STM32的`HAL_GetTick()`，然后重新启动接收并检查HAL返回值。若选DMA/IDLE，先正确处理环形缓冲区和时戳，不重复投递字节。
3. 只允许一个UART字节生产者；ISR只投递，不解析、不计算电机控制、不调用`RV_Step`。HAL错误恢复和主控自身故障锁存由集成代码负责，本模块没有现成UART错误回调接口。
4. 只在一个控制上下文中每10ms调用`RV_Step`。返回结果包含状态、原因、前进速度和角速度；此任务必须在无串口数据时继续运行，以触发超时。
5. 纯接收阶段只记录/显示返回值，所有驱动保持未授权状态。完成符号、单位和停车验证后，才由主控唯一的运动命令出口执行这些值，**包括每次返回的零速度**。
6. `RV_STOPPED`为到位锁存；新试验由任务状态机在同一控制上下文显式调用`RV_ResetTrial`，复位后仍等待新图像。不能在每个周期自动复位。

例如以下三个片段分别属于初始化、已有UART回调分支、控制任务，**不是一份可直接粘贴覆盖main.c的完整补丁**：

```c
/* 静态存储；config由实测安装参数构成。 */
static RV_Receiver vision_rx;
static RV_Command vision_command;
/* 初始化阶段，早于开启该UART接收中断： */
/* RV_Init(&vision_rx, config); */

/* 现有视觉UART接收分支内，byte为刚收到的单字节： */
/* RV_OnByte(&vision_rx, byte, HAL_GetTick()); */

/* 主循环中的单个10ms任务： */
/* vision_command = RV_Step(&vision_rx, HAL_GetTick()); */
```

接收任务调用完`RV_Step`后，可在同一上下文读取`vision_rx.observation`、`crc_errors`、`format_errors`和`duplicates`做调试显示；不要让另一个上下文并发解析或改写接收器。

## 9. 接收条件和失效时序

无数据时输出0。有效运动须满足帧bit0/bit1同时置位、`status=0`、无帧阻断、恰有一个普通物资，且该目标置信度≥35、bit4/bit5同时置位、非危险、非遮挡、距离非0。接收器还会独立阻断未知目标和接近通道中的危险目标；危险位置不可信时阻断。

- 用STM32本地接收时间判断300ms有效期，不能用视觉tick直接减STM32 tick。
- 同一采集tick的重复有效图像不刷新运动有效期，也不增加到位计数；sequence变化不能绕过。
- 源tick在短于300ms的接收间隔内倒退会被拒绝。间隔达到300ms后，不同tick可重新建立源时间，兼容视觉重启；相同tick仍拒绝。实现兼容u32回绕。
- 正常空场、目标丢失、标定无效、危险等在控制任务处理到时撤销运动，不等待300ms；故障心跳不能保活旧目标。
- 接收残帧的相邻字节本地时间间隔超过30ms会清空残帧；这不是整帧必须在30ms以内的另一项长度规则。
- CRC、版本、长度、状态、类别等错误帧不发布为新观测，也不刷新有效期；当前实现不因单个CRC错误立即清除上一有效观测，上一观测仍按300ms限期失效。接收环溢出则在下一次`RV_Step`立即撤销输出。
- “立即”是下一次控制任务执行时，300ms是软件命令失效阈值；20ms电机任务、合作式调度阻塞、制动及机械惯性会增加实际停车时间，需实测。

当前环容量512个槽位，保留一槽用于区分满/空；115200/8N1连续满速输入约44ms可填满511个槽位。长时间阻塞显示、日志或其他任务会影响接收，不能只看任务名义周期。

## 10. 参考控制的能力和速度单位

`RV_Command`中的`forward_mm_s`单位mm/s，前进最大100；`clockwise_rad_s`单位rad/s，顺时针为正，幅值最大0.4。接近方向误差超过5°时先转向。默认车头间隙目标200±30mm，机构横向误差±20mm，三个不同且被控制任务消费的新图像满足后锁存停车；过近不自动倒车。这些数值是参考接近策略，不是通信字段或全部赛题动作。

当前视觉流水线主要确认并定位一个选中的普通物资；协议虽能携带12个目标项，**不代表已实现12目标跟踪、稳定ID和所有类别的转运决策**。核心/伤员检测项不能直接作为已经通过同等连续确认的可执行收集目标。要改变策略，应明确更改主控控制层，而不是偷偷改变v1字段含义。

`MotorSetSpeed1..4`实际写入各PID的target；当前实现用`encoder_get_speed()`归一化反馈，不直接接受mm/s。`motor.h`中PWM注释与实际PID目标用法仍不一致，以实现为准。

底盘适配需先由真实底盘运动学将车体速度换算为四轮线速度，再按实测轮径和有效每轮计数换算：

```text
raw_counts_s = wheel_mm_s * effective_counts_per_wheel_rev / wheel_circumference_mm
normalized_target = raw_counts_s * 1000 / ENCODER_SPEED_AT_PWM1000
```

还需校验各轮方向、轮序、限幅、转弯符号及计数比例。当前代码中共用的543170和里程计公式使用的40000是现有实现参数，不等于已对全部实车电机独立验证。不能把100mm/s直接传给`MotorSetSpeedN(100)`，也不能通过把四个电机统一置相同目标实现转向。

## 11. 可直接用于串口助手的合成样例

仅在电机未使能或输出隔离时使用。串口助手选择HEX发送，不附加文本换行；不要持续发送同一tick期待它一直有效。

正常样例：sequence=0、tick=1000、帧flags=3、status=0，一个普通物资，置信度90，角度0、距离500mm，宽高40px，姿态0、目标flags=48；28字节：

```text
AA 55 01 00 14 00 E8 03 00 00 03 01 00 00 01 5A 00 00 F4 01 28 00 28 00 00 30 03 1C
```

空场样例：sequence=1、tick=1080、标定有效但无有效观测，status=1；16字节：

```text
AA 55 01 01 08 00 38 04 00 00 02 00 00 01 07 71
```

标定无效样例：sequence=2、tick=1160、flags=0、status=3；16字节：

```text
AA 55 01 02 08 00 88 04 00 00 00 00 00 03 8A 7F
```

需要连续正常模拟帧时，从`perception/`执行下面代码生成不同tick的HEX文本。它只打印，不打开串口；`flags=3`仅模拟软件有效条件，不表示实车已经完成标定：

```python
from rescue_vision.protocol import encode
from rescue_vision.types import Observation, Status, Target

for seq, tick in enumerate((1000, 1080, 1160)):
    target = Target(1, 90, 0, 500, 40, 40, 0, 48)
    observation = Observation(tick, 3, Status.OK, (target,))
    print(encode(observation, seq).hex(" ").upper())
```

用串口助手逐条发送时，实际间隔按需要控制。测试三个不同图像到位时，应按**实测车头偏移+200mm**构造径向距离（角度0且机构居中时）；当前固定500mm样例不保证处于到位区。

## 12. 主控交付与联调验收

先交付：选定UART、复用回调、C模块加入工程、10ms解析/失效任务、调试显示、统一运动命令出口。ARM编译、链接和实际中断接收必须在主控工程单独验收，主机GCC通过不能代替。

推荐按以下步骤验收：

1. 电机输出隔离，注入正常/空场/未标定帧，核对类别、置信度、flags、状态和数值。
2. 逐字节拆包、两帧粘包、噪声、错误CRC及错误长度后能恢复；坏包不产生新有效观测。
3. 正常帧后改发空场/故障，下一控制任务输出0；完全断流或只重复有效旧tick，300ms后输出0。
4. 左右目标符号正确，角度单位正确；图像坐标、实验坐标、底盘坐标和里程计坐标不混用。
5. 完成实车标定及底盘速度换算，再进行只转向、低速接近、危险阻断、实际停车延迟和到位锁存验收。`allow_forward=0`仍能转向，纯接收验收需由命令出口禁止全部运动。
6. 明确旧50cm试验逻辑及ASCII调试与视觉控制的仲裁规则，验证视觉输出0不会被其他任务重新覆盖为非0。

从`perception/`运行`python -m pytest -q tests/test_protocol_stm32.py`可复现24项主机协议/跨语言测试；v0.3.2完整软件套件114项通过，旧单电机兼容测试已移除，并覆盖旧数据/旧配置拒绝行为。当前`faraway_report()`此前经只读提取，在独立GCC夹具中输出JSON并通过解析核验。仓库未包含实车闭环验收通过的证据，不应把这些测试结果当成当前板内固件的停车保证。

## 13. 尚需双方确认，但不阻塞编写接收代码的项目

- 视觉所用主控UART、接头引脚和视觉板设备路径。
- 相机实车安装姿态、地面映射、底盘原点及两个机构偏移。
- 底盘类型、轮序/方向、轮径、编码器每轮有效计数和车体速度接口。
- 主控输出使能、故障锁存及与现有试验/调试命令的仲裁。
- 板端真实有效观测频率、全链路延迟、调度和机械停车时间。

双向任务命令、任务应答、稳定目标ID、场地区域、伤员姿态和完整救援流程属于后续接口范围。双方未定义前不发送自创指令，不把v1保留位当成已支持功能；新增布局或改变旧字段含义应升级版本，并同时更新发送端、接收端、文档及互通样例。
