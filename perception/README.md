# 视觉→主控 v1 协议交接

本目录只保留观测协议、独立C接收与单普通物资接近参考模块，以及Python协议编解码工具。主控可据此开发接收、校验、坐标还原、超时处理和接近停车，不需要安装或运行视觉识别程序。

相机采集、YOLO推理、权重、标定数据、串口运行器和部署工具已从本目录移除。本次不修改仓库的`project/`主控工程，不改变v1线上字节布局，也不表示实际固件已经接入视觉。

## 2026-11-02 更新依据

依据本地视觉工程现有的统一坐标、双摄工作台、`dashboard.py`串口适配器及`dashboard_state.py`状态生成逻辑更新本交接包，继续只保留7个文件。

- 工程坐标统一为X向右、Y向前，单位mm。二进制v1仍发送径向距离和向右为正的方位，字节布局、CRC及物理含义不变。
- 本地电脑端已实现上视主要测距、下视近距离辅助与夹爪区域类别核验；这些视觉运行代码不随本包上传。
- 本文新增双摄JSON接口草案，仅记录现有本地适配器行为，须与主控负责人确认后接入。它不是二进制v1的新字段，现有C接收器和Python编解码器均不解析该JSON接口。
- 当前上视仍是局部检查、下视仍是拟合阶段，摄像头绑定、完整实车运动标定和夹爪区域核验待完成；本地默认`motion_valid=false`。主控串口实机、香橙派双摄性能与整车闭环仍待验收。

## 文件与阅读顺序

- [本文完整协议](#完整观测协议-v1)：字节布局、CRC、单位、类别、状态/标志、时序、坐标还原和合成HEX报文。
- 本文“双摄JSON接口草案”部分：主控任务下发、双摄观测回传、有效性和待确认项，不代表正式固件已实现。
- [stm32/rescue_vision.h](stm32/rescue_vision.h)：接收器、配置、观测、控制输出及公开C接口。
- [stm32/rescue_vision.c](stm32/rescue_vision.c)：流式组帧、字段与CRC校验、重复图像防保活、危险阻断、300ms失效和接近停车参考实现。
- [rescue_vision/types.py](rescue_vision/types.py)、[protocol.py](rescue_vision/protocol.py)、[__init__.py](rescue_vision/__init__.py)：只用于协议验证的Python类型与编解码，不含图像处理或串口发送。
- [.gitignore](.gitignore)：排除本地缓存和实验输出。本文件是使用入口；Git版本共保留7个文件。

## 完整观测协议 v1

本文、[Python编解码器](rescue_vision/protocol.py)及[C接收器](stm32/rescue_vision.c)共同构成v1交接基线。仅精简交付文件，原帧格式、CRC、字段含义和C参考策略保持不变。主控接入步骤见本文“主控接入”部分。

### 1. 链路和边界

- 单向视觉TX→主控RX，3.3V TTL、交叉接线并共地，8数据位、无校验、1停止位。联调115200bps；协议也允许921600，但双方必须同步配置，波特率不在帧内传输。
- 二进制帧采用小端；有符号整数是二进制补码，不按换行符分包，不使用`strlen`或直接强转C结构体。
- 视觉主动发送新观测，不需要主控请求、ACK或重传；正常空场、故障、目标丢失也要发送无效观测，不能只静默停止。
- 二进制v1不定义主控到视觉的任务、应答、稳定目标ID和完整转运任务协议。本文另列的JSON任务草案属于独立接入契约；区域与姿态预留不能视为已实现功能，改变二进制布局或既有语义须升级版本。

### 2. 外层帧

```text
AA 55 | version:u8 | sequence:u8 | payload_length:u16 | payload | CRC16:u16
```

所有偏移从0开始：

- 整帧偏移0～1：固定帧头`AA 55`。
- 偏移2：`version`，必须为1。
- 偏移3：`sequence`，每次发送递增，255后回0；仅供诊断，不能代替图像新鲜性。
- 偏移4～5：`payload_length`，仅计payload字节数，范围8～184。
- 偏移6开始是payload；最后2字节是CRC，低字节先发。

```text
payload_length = 8 + 12 * target_num + 8 * zone_num
total_length = payload_length + 8
```

空观测16字节，一个目标28字节，最多192字节。115200/8N1发送最大帧约16.7ms；30Hz最大帧约占链路50%。观测频率不由协议保证。

### 3. payload固定头：8字节

以下偏移相对于payload，整帧偏移需加6：

- 0～3：`tick_ms:u32`。图像观测为视觉读取该帧完成时的本地单调毫秒时刻，故障心跳为发送侧当前时刻，允许32位回绕。
- 4：`frame_flags:u8`。
- 5：`target_num:u8`，0～12。
- 6：`zone_num:u8`，0～4，当前发送端固定0。
- 7：`status:u8`，0～7，定义见下文。

### 4. 目标项：每项12字节

目标数组从整帧偏移14开始。以下偏移相对于单个目标：

- 0：`type:u8`，1普通物资、2核心物资、3伤员、4危险目标、255未知。
- 1：`confidence:u8`，0～100，不是0～1浮点数。
- 2～3：`angle_cdeg:i16`，单位0.01°，范围-18000～18000，向右为正。
- 4～5：`distance_mm:u16`，底盘原点到贴地点的地面径向距离；0表示无效，非0还须检查有效标志。
- 6～7：`width_px:u16`，检测所在图像分辨率下的框宽。
- 8～9：`height_px:u16`，相同分辨率下的框高。
- 10：`posture:u8`，当前发送0，表示未知，不据此驱动机构。
- 11：`target_flags:u8`。

宽高只供诊断，不是固定补边640坐标，主控不据此重新猜测物理距离。协议类别不等于YOLO原始class_id。

目标后面是`zone_num`个8字节预留项，当前只校验数量和总长度并跳过，不定义其内部字段，不用于运动。

### 5. CRC与结构校验

CRC16/CCITT-FALSE：poly=`0x1021`、init=`0xFFFF`、不反射、xorout=`0`。从整帧偏移2的version开始，计算到payload结束，不含帧头和CRC自身。CRC按小端发送；标准串`123456789`结果为`0x29B1`。

接收器校验帧头、版本、payload长度、实际总长度、CRC、目标/区域数量、数量对应的精确长度、状态、目标类别、置信度和角度范围。不以结构体对齐或`sizeof(RV_Target)`决定线上长度。

CRC、格式和字段错误帧不发布为新观测，也不刷新有效期。单个坏CRC不会立即撤销上一有效观测；上一观测仍按本地300ms有效期失效。与无效观测主动撤销的语义不同。

### 6. 状态与标志

`status`：

- 0 `OK`：当前普通目标流程已确认，仍须同时检查有效标志及危险条件。
- 1 `NO_TARGET`：没有当前可用普通目标，不保证画面完全没有其他类别。
- 2 `CAMERA_FAULT`：相机故障。
- 3 `CALIBRATION_INVALID`：标定无效。
- 4 `TARGET_INVALID`：目标/位置不可信，或连续帧确认尚未完成。
- 5 `TARGET_LOST`：此前确认的目标丢失。
- 6 `STALE`：图像、推理结果过期或未及时取得新结果。
- 7 `INFERENCE_FAULT`：推理异常。

`frame_flags`按位检查：

- bit0 / `0x01`：视觉观测有效。
- bit1 / `0x02`：标定已验证。
- bit4 / `0x10`：接近通道被危险或未知目标阻断。
- bit2安全区可见、bit3近场沿用预留，当前发送端不置位，接收器不据此控制运动；其余位未定义。

`target_flags`：

- bit0 / `0x01`：危险。
- bit2 / `0x04`：遮挡或截断。
- bit4 / `0x10`：连续帧确认。
- bit5 / `0x20`：位置有效。
- bit1姿态可信、bit3进入铲入区尚未实现，主控不可依赖；其余位未定义。

`0x10`在帧标志中是阻断，在目标标志中是连续确认。置信度高不能替代标定、连续确认和位置有效性。

### 7. 坐标还原和安装约定

测量点是**可见贴地边缘中点**，不是物资几何中心。工程统一采用相对于约定车辆原点的X向右、Y向前，方位向右为正：

```text
angle_rad = angle_cdeg * pi / 18000
x_right_mm = distance_mm * sin(angle_rad)
y_forward_mm = distance_mm * cos(angle_rad)

gap_mm = y_forward_mm - bumper_forward_mm
lateral_right_mm = x_right_mm - collection_center_right_mm

# 现有C公开接口保留向左为正的命名，供兼容使用：
collection_center_left_mm = -collection_center_right_mm
lateral_mm = -lateral_right_mm
```

正前方角度为0；右侧目标角度为正，还原的`x_right_mm`为正。`gap_mm`是车头前缘到贴地点的前向间隙；`lateral_right_mm`向右为正，C输出`lateral_mm`仍向左为正，二者符号相反。

双方共同固定车辆原点，实测前向车头偏移`bumper_forward_mm`和向右机构偏移`collection_center_right_mm`，并使用一致数值。现有`RV_Config`没有`collection_center_right_mm`字段，须把它取负后赋给已有的`collection_center_left_mm`。本次不修改公开C结构体布局；示例中的120mm等参数只是合成数据，不代表实车安装值。

相机内参、畸变、地面映射与外参留在视觉侧，不随每帧发送。电脑实验与实车均用X向右、Y向前，但原点、安装朝向和偏移仍须核对；原点和朝向一致时不交换X/Y或改变符号，否则先做实测平移和旋转变换。编码关系为：

```text
distance_mm = round(hypot(x, y))
angle_cdeg = round(degrees(atan2(x, y)) * 100)
```

非有限值、越出u16距离/角度编码范围或实测有效区域的位置不得截断成有效观测。安装关系或地面映射未验证时，应清除观测/标定有效位并发送`status=3`。可以使用合成报文开发通信，不能把模拟有效标志当成实车标定通过。

这是当前车体相对位置，单位mm，不能直接当成主控里程计的绝对位置或厘米量。

### 8. 运动准入和危险阻断

参考C模块仅在以下条件全部满足时允许计算接近输出：

- 帧bit0和bit1均置位，`status=0`且无帧阻断。
- 恰有一个普通物资，置信度≥35，目标bit4/bit5均置位，非危险、非遮挡，距离非0。
- 安装参数有限，车头前向偏移非负，观测未超时。
- 无未知目标；危险目标位置不可信时直接阻断。危险目标位置可信时，以同样坐标还原，处于车头前方`0 < gap <= 1400mm`且机构中心线左右180mm内则阻断。

协议可携带12目标不等于已实现12目标跟踪、目标锁定或全部类别转运。核心/伤员项不能自动作为普通物资接近策略的执行目标。

### 9. 时序、超时和接收恢复

- 使用STM32本地收到字节的毫秒时刻；一帧接收时刻为末字节时刻，不把视觉tick直接减STM32 tick。
- 到本地接收时刻的间隔**达到300ms**时撤销运动，未收到字节时仍需周期调用`RV_Step`。
- 同一图像tick的重复有效帧不刷新有效期，不增加到位确认；改变sequence不能绕过。超过300ms后，同一tick仍拒绝。
- 在距上次接受的源时间基准接收间隔短于300ms时，源tick倒退会拒绝；间隔达到300ms后，不同tick可重新建立源时间基准，兼容视觉重启。源时间和主控时间均按u32回绕处理。
- 故障心跳撤销当前有效观测，但不推进最后接受的有效图像源时间，避免拒绝仍在处理中的较早图像。故障/空场/目标丢失不等待300ms。
- 残帧相邻字节间隔**超过30ms**会重置残帧；这不是整帧必须在30ms内完成的规则。噪声、坏头、坏版本/长度或CRC错误后继续搜索下一帧。
- 接收环为512槽，保留一槽区分满/空。溢出时下一次`RV_Step`清除接收队列和观测并输出零速度。115200满速输入约44ms可填满511槽，应避免控制任务长期阻塞。

“立即撤销”指下一次控制任务执行。300ms是软件命令有效期，调度、驱动执行、制动与惯性仍会影响物理停车时间。

### 10. 单普通物资接近参考逻辑

这是C模块的主控参考策略，不是额外线上命令：

- 状态为`RV_WAIT / RV_ALIGN / RV_APPROACH / RV_STOPPED`，输出同时携带原因码。
- 车头间隙小于170mm时输出0，不自动倒车。
- 间隙170～230mm且机构横向误差≤20mm时输出0。三个不同且被控制任务消费的新图像均到位，进入`RV_STOPPED`锁存；重复图像或同一快照的重复控制周期不计数。到位条件中断时计数清零。
- 其他情况下，转向误差为`atan2(-lateral_mm, y_forward_mm)`，等价于`atan2(lateral_right_mm, y_forward_mm)`，顺时针角速度为其1.5倍，限幅±0.4rad/s。
- 误差绝对值大于5°、间隙≤230mm或`allow_forward=0`时只输出转向。
- 允许前进且已对准时，前向速度为`0.6 * (gap_mm - 200)`，限幅100mm/s。
- 到位锁存须由控制上下文显式调用`RV_ResetTrial`开始新试验，仍等待新图像。不要让其他控制来源覆盖零输出。

C接口、静态对象、HAL回调和底盘速度单位适配见本文“主控接入”和“速度适配与参考接近策略”部分。

### 11. 合成HEX样例

以下只供电机输出隔离后的编解码测试。串口助手选择HEX发送，不附加文本换行，不持续重发同一tick维持运动。

正常观测：sequence=0、tick=1000、帧flags=3、status=0；一个普通物资，置信度90、角度0、距离500mm、宽高40px、姿态0、目标flags=48，共28字节：

```text
AA 55 01 00 14 00 E8 03 00 00 03 01 00 00 01 5A 00 00 F4 01 28 00 28 00 00 30 03 1C
```

正常空场：sequence=1、tick=1080、flags=2、status=1，无目标，共16字节：

```text
AA 55 01 01 08 00 38 04 00 00 02 00 00 01 07 71
```

标定无效：sequence=2、tick=1160、flags=0、status=3，无目标，共16字节：

```text
AA 55 01 02 08 00 88 04 00 00 00 00 00 03 8A 7F
```

需要不同tick时，在`perception/`目录执行以下Python代码；它只打印HEX，不打开串口：

```python
from rescue_vision.protocol import encode
from rescue_vision.types import Observation, Status, Target

for seq, tick in enumerate((1000, 1080, 1160)):
    target = Target(1, 90, 0, 500, 40, 40, 0, 48)
    frame = encode(Observation(tick, 3, Status.OK, (target,)), seq)
    print(frame.hex(" ").upper())
```

500mm样例不保证到位。测试到位需按实测车头偏移+200mm构造距离，并满足角度0和机构居中条件；例如偏移120mm时，对应距离320mm。

主控接入和硬件验收步骤见本文相应部分。合成报文使用人工观测，不证明视觉识别准确率、实际标定或整车停车效果。

## 主控接入

1. 将`stm32/rescue_vision.c`加入构建目标，增加头文件搜索路径并链接数学函数。模块仅使用标准C和`stdint.h`、`string.h`、`math.h`，不依赖HAL、电机驱动或动态内存。约5KB的`RV_Receiver`建议静态分配。
2. 双方确认视觉UART、3.3V TTL接线、底盘原点与实测安装偏移。联调使用115200/8N1，TX/RX交叉、共地。当前UART5的ASCII电机调试及JSON遥测不等于支持v1二进制观测。
3. 在开启接收中断前调用`RV_Init`。在已有UART回调的视觉分支调用`RV_OnByte(receiver, byte, HAL_GetTick())`，随后重新启动接收并检查返回值；复用现有回调，不新建第二个同名全局HAL回调。只允许一个字节生产者，UART错误恢复由集成代码处理。
4. 在单个控制上下文每10ms调用`RV_Step(receiver, HAL_GetTick())`，即使无数据也必须调用。ISR只投递字节，不解析或驱动电机；同一控制上下文可读取观测及错误计数。
5. 纯接收阶段只显示结果并禁止驱动输出。完成单位、方向及停车验收后，由唯一运动命令出口执行返回速度，**包括零速度**；明确与开机试验、ASCII调试及其他任务的仲裁，避免旧命令覆盖停车。
6. `RV_STOPPED`为到位停车锁存。新试验在控制上下文显式调用`RV_ResetTrial`，仍需等待新图像，不能每周期自动复位。

公开接口以头文件为准：

```c
void RV_Init(RV_Receiver *receiver, RV_Config config);
void RV_OnByte(RV_Receiver *receiver, uint8_t byte, uint32_t received_ms);
RV_Command RV_Step(RV_Receiver *receiver, uint32_t now_ms);
void RV_ResetTrial(RV_Receiver *receiver);
uint16_t RV_Crc16(const uint8_t *data, uint16_t length);
```

`RV_Config`中的`bumper_forward_mm`必须与视觉标定所用前向安装值一致；`collection_center_left_mm`为视觉向右机构偏移`collection_center_right_mm`的相反数。C内部使用`forward_mm/left_mm`显式命名，不再将它们简称为工程X/Y。`allow_forward=0`只禁止前进，仍可能输出转向请求，不等于禁止全部运动。

## 速度适配与参考接近策略

输出`forward_mm_s`是前向速度，最大100mm/s；`clockwise_rad_s`是顺时针角速度，幅值最大0.4rad/s。偏角大于5°时先转向；车头间隙200±30mm、机构横向误差±20mm，三个不同且被控制任务消费的新图像满足后锁存停车。过近不自动倒车。详见本文完整协议的参考策略。

输出不是PWM，也不是`MotorSetSpeed1..4`的PID目标。先通过真实底盘运动学得到各轮线速度，再按实测轮径、有效每轮编码器计数和归一化基准换算：

```text
raw_counts_s = wheel_mm_s * effective_counts_per_wheel_rev / wheel_circumference_mm
normalized_target = raw_counts_s * 1000 / ENCODER_SPEED_AT_PWM1000
```

还须验证轮序、方向、限幅和转弯符号；不能把100mm/s直接写成电机目标100。视觉坐标是当前车体相对坐标、单位mm，不能直接当作主控里程计的绝对坐标或厘米数值。

## 合成报文联调

本文完整协议给出可供串口助手HEX发送的合成报文，以及用Python生成不同tick报文的示例。Python工具只依赖Python 3.10及以上的标准库，不打开串口。正常样例带有效标志，仅用于隔离的接收测试；示例中的120mm等安装偏移不是实车测量值。

本交付不包含自动化测试脚本或C测试夹具。接收、校验、坐标、超时和接近行为须在主控集成时按协议验收。

## 双摄JSON接口草案（待主控确认）

本节按2026-11-02本地`ControllerLink`、`DashboardState`和双摄观测生成逻辑记录接口，供双方评审，**不是已冻结或已接入固件的正式协议**。本包不包含双摄运行器、JSON解析器或任务状态机；`stm32/rescue_vision.c/.h`只接收前述二进制v1。

### 1. 传输与启用

- 草案模式名为`rescue_json_v1`，须在本地双摄运行器中显式启用；默认`motor_debug_ascii`仅接收现有四电机遥测，不发送JSON观测，也不把遥测视为救援任务。
- 串口115200/8N1、3.3V TTL、交叉接线并共地。传输UTF-8 JSON对象，每条消息以换行结束；当前视觉发送器使用ASCII转义的JSON文本，属于UTF-8的兼容子集，末尾加`\n`。接收器接受`\n`或`\r`分隔，兼容CRLF。
- 处理拆包、粘包和空行。当前视觉任务接收单行最多2048字节，不含结束符；超长后丢弃至下一个换行并计无效行，坏JSON或非法任务不刷新有效期。
- JSON消息没有二进制帧头、长度字段或CRC，也没有ACK、重传和主控动作命令。不要把JSON发给`RV_OnByte`期望它解析，不在同一串口无约定地混发JSON、二进制v1和ASCII电机命令。

### 2. 主控→视觉：任务与心跳

合成示例，`VERIFY`只是说明用状态名，不是双方已冻结的状态枚举：

```json
{"type":"rescue_task_v1","session_id":"boot-001","seq":1,"task_id":"collect-001","state":"VERIFY","expected":"normal_material"}
```

全部字段均须存在：

- `type`：固定字符串`rescue_task_v1`。
- `session_id`：1～64字符的启动会话标识。主控重启或seq耗尽时使用新会话；旧会话最后接受的任务仍在1000ms窗口内时，视觉拒绝另一会话切入。
- `seq`：0～4294967295的整数，布尔值无效；同一会话严格递增，每次任务心跳递增。同一会话的重复或倒序值始终拒绝，即使已经超时；不按u32回绕接受旧序号。
- `task_id`：1～64字符的任务标识。换目标或需要重新开始核验的重试，应换任务编号；不是物体跟踪ID。
- `state`：1～64字符的主控状态名，视觉按原值保存并参与核验重置；合法状态枚举及动作语义仍待主控确认。
- `expected`：`normal_material`、`core_material`、`casualty`、`red_material_zone`、`blue_material_zone`、`red_casualty_zone`、`blue_casualty_zone`，或显式`null`。不接受`hazard`作为任务目标；`null`不选择任务目标。

主控周期回传同一任务时，保持会话、任务、状态与类别不变，只递增seq。视觉按本地接收时刻维护任务有效期，当前超过1000ms未接受新任务则任务失效；拒绝帧不刷新该时刻。会话、任务、类别或状态改变会使旧核验结果失效，重新累计连续图像。

本地网页的联调任务使用`desktop`来源。见过有效主控会话后，主控失联不自动恢复网页联调任务；主控也不应接受不匹配自身当前会话/任务的观测。

### 3. 视觉→主控：双摄观测

当前约每100ms尝试发送一条`rescue_observation_v1`，实际周期受串口读写和线程调度影响，不保证固定10Hz，也不等于两路推理帧率。以下为人工构造、保持运动无效的字段示例：

```json
{
  "type": "rescue_observation_v1",
  "task_id": "collect-001",
  "session_id": "boot-001",
  "tick_ms": 10000,
  "source": "upper",
  "expected": "normal_material",
  "motion_valid": false,
  "hazard_blocked": false,
  "position_mm": {
    "x_right": 30.0,
    "y_forward": 500.0,
    "ground_range": 500.89919145472777,
    "bearing_deg": 3.433630362450522
  },
  "position_quality": "local_checked",
  "visual_target": {
    "name": "normal_material",
    "confidence": 0.9,
    "box": [300, 200, 340, 240],
    "confirmed": true,
    "center_offset_normalized": 0.0
  },
  "gripper_status": "UNCONFIGURED",
  "gripper_match": false,
  "upper_status": "OK",
  "lower_status": "OK"
}
```

字段约定：

- `type`固定为`rescue_observation_v1`。`session_id/task_id/expected`对应当前任务，任务失效时可为`null`；主控须与自身当前任务匹配，不把换任务后的旧结果当作本次结果。
- `tick_ms`为视觉**生成发送快照时**的单调毫秒时刻取u32，不是图像采集时刻。这与二进制v1有效观测的tick语义不同，不能用它单独证明新图像到达，也不能与主控时钟直接相减。
- `source`是测距来源`upper`、`lower`或`null`。优先采用上视带位置的选中目标，无可用上视位置时可显示下视辅助位置；下视替代不授权整车前进。
- `position_mm`为对象或`null`。`x_right/y_forward`分别为向右/向前坐标，`ground_range`为地面径向距离，均为mm；`bearing_deg`为向右为正的角度，单位**度**，不是二进制的0.01°。车辆原点和机构偏移仍须按实测安装关系确认。
- `position_quality`为`validated`、`local_checked`、`fit_only`、`near_estimate`、`bbox_ground_estimate`或`null`。局部检查、仅拟合、外推和框底边估算均不能当作完整运动标定通过；是否允许运动以有效性条件为准。
- `visual_target`是主要图像目标或`null`，优先上视、其次下视，不保证它与`source`来自同一路。`name`为类别名称；`confidence`为0～1数值，区别于二进制0～100；`box`为原图像素`[x1,y1,x2,y2]`；`confirmed`为连续帧确认；`center_offset_normalized`为框水平中心相对图像中心的归一化偏移，向右为正，范围约-1～1，不是毫米距离。
- `motion_valid`为布尔值，只表示当前视觉运动条件是否满足，不是使能或速度命令。必须结合任务匹配、接收超时、相机状态和主控自身运动仲裁。
- `hazard_blocked`为双路危险/未知类别阻断汇总。该值为false不能独立证明通道安全，尤其在故障或图像过期时。
- `gripper_status`为下视区域核验状态字符串；`gripper_match`是经过主控任务、双路新鲜性和危险检查后的布尔结果，不是原始类别相符值。
- `upper_status/lower_status`为相机观测状态字符串，如`OK`、`DISABLED`、`CAMERA_FAULT`、`CALIBRATION_ERROR`、`STALE`、`REPLAY_ENDED`等；正式状态枚举待双方确认。非OK状态不可仅因仍有数值而继续运动。

### 4. 双摄有效性与夹爪核验

上视承担主要测距，下视承担近距离辅助及夹爪区域核验。当前配置的上视显示范围为前向100～800mm，其中100～150mm是近场外推；下视为0～300mm，其中0～100mm是外推。范围来自当前标定/配置，不是v1固定量程，也不代表全范围精度或实车运动标定已通过。

运动有效要求上视具有已验证的普通物资位置、连续新帧确认、上下两路均为OK且年龄不超过150ms、有效主控任务，以及无危险/未知阻断。下视有距离而上视失效时，`motion_valid`仍为false。视频回放不授权运动。当前交付所依据的本地标定未达到上述实车运动条件，所以默认不开放运动有效标志。

框位于下视夹爪ROI的目标不再经地面H映射计算位置，避免把离地物体当成贴地物资。区域核验的现有工程初值为置信度≥0.35、框面积至少70%在ROI内、关联IoU≥0.2、至少3个连续新鲜图像，均待实拍验证。

区域状态包括：`UNCONFIGURED`未配置；`WAITING_TASK`无适用物资任务；`CONFIRMING`确认中；`MATCH`类别相符；`MISMATCH`稳定检出其他类别；`EMPTY`连续未检出；`AMBIGUOUS`多个候选；`HAZARD`危险目标。未验证ROI时，稳定结果带`_UNVERIFIED`后缀且match保持false；故障、过期或任务切换后还可能输出`STALE`、`WAITING_TASK_FRAME`等无效状态。

只有区域已验证、单物体与当前任务相符且连续稳定，并满足主控任务有效、双路新鲜且无阻断时，`gripper_match`才为true。`EMPTY`也可能是漏检，`MATCH`不能替代机构闭合、到位或接触传感器反馈。

### 5. 时序区别与待冻结项

- 视觉读取状态和发送观测时重新检查年龄：超过150ms禁止控制有效，超过300ms清除显示目标；任务超过1000ms失效。这些是本地双摄当前配置，不替代二进制C模块的300ms接收有效期。
- JSON的发送tick持续变化也不代表图像更新。当前消息没有独立观测序号、双路图像采集tick或年龄字段，主控不能照搬二进制“不同tick即新图像”的计数逻辑；正式方案应确认是否补齐这些字段及去重规则。
- 主控须用自己的本地到达时间维护JSON接收看门狗，并对任务失配、无效观测、相机故障和失联撤销动作。主控端具体超时阈值、制动与恢复策略仍待确认，本文不把视觉的1000ms任务接收阈值指定为车辆停车阈值。
- 现有草案不回显主控任务seq，不提供ACK、命令重传、CRC或动作完成确认。视觉发送端没有另设固定JSON观测帧长上限，主控缓冲容量、必选字段与未知字段策略需一起冻结。
- 115200下约10Hz的JSON发送不代表已达到项目≥15Hz有效观测目标。正式接口需确认输出频率、帧大小、串口带宽与完整延迟。
- 最终主控源码副本、实际烧录版本、UART引脚、`state`枚举、任务切换/重试语义、标定原点、ROI、停止策略及采用二进制或JSON的取舍均待确认。仓库现有UART5遥测不能视为已实现该双向草案。

## 硬件验收与范围

先在输出隔离条件下验收正常/空场/故障帧、拆粘包、坏CRC、字段错误、左右坐标符号及重复tick。随后验证完全断流、危险目标及无效观测能撤销命令，并实测调度、驱动执行与机械停车延迟。

300ms是软件命令有效期，不能视为车辆物理停止时间。合成报文核对也不能代替ARM编译、UART中断、实际标定和实车停车验收。

二进制v1只定义视觉→主控观测；下文JSON草案不向二进制协议增加任务命令、ACK或目标ID。全场定位、安全区导航和完整转运流程仍须后续定义。新增二进制布局或改变既有字段含义时须升级版本并同步双方，不把预留字段当成已实现能力。
