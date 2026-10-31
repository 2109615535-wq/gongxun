# STM32F407VETx新主控交接（源代码只读）

> 以下为早期主控核验记录，接口和行号不是当前仓库主控的保证。最新四电机遥测、50cm试验逻辑及视觉接入说明见[主控接入README](../README_STM32.md)。视觉包v0.3.2的`stm32-status`只支持当前四电机格式，不再兼容下文的旧单电机遥测；默认仅接收，可选只发一次GET。独立C接收器仍未集成到实际主控。

已核对`E:/智能救援/STM32主控项目/project`。本次不修改、编译或烧录该项目，不增加HAL回调，不给UART5发送控制指令。以下说明独立模块与实际源码之间仍需由电控组处理的接入点。

## 当前工程的实际接口

- `main.c:153`注册`motor_process`周期20ms；`faraway_process`周期5ms、遥测50ms；使用合作式主循环调度。
- `faraway.h/.c`：UART5 PC12/PD2、115200/8N1，只控制电机2。支持`TARGET`、`PID`、`ENABLE`、`GET`；交接工具仅支持被动接收或单次`GET`。
- `encoder.c:82`求counts/s，`:124`使用`encoder.h:7`的543170归一化到±1000。
- `motor.c:151`之后的`MotorSetSpeed1..4`只更新PID目标，并不直接接受mm/s。头文件写的“PWM duty”与实现不一致，不应据此跳过速度单位换算。
- HDZ601用USART2 PA2/PA3；共享UART接收/错误回调在`hdz601.c:114`及后续位置，已转发UART5，不能新增同名函数覆盖它。
- USART3 PC10/PC11、USART6 PC6/PC7已初始化115200，但没有找到业务接收代码；是否能给视觉使用需确认实物接线。
- 尚无整车速度接口、观测帧接收或UART5断线停车。`main.c:166`将电机2目标设为5，随后使能；源代码与板内固件是否一致仍待实机核对。

因此不能把视觉方位、距离或本模块输出直接换成UART5的`TARGET`。也不能在没有接收器的端口发送观测帧后宣称整车可控。

## 本包独立C模块

`rescue_vision.c/.h`不依赖HAL，不分配动态内存，使用约5KB接收器对象。主机GCC测试已验证协议互通、危险阻断、新帧计数、源时间回绕和300ms超时。它尚未加入上述工程，ARM构建、任务调度与实车运行未验收。

主要接口：

- `RV_Init(receiver, config)`：配置已实测的车头和机构偏移；`allow_forward=0`用于以后仅转向试验。
- `RV_OnByte(receiver, byte, received_ms)`：只在单个接收来源投递字节；时刻来自本地主控毫秒时钟。
- `RV_Step(receiver, now_ms)`：在单个控制上下文解析并返回命令；建议10ms调用，始终执行返回的零速度。
- `RV_ResetTrial(receiver)`：停车锁存后显式开始新试验，仍须等待新图像。

输出`forward_mm_s`最大100mm/s；`clockwise_rad_s`最大0.4rad/s、顺时针为正。大于5°方位偏差先转向。到位要求车头前200±30mm、机构横向±20mm，连续三个不同有效图像满足后锁存停车。重复控制周期读同一快照不增加计数；过近不自动倒车。

## 电控组后续接入要求

本次不自动生成主控补丁。若电控组以后在获准修改的工程副本接入，应复用实际的`scheduler_add_task`、既有UART回调和电机闭环，先明确串口分配、接收器、看门狗及完整车体速度接口。

当前归一化速度换算必须补齐实测数据：

`raw_counts_s = normalized_speed * 543170 / 1000`

`wheel_mm_s = raw_counts_s * wheel_circumference_mm / effective_counts_per_wheel_revolution`

这些公式仍不足以决定四轮目标：还需实际轮径、有效每轮计数、四轮安装位置/方向、底盘类型与运动学。不能沿用旧工程轮序，也不能把归一化值100当成0.1m/s。543170在代码中由所有通道共用，其对每个实车电机是否合适尚未验收。

`RV_Step`的300ms阈值是控制命令失效阈值。当前20ms电机任务和合作式调度可能增加执行延迟，物理停止还受制动影响；不能把主机单测当成当前固件的断线停车保证。接入后要实际测量调度、停止命令执行和机械停车时间。

先只接收和显示，再确认左右转向和单位，最后开放≤100mm/s前进。上述工作必须在整车接口和独立失效停车具备后进行；当前版本只完成接口交接和感知工具，不开展电机试验。

## 未修改源码的校验

交接包包含审阅前源/工程文件哈希与审阅后核验结果，记录在`artifacts/hardware_review/`。校验工具仅读取源文件，输出必须位于工程外：

```bash
python tools/verify_stm32_readonly.py --project ../STM32主控项目 --baseline artifacts/hardware_review/stm32_source_before.json --output artifacts/hardware_review/recheck.json
```

1306个源/工程文件的哈希覆盖`.c/.h/.s/.ioc/.uvprojx/.uvoptx`，不包含`.git`。该结果证明本次审阅期间这些文件未变更，不证明板内烧录内容与磁盘工程一致。
