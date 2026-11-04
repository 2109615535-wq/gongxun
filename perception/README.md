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
