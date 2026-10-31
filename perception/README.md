# 智能救援：香橙派C100感知交接包 v0.3.2

**GitHub版首次使用请先阅读[启动说明](START_HERE.md)**：包含仓库内模型和标定文件的相对路径启动命令。当前改动仅限`perception/`，不修改`project/`主控。主控接入以[README_STM32.md](README_STM32.md)为准；早期实验与交接记录中的机器路径和设备状态不能直接视为当前环境。

已复用原RKNN完成香橙派真机采图和推理核验。先阅读[第三版说明](HANDOFF_V3.md)；[第二版说明](HANDOFF_V2.md)保留为早期接口交接记录。新STM32主控源码保持只读。

## 2026-10-31：适配最新主控遥测

按仓库主控`d915341`核对UART5接口，`stm32-status`只解析四电机`speed1/target1`到`speed4/target4`及`y_body`，全部九个字段必须存在且合法；不再支持旧单电机格式。速度值仍为±1000归一化值；`y_body`是主控里程计厘米量，不是目标前向距离或毫米值。

JSONL记录附`telemetry_format`、`speed_unit`及四电机记录的`y_body_unit=cm`。命令应答单独统计，错误应答保留数量和最后一条文本；缺字段、布尔值、非有限值和越界速度不计为有效样本。

调试配置只接受`motor_debug_ascii`，旧名称`motor2_debug_ascii`已删除，使用它的配置需更新。工具默认仅收，`--query`最多发一次`GET`；不发送`TARGET/PID/ENABLE/RUN`或视觉二进制帧。视觉v1的字节布局没有变化，也未新增主控视觉接收能力。

v0.3.2完整主机套件114项通过，覆盖四电机格式、拒绝旧数据/旧配置、非法数值、拆包恢复及只读发送约束。旧单电机解析函数和兼容分支已删除。此前还只读提取当前主控的`faraway_report()`，在独立GCC夹具中生成实际JSON并通过解析；未构建、修改或烧录主控工程，也未连接串口或电机。

## 2026-10-30：电脑端绿色物资测距

已接入本台C100实测内参与离车地面映射。运行[一键启动入口](tools/run_green_distance.cmd)，或在已有yolo环境运行[检测脚本](tools/detect_green_distance.py)，详见[使用说明](GREEN_DISTANCE.md)。

独立6点检查最大横向/前向误差为6.6/14.0mm；绿色物资单张照片检查误差为1.2/19.9mm。脚本15项测试通过；近处新增橙色NEAR_ESTIMATE模式，默认前向0～150mm、横向在原实测范围内输出未验证估算，已用运行日志回放核对。此前电脑连接C100短时5秒处理124帧，其中2帧确认、122帧正常输出。以上为当前离车实验，不能代替板端实时性能或实车验收；config/calibration.json仍保持未验证。

## 原香橙派与主控核验记录

- 用户已完成香橙派Linux生成RKNN及USB相机YOLO检测，相机确认为WHEELTEC C100。
- `E:/智能救援/STM32主控项目/project`为STM32F407VETx工程。UART5是115200单电机ASCII调试接口，尚无视觉接收和整车速度接口。本次不修改、编译或烧录它。
- 厂商资料提供参考内参和标定教程，不等于本车完整标定。`config/calibration.json`保持未验证。
- 用户已确认相机正向固定，尚未重新采图核验；实车标定计划待定。下述倒置画面及性能数据保留为此前测试记录。
- 已通过SSH核对当前地址`10.61.67.250`。Python环境、原项目、模型、运行库和C100设备路径已确认，见第三版。

## 已核验的香橙派配置

从解压包根目录执行，不改原项目，不发送UART：

```bash
/home/orangepi/venvs/yolo-rknn-board/bin/python -m rescue_vision run --config config/orangepi_existing.json --record-video --duration 10 --output recordings/c100_01
```

配置沿用`/home/orangepi/yolo_deploy`的模型和运行库，使用C100的稳定UVC节点，MJPG/640×480/30fps；不会把HDMI `/dev/video0`或元数据节点当作图像输入。

本次短时实测处理约12.7～12.9fps、采集后到记录延迟P95约137～145ms，尚未达到15Hz目标。参考照片四类目标识别通过，随后绿色实物片段的90帧均出现普通物资类别。画面倒置且有额外边缘框，本车标定未验证，运动有效标志始终为0；这些类别检出记录不代表目标准确率、米制定位或整车验收。

## 安装与测试

命令从本目录执行。香橙派优先用现有检测脚本的Python解释器，不自动升级已有可用环境：

```bash
python -m pip install -e . --no-deps
python -m rescue_vision --help
```

依赖NumPy、OpenCV和pyserial；主机测试另需pytest和GCC。现有检测输出接入接口不导入RKNN、Torch或ONNX Runtime。Windows开发环境：

```powershell
& 'C:\Users\LENOVO\miniconda3\envs\yolo\python.exe' -m pytest -q
```

## 复用当前检测程序

参考`examples/existing_detector_hook.py`，保留已有模型、采集、预处理和后处理：

```python
import time
from rescue_vision.session import DetectionSession, detections_from_arrays

session = DetectionSession.from_config("config/vision.json")
# 在原有read成功后、推理之前记录：
captured = time.monotonic()
# 原有推理和后处理完成后，传入实际结果：
detections = detections_from_arrays(boxes_xyxy, scores, class_ids, actual_class_names)
observation = session.observe(original_bgr_frame, detections, captured, pixel_space="raw")
```

数组来自原程序实际后处理，框必须还原到原图，不能传补边640坐标。原图与标定尺寸一致；原图用`raw`，已在相同K坐标系去畸变的图用`undistorted`。接口只返回观测，不打开硬件或发送电机命令。

现有RKNN采用Ultralytics导出布局，由板端现有后端解码，不用本包参考模型的九输出解码器。新配置已读取实际10类名称表；更新模型后须重新核对校验值与类别表。

## C100标定

需要本台C100实测K/D、安装偏移、地面映射和独立验证。`config/wheeltec_reference.json`仅供参考，导入器会拒绝把它当成实测标定。

已有本台相机的OpenCV YAML/XML或JSON时：

```bash
python -m rescue_vision import-intrinsics --input camera_params.yml --image-size 640 480 --output artifacts/intrinsics.json
```

必须有K和畸变；分辨率缺失时提供真实`--image-size`，存在时检查一致，不自动缩放或补零。导入后仍为`validated=false`。

尚未实测时，用提供的A4板，内角9×6、名义格长25mm，打印后尺量确认：

```bash
python -m rescue_vision capture --camera /dev/v4l/by-id/usb-HJ_USB_2.0_Camera_HJ_USB_2.0_Camera_SN0001-video-index0 --width 640 --height 480 --count 25 --interval 2 --show --output recordings/chessboard_01
python -m rescue_vision intrinsics --images recordings/chessboard_01 --pattern 9 6 --square-mm 25 --output artifacts/intrinsics.json
python -m rescue_vision undistort --calibration artifacts/intrinsics.json --image 地面标记原图.png --output artifacts/ground_undistorted.png
```

至少12张有效图、RMS≤0.7px。参数25必须匹配尺量；其他采集分辨率应同步改命令，不能直接套640×480参考参数。

在去畸变地面图上按`config/ground_points.example.json`格式填写至少6点。底盘原点固定，x向前、y向左、单位mm：

```bash
python -m rescue_vision ground --calibration artifacts/intrinsics.json --points 地面实测点.json --output artifacts/ground_unvalidated.json
python -m rescue_vision validate-ground --calibration artifacts/ground_unvalidated.json --points 独立验证点.json --output config/calibration.json
```

验证点不能复用拟合点；至少6点，停车区域至少2点，车头前≥1m点至少1点；前向误差≤30mm、横向≤20mm。有效区域覆盖车头前200±30mm、横向±20mm。示例坐标不是本车参数。

实测`bumper_forward_mm`、`collection_center_left_mm`并与电控保持一致。车体遮挡、减速带上俯仰变化等工况不属于当前平地映射的有效范围。

## 当前主控状态读取

UART5 PC12 TX、PD2 RX，交叉接线、共地、3.3V TTL、115200/8N1；USART2是IMU口，不复用。

```bash
# 默认只收已有JSON遥测，不发送命令
python -m rescue_vision stm32-status --port /dev/ttyUSB0 --duration 5 --output recordings/stm32_status_01.jsonl
# 可选只发一次GET，不发TARGET/PID/ENABLE/RUN
python -m rescue_vision stm32-status --port /dev/ttyUSB0 --duration 5 --query
```

当前遥测只接受`speed1/target1`到`speed4/target4`及`y_body`；旧单电机数据计为无效行，不写入有效遥测记录。速度和目标是归一化值，不是mm/s；`y_body`单位cm。监测不产生电机控制指令，也不依据`y_body`自动触发视觉动作。当前主控开机四电机目标为50，且有`y_body>=50`停车试验逻辑；是否使能、何时停车由主控现有代码决定，不能从遥测工具推断已具备视觉闭环或断线停车。

## 独立运行、回放与后续协议

只有模型与清单布局匹配时才使用内置检测器。本地ONNX检查示例：

```bash
python -m rescue_vision run --config config/vision.json --model artifacts/best.onnx --manifest artifacts/model.json --show --record-video --output recordings/bench_01
python -m rescue_vision replay --config config/vision.json --model artifacts/best.onnx --manifest artifacts/model.json --video recordings/bench_01/raw.avi --output recordings/replay_01
```

默认`serial_protocol=motor_debug_ascii`，`run --serial`会在打开端口前拒绝发送二进制观测。电控以后实现视觉接收器、确认接线后，才能为对应端口使用`vision_v1`，双方使用相同波特率。改配置不能让现有固件自动支持新协议。

详见`PROTOCOL.md`和`stm32/README.md`。运行记录包含JSONL时间戳、协议流，可选原图/叠框AVI。AVI固定15fps，精确时序以JSONL为准；软件延迟不包含曝光、相机内部缓冲和STM32执行。

```bash
python -m rescue_vision report --input recordings/trial_01/observations.jsonl --output artifacts/performance.json
python -m rescue_vision trial-report --input 实车测试.csv --output artifacts/trials.json
```

接通后再验收≥15Hz有效观测、软件延迟P95≤150ms、30次实车接近≥90%满足停车距离和横向误差，以及遮挡、相机/串口断线和危险目标停车。未做实车测试时不预填通过记录。

## 板端只读信息采集

用现有检测程序的Python环境执行，替换为实际脚本、模型和设备路径：

```bash
python tools/inspect_orangepi.py --script /实际目录/detect.py --model /实际目录/best.rknn --camera /dev/video0 --output artifacts/orangepi_inspection.json
```

工具只查询版本、文件SHA256、设备节点及V4L2格式/控制项，不运行模型、不启动视频流、不发串口、不修改相机参数。它不替代真实帧率测试。

## 保留的转换工具

已有RKNN检测不必重做下列步骤，仅在重新导出、回归比较时使用：

```bash
python -m rescue_vision export --weights ../smart-rescue-sim/gazebo/weights/best.pt --output artifacts
python -m rescue_vision convert-rknn --onnx artifacts/best.onnx --manifest artifacts/model.json --output artifacts/best.rknn
python -m rescue_vision compare --reference ../smart-rescue-sim/gazebo/weights/best.pt --candidate artifacts/best.onnx --manifest artifacts/model.json --images 实拍测试图目录 --output artifacts/compare_onnx.json
```

RKNN转换需兼容Linux RKNN-Toolkit2，默认不量化；INT8另需实拍校准图。九输出、RGB归一化、补边114和CPU解码必须配套。官方依据：[瑞芯微YOLO11示例](https://github.com/airockchip/rknn_model_zoo/tree/main/examples/yolo11)、[适配检测头](https://github.com/airockchip/ultralytics_yolo11/blob/main/ultralytics/nn/modules/head.py)。
