# GitHub版视觉模块使用入口

本目录新增于2026-10-31，独立于仓库已有的`project/`主控工程；本次上传不修改任何原有仓库文件。

## 包含内容

- `rescue_vision/`：YOLO检测接入、地面位置解算、离车绿色物资测距、观测协议编解码、串口运行及记录。
- `stm32/`：独立C视觉接收和单普通物资接近停车参考模块，尚未集成到仓库的实际主控固件。
- `PROTOCOL.md`：视觉到主控的v1协议、异常语义、坐标约定、合成报文及联调基线。
- `models/`：本项目已有的PT权重、参考ONNX模型及其清单，见[模型说明](models/README.md)。
- `calibration/`：C100实测内参、离车地面映射、拟合点与独立检查报告。
- `config/`、`examples/`、`tools/`、`tests/`：配置模板、接入示例、工具和主机测试。
- `hardware/`及`HANDOFF_V2.md`、`HANDOFF_V3.md`：此前硬件核验和交接记录；其中地址、目录及主控状态是历史记录，不能代替当前设备确认。

不包含相机录像、采集照片、缓存、历史压缩包、完整仿真工程或香橙派原部署目录。文档中提到但不在本目录中的历史实验材料未随本次上传提供。

## 环境与测试

Python 3.10及以上，从克隆仓库根目录进入本目录。若已具备原项目可用的`yolo`环境，先激活该环境；首次安装可执行：

```text
cd perception
python -m pip install -e ".[desktop,test]"
python -m pytest -q
```

主机与C接收器互通测试还需`gcc`在PATH中。2026-10-31，本地源模块和准备上传的目录分别运行主机测试，均为85项通过；上传目录的命令行入口和显式路径标定加载也已核对。主机测试不需要连接相机、串口或电机。

## 电脑端YOLO与C100离车测距

以下命令从`perception/`执行，显式指定仓库内的模型、检查报告和映射，避免使用原电脑的绝对路径：

```text
python tools/detect_green_distance.py --camera 1 --weights models/best.pt --check-report calibration/20261030_180403_198681_check_report.json --mapping calibration/20261030_180026_563174_unvalidated.json
```

也可在已经激活Python环境的Windows终端运行：

```text
tools\run_green_distance_repo.cmd --camera 1
```

相机编号按实际设备调整；Windows默认后端打不开时可追加`--capture-backend dshow`。按S保存数据，按Q退出。运行输出保存在被忽略的`artifacts/green_distance_runs/`。

原`run_green_distance.cmd`保留原电脑解释器路径，作为历史入口；其他电脑使用上述显式命令或新增的`run_green_distance_repo.cmd`。

标定报告保存了原实验文件路径和哈希，用于追溯。命令中的`--mapping`覆盖旧路径，同时仍检查映射内容的SHA256，没有修改原标定文件或将其变为实车有效。

## 标定与使用边界

`calibration/c100_intrinsics_20261029.json`是本台相机的内参；地面映射对应原离车实验安装姿态和地面标记原点。换相机、改变分辨率、相机姿态或原点后，需要重新标定并独立验证，不能直接套用旧数值。

已附离车独立6点检查，横向/前向最大误差约6.6/14.0mm；这不表示任意安装、任意范围或实车测距都有同样精度。原验证前向范围150～800mm；默认0～150mm近场外推显示为`NEAR_ESTIMATE`，精度未验证。追加`--strict-range`可关闭近场估算。

离车输出为`x_right / y_forward`，协议使用底盘原点、x向前/y向左；接入主控前必须完成原点和朝向变换。附带标定及`config/calibration.json`均保持`validated=false`，电脑测距工具不发送串口或控制电机。

## 与主控对接

先阅读[协议](PROTOCOL.md)和[独立STM32模块说明](stm32/README.md)。现有v1只定义视觉到主控的观测方向，主控反馈和任务命令尚未定义。

视觉编码入口是`rescue_vision.protocol.encode()`；C接收入口是`RV_Init`、`RV_OnByte`、`RV_Step`。实际主控是否已更新接收能力、串口占用及整车速度接口，需要电控负责人核对当前工程和板内固件，不能直接用历史UART5单电机调试命令承接视觉观测。

先做纯收发、显示及断流测试，再进行完成实车标定后的低速闭环。当前目录的上传不会自动修改主控固件。

## 香橙派RKNN

`config/orangepi_existing.json`是原香橙派部署参考，依赖该设备上已有的模型目录、`metadata.yaml`、`rknn_compat.py`及配套运行库。实际板端RKNN文件不在本次上传中；这些路径、哈希和设备节点须在目标板上核对。

附带ONNX是参考九输出模型，不是原香橙派已部署RKNN的直接替代品。已有板端模型的布局和运行库须按`HANDOFF_V3.md`保持配套。
