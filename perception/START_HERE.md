# 交付目录与首次运行

本版保留完整Python模块、双摄配置、网页、启动脚本、模型和简易观测协议C参考接收器。新增双向任务协议、连续目标编号、阶段隔离和ROI核验结果；未部署香橙派、修改真实主控或提交GitHub。

当前离线回归源码位于tests目录，覆盖新版CRC、任务约束、序号/会话、阶段切换、目标关联和核验禁止放行。运行 `python -m unittest discover -s tests -v` 可复查，主机验证不代替串口和实车验收。

```text
perception/
├─ rescue_vision/                 完整Python模块及web网页
├─ config/                       双摄配置、标定与原配置示例
│  └─ dual_calibration/           上视/下视映射及检查报告
├─ stm32/
│  ├─ vision_simple.c
│  ├─ vision_simple.h
│  └─ SIMPLE_PROTOCOL.md
├─ tools/
│  ├─ run_dual_orangepi.sh
│  ├─ run_dual_dashboard.cmd
│  └─ run_dual_cameras.cmd
├─ models/
│  ├─ best.pt                    电脑PT模型
│  ├─ best_rknn_model/
│  │  ├─ best-rk3588.rknn         板端RKNN模型
│  │  └─ metadata.yaml           类别表与输入尺寸
│  └─ rknn_compat.py             与现有板端一致的兼容脚本副本
├─ pyproject.toml                安装元数据与依赖声明
├─ README.md                     当前能力与启动选项
├─ TASK_PROTOCOL.md              双向任务协议与实现边界
├─ tests/                        离线回归检查
└─ START_HERE.md
```

## 电脑运行

使用已经装好依赖的Python环境，在`perception/`根目录执行：

```text
python -m rescue_vision.dashboard
```

浏览器访问`http://127.0.0.1:8080`。默认只开网页，真实双摄需配置编号或执行：

```text
python -m rescue_vision.dashboard --cameras --upper-source 1 --lower-source 2
```

电脑模型已改为`config/dual_camera.json`中的`../models/best.pt`，不依赖原电脑的仿真目录。Windows脚本使用`VISION_PYTHON`指定的解释器，未设置时使用当前环境的`python`。

## 香橙派与现有环境

```bash
sh tools/run_dual_orangepi.sh
```

脚本优先使用`VISION_PYTHON`指定的解释器，默认沿用`/home/orangepi/venvs/yolo-rknn-board/bin/python`，不存在时尝试`python3`。模型改为本包的`models/best_rknn_model/`。

`config/dual_orangepi.json`的`existing_project`仍为`/home/orangepi/yolo_deploy`，用于加载板端已有`rknn_compat.py`和匹配的`runtime-2.3.2/librknnrt.so`。本包`models/rknn_compat.py`为同版备份；当前配置不直接从备份位置加载，因为那里没有复制底层运行库。更换原部署目录时须同步核对辅助脚本、运行库位置和配置，不能只移动一个脚本。

本包不包含NumPy、OpenCV、pyserial、PyYAML、Ultralytics、Torch、RKNNLite安装包或虚拟环境，也不包含`librknnrt.so`。它们沿用目标设备现有环境。`pyproject.toml`只是项目声明，不是上述环境文件；依赖已就绪时可直接从源码运行，也可选择`python -m pip install -e . --no-deps`登记项目。

相机USB路径、分辨率、ROI和实测安装姿态必须与配置一致。当前ROI仍为待重新核验，局部检查/拟合和近场外推不是完整实车运动标定通过。无地面位置时angle为NA，位置无效或年龄达到150ms后，主控angle_valid不能保持有效。

## 使用范围

当前入口为双摄`dashboard`；`config/vision.json`等其他配置保留为原流程示例，若运行旧单摄/标定命令，还需准备其各自输入数据。保留三个双摄启动脚本与当前回归测试，不附实验录像、照片、缓存、历史压缩包或旧二进制C接收器。

双向任务通信见 [TASK_PROTOCOL.md](TASK_PROTOCOL.md)，启动选项见 [README.md](README.md)。现有vision_simple.c/.h只支持RV1/RV2；启用rescue_json_v2前主控需要适配。库口姿态、入仓和投放成功接口当前保持无效，不能用于放行。
