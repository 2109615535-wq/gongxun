# 安全区坐标与视觉入仓部署记录

- 日期：2026-11-16（北京时间）。
- 状态：已部署香橙派并验证服务；按用户选择，主控稍后烧录。
- 设备：`orangepi@10.163.0.250`。
- 监看页面：[双摄视觉监看](http://10.163.0.250:8080)。
- 程序目录：`/home/orangepi/rescue_vision_dual`。
- 服务：`rescue-vision.service`，重启后 `active/running`，主进程 PID 为 4205。
- 部署前备份：`/home/orangepi/rescue_vision_dual/backups/zone_entry_20261116_204503_3980`。

## 本次行为

红蓝双方的物资安全区、伤员安全区均使用完整检测 ROI 的中心作为参考点，经去畸变、地面投影和已有二次测距补偿后，提供 X、Y、径向距离和地面方位角。物块继续使用原来的贴地点。角度是目标相对小车的方位，不是安全区自身朝向。

按本轮确认的口径，物体整个检测 ROI 位于对应安全区 ROI 内，且同一物体、同一区域连续 3 帧满足时，报告视觉入仓。普通/核心物资对应物资区，伤员对应伤员区；返回实际区域颜色和类别，供主控核对本方。越界、遮挡、区域歧义、目标丢失、任务切换或结果过期时撤销确认。该结果不代表夹爪已经释放或退出后物体仍留在区内。

网页及 `/api/status` 提供 `position_mm`、方位和 `zone_entry`。网页保留板端原来的估算位置显示，并为各类目标增加距离、方位和入仓状态。

## 通信与主控

当前实际服务保持 `vision_simple_v2`，串口连接正常，继续向现有主控返回距离、方位角等旧字段。RV2 不携带新增的直接 X/Y 和入仓字段。

已加入可选的 `vision_simple_v3`，每个目标十个字段：

```text
kind,distance_mm,distance_valid,in_roi,angle,x_mm,y_mm,zone_kind,inside_valid,object_inside
```

本机未提交的新版主控接收器草稿兼容 RV1/RV2/RV3，未纳入此次视觉提交。主控整理适配并烧录后，再将视觉服务启动参数改为 `--controller-protocol vision_simple_v3`。本次未切换线上协议，也未烧录主控。

主控代码通过 `VisionObject.x_mm/y_mm/position_present/angle/angle_present/distance_mm/distance_valid/zone_kind/inside_valid/object_inside` 读取结果。入仓有效位独立于地面测距有效位，达到 150 ms 时撤销。完整字段见[简易协议说明](../perception/stm32/SIMPLE_PROTOCOL.md)。

## 部署与验证

部署前对板端文件进行了比对，合并保留了板端已有的绿色估算显示逻辑。仅更新 6 个运行文件和 4 个测试文件；模型、相机参数、两份配置、当前夹爪 ROI `[158,49,486,452]` 及启动脚本均保留。先在备份目录建立隔离副本并通过测试，再停止服务、原子替换文件并启动服务。

- 电脑端 Python 回归：57 项通过。
- 香橙派隔离副本回归：57 项通过。
- 香橙派部署后生产目录回归：57 项通过。
- 已部署文件 SHA-256 与更新清单一致。
- RKNN 模型状态：`READY`。
- 上下视：均为 `OK`，两次快照的帧号持续增长，处理速度约 15 Hz。
- 串口：`CONNECTED`，协议为 `vision_simple_v2`。
- 实际主控 C 接收器：RV3 坐标、方位、区域、入仓字段、过期撤销及 RV2 兼容检查通过。
- 本机主控新固件已编译，产物为 `project/build/safety_zone/project.hex`，未纳入此次视觉提交。完整固件回归仍有原转向 PID 参数断言不匹配，详见[阶段记录](../process/第01阶段_50cm内绿色物块夹取与放回物资安全区/01_该阶段目标/阶段目标.md)。

部署验证时，画面未提供完整的目标物体与安全区组合，因此尚未完成实景入仓正例验证。50 cm 内夹取、放回及 30 s 用时仍待整车验收。

本次为连接设备临时添加的 TUN 单地址排除项已恢复，未改变原代理模式。机器可读记录见[部署报告 JSON](vision_zone_entry_deployment_2026-11-16.json)。

## 本地复查

在 `perception/` 执行：

```powershell
python -m unittest discover -s tests -v
```

以下为本机未提交的主控配套测试及构建工具命令，不属于此次视觉提交；仅拉取视觉提交时不提供这些新选项和工具。主控适配要求见[接入清单](../perception/MCU_RV3_HANDOFF.md)。整理主控改动后，可在本机仓库根目录执行：

```powershell
python tests/vision/run_tests.py --receiver-only
python tools/build_firmware.py
```

主控固件输出位于 `project/build/safety_zone/`；构建器复用现有 Keil 工程的源文件清单、已安装的 CubeIDE GCC，以及 STM32F407VETx 的 512 KiB Flash、128 KiB SRAM 配置，不执行烧录。
