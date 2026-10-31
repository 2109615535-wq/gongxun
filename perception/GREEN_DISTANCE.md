# C100绿色物资与距离检测

## 直接运行

1. 保持相机、胶带原点和白色背景与2026-10-30 17:58标定照片一致。
2. 关闭hello10.py、c100_view_check.py等占用相机的程序。
3. 双击 tools/run_green_distance.cmd；默认相机编号1、640×480、YOLO模型。
4. 画面出现绿色框，红色十字为相机这一侧的可见贴地边缘中点。
5. 按S保存当前原图、标注图和数据，按Q退出。

PyCharm也可以运行 tools/detect_green_distance.py。解释器请选择
C:\Users\LENOVO\miniconda3\envs\yolo\python.exe。
之前的micu环境没有torch和ultralytics，不需要为此安装或升级，直接选择已有yolo环境即可。

## 数值含义

- X right：横向偏移，向右为正、向左为负，单位毫米。
- Y forward：贴地点相对胶带横轴的前向距离，单位毫米。
- Ground range：贴地点到胶带原点的地面直线距离，等于sqrt(X²+Y²)。
- 数值对应相机这一侧的可见底边，不是物资中心；也不是相机光轴深度。
- 当前只进行了离车实验；没有实际车头和相机安装关系，所以不能称为距真实车头距离。
- 原标定区域前向150～800mm，区域内显示正常坐标。
- 默认增加前向0～150mm的近处估算，横向限制在原实测左右范围（当前±200mm）；
  显示橙色NEAR_ESTIMATE，表示地面映射外推，精度尚未验证。
- 800mm之外、横向超出实测范围，以及底边被画面截断时不输出坐标。
- 若要保证更近处的精度，需增加50、100mm等近处实测点重新标定，并用其他点独立检查。

## 状态

实时模式连续3帧关联并有有效贴地点后显示距离；没有目标、贴边、相互遮挡、
颜色提取不到贴地点或超出允许范围时清空距离。相机故障退出并保存fault.json。
只保留最新待处理帧；采集读取返回到推理结束超过300ms时标为STALE，清空位置。
这里的时间不是曝光到控制执行的全链路延迟，也不代表已达到板端15Hz目标。

## 结果保存

结果目录：artifacts/green_distance_runs/每次运行的时间戳/

- session.json：所用模型、类别表、标定和检查报告的校验值。
- observations.jsonl：实时逐帧观测、位置、状态和时间。
- 按S保存的raw.png、annotated.png、result.json。
- 离线照片回放直接保存标注图和结果，不要求连续三帧。

## 常用参数

在PyCharm的“脚本参数”填写即可：

    --camera 1

离线核对现有照片，传原始图，不传去畸变图：

    --image "E:\智能救援\perception\artifacts\c100_captures\20261030_180648_329900_640x480_raw.png" --no-gui

纯颜色调试（可以在micu环境运行，不属于YOLO识别）：

    --detector color

Windows相机默认后端无法打开时：

    --capture-backend dshow

限时无界面联调：

    --duration 10 --no-gui

关闭近处估算，恢复只输出标定区域内坐标：

    --strict-range

限制只估算前向50mm以上的近处：

    --near-min-mm 50

近处不显示坐标时，应先看状态。OUTSIDE_CALIBRATION表示超范围，BORDER表示检测框贴边，
CONTACT_INVALID表示绿色轮廓没有提供可靠贴地点；降低YOLO置信度不能解决这些情况。

默认使用原训练PT权重，SHA256已核对与板端模型所用原权重一致。
这是电脑PT版本；香橙派RKNN后端仍使用原交接包程序，本脚本没有替换板端环境。

## 标定与边界

默认地面映射由独立检查报告
artifacts/ground_experiment/20261030_180403_198681_check_report.json绑定，
检查报告与映射的SHA256必须匹配。可以用--check-report、--mapping指定重新标定后文件。
相机姿态、地面原点、焦距或采集模式变化后，需要重新进行相应标定和检查。

不修改config/calibration.json的validated状态，不生成有效运动指令，
不连接串口或控制电机。黑色胶带可能被YOLO识别为核心物资，本工具只选择
normal_material且通过绿色贴地点筛选的物资作为测距对象。
