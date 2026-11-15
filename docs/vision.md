# vision 使用说明（当前 RV2 版）

当前主控适配 `codex/uart6-vision-oled` 分支的 `vision_simple_v2`，USART6：PC6 TX / PC7 RX，115200 / 8N1。具体上板步骤见 [vision_static_test.md](vision_static_test.md)。

`vision_init()` 在初始化时启动接收；`vision_process()` 每 5 ms 解析并更新有效期，HAL 接收/错误回调继续通过原工程分发。`vision.upper` / `vision.lower` 为最新两路观测。`fresh` 表示正常未过期帧，`count` 是当前目标数，目标的 `distance_valid`、`angle_valid` 才表示测距是否有效；`angle_present` 用于显示估计，不能替代有效性。

RV2 是视觉主动持续发送的观测协议，不需要也不支持发送 task1。`vision_set_task()` 返回 false，`vision_set_yaw()` 不发送数据。`acknowledged`、`vision_ready` 保持 0，不能作为 RV2 收到数据的条件。`target_id` 暂作帧内序号，不是跟踪 ID。后续 case 自行依据观测和有效性选择下一步。

`vision_simple.c/.h` 来自 GitHub 分支提交 `ab6c6fe`，本地接收器已修正年龄累计并将观测超时调整为严格大于 400 ms；`vision.c` 负责本地 USART6 队列和对现有遥测结构的转换。旧 JSON 实现保存在 outputs/vision_rv2_backup_20261114，当前不参与编译。
