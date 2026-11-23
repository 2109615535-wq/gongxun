# vision 使用说明（RV3/RV4）

STM32 通过 USART6 接收香橙派发来的 RV3 或 RV4 观测，115200 / 8N1。PC7 为 RX，PC6 为 TX。烧录支持 RV4 的主控后，在香橙派启动命令中指定 `--controller-protocol vision_simple_v4` 即可传输安全区抓取保护位。

`vision_init()` 启动接收，`vision_process()` 每 5 ms 解析并检查有效期。业务层分别从 `vision.upper` 和 `vision.lower` 读取观测。先检查本路 `fresh` 和目标数 `count`，再读取对应物体：

- `x_mm/y_mm` 为相对视觉标定原点的地面坐标，右正、前正。两项都有数值时 `position_present=1`。场地地图优先使用这对坐标并沿用观测时刻的位姿。
- `distance_valid` 和 `angle_valid` 表示测距及方位有效。标志为 0 的数值可作为估计值。主控直接使用视觉端已补偿的值。
- `in_roi` 为下视夹爪区域状态。入仓单独读取 `zone_kind`、`inside_valid` 和 `object_inside`。业务层仍须核对本方颜色和物块类别。
- RV4 在每个物体的 RV3 字段后追加 `in_safety_zone`，取值为 0 或 1。由视觉按单个安全区框覆盖物块框至少 80% 计算。搜索、追踪和场心扫描跳过被标记的物块。下视若发现被标记的物块位于夹爪区域，会撤销尚未闭爪的夹持请求。RV3 没有该位，主控按 0 读取。
- 累计年龄达到 150 ms 时撤销测距、方位和入仓有效位。整帧累计年龄大于 400 ms 时过期，上下视各自计算年龄。

每帧最多 12 个目标，每行最多 1024 字节（含 CRC，不含换行）。RV3/RV4 单向发送观测，`target_id` 是帧内序号。UART5 沿用现有二进制遥测格式，展示上视 X/Y 和下视入仓结果。
