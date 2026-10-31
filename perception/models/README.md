# 模型文件

本目录保存已有模型原文件，上传时核对SHA256，不重新训练或转换。

- `best.pt`：当前电脑端YOLO测距使用的Ultralytics PT权重，约5.46MB。SHA256：`ab67fffaad0bbdac0f915941a328a310d55bc5a45fd5bc60f4815bbd33146d89`。
- `best.onnx`：此前导出的参考九输出ONNX，约10.41MB。SHA256：`555a69184e5982c1d4acdb8d84a36f49ebffa9b4b83828012db51ddbe5ff79dd`。
- `model.json`：ONNX输入、输出布局、类别表、导出版本和模型哈希清单。参考ONNX后端应与此清单配套。

模型包括业务目标以外的辅助类别；视觉协议必须按名称映射普通、核心、伤员、危险，不能直接把模型原始类别编号当作协议类别编号。

本目录不含香橙派设备上的`best-rk3588.rknn`。原板端RKNN与此ONNX的输出布局不同，不能混用解码方式。电脑端使用方法见[启动说明](../START_HERE.md)。
