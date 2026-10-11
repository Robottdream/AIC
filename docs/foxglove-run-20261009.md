# 2026-10-09 当前办公室地图接入 Foxglove

本机 Foxglove 桌面客户端已运行，用户确认已连接并看到话题列表。
使用 Rosbridge 协议连接 `ws://127.0.0.1:8765`。
参考 [Foxglove 实时连接说明](https://docs.foxglove.dev/docs/visualization/connecting/live)。
9090 当前由另一程序使用，因此本次显式设置 `AIC_ROSBRIDGE_PORT=8765`；
未设置变量时桥接仍保留原 9090 默认值。桥接模型资源 URL 使用同一端口配置。

启动当前斜向障碍、1 × 0.5 m 放置区场景：

```bash
PYTHONNOUSERSITE=1 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 \
AIC_ROSBRIDGE_PORT=8765 \
LLAMA_BIN="$PWD/runtime/llama.cpp/build/bin/llama-server" \
MODEL="$PWD/runtime/models/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf" \
bash test_map.sh short_routes_20261007 start
```

13 包构建完成；Gazebo、控制器、Nav2、模型服务、指令解析、导航、机械臂、检测、
主控和桥接均启动。`start` 等待人工指令，不会自动执行五件任务。
原始运行日志保存在 `/tmp/aic-map-short_routes_20261007/log/run/`。

Foxglove 发布面板选择 `/command`，消息类型 `std_msgs/msg/String`，例如：

```json
{"data":"将三个红色物块放到A区，再将两个蓝色物块放到B区"}
```

解析结果显示在 `/chat`。3D 面板参考坐标系选 `map`，开启 `/map`、`/scan`、
`/tf`、`/robot_description` 和导航路径主题。检测图像使用 `/detector/annotated_image`。

真实 WebSocket 的 CBOR-raw 只读验证通过，5 秒内收到：地图 1 条、雷达 5 条、
机器人描述 1 条、坐标变换 5 条、检测图像 4 条。雷达真实射线和强度数组均 1441 项，
机器人描述可解析为 21 个 link，当前模型使用基础几何体。
低于传感器发布频率的计数是主动设置 1 秒订阅节流，并非传感器发布频率。
普通 JSON 测试订阅与已连接客户端的 CBOR-raw 共享订阅出现类型转换报错；
按 Foxglove 所用的 CBOR-raw 路径复核数据均正常。

自动审批拒绝 `/command` 的无效连接探测消息，理由是可能触发控制解析。
此次没有通过工具发布任务或探测控制消息；任务仍等待用户在 Foxglove 中发令。
本次验证连接和传输，不代表已从 Foxglove 完成五件搬运回归，也未自动操作本地面板。

停止当前栈：

```bash
bash test_map.sh short_routes_20261007 stop
```
