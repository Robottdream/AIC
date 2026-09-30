# Foxglove 发令无响应（2026-09-30）

## 后续回归：地图消失与低帧率

替换官方 XML 启动方式时遗漏原文件 max_message_size=10000000 的设置，直接启动 rosbridge_websocket 的默认值只有 1000000。Foxglove 使用 CBOR-raw，当前地图约 3.94 MB，相机约 2.76 MB，超过默认上限；已安装协议层进入分片流程，尝试把二进制 CBOR 消息再次当作 JSON 序列化，报 reject_bytes 错误。地图发送失败，巨大消息内容被反复打印到日志，运行日志增长至约 7.4 GiB，launch 日志进程占用约一个 CPU 核。属于本轮桥接替换引入的回归，已纠正。

修正：显式设置 max_message_size=16000000，覆盖现有地图/相机 CBOR 消息，write_queue_size=16 限制待发送队列。没有修改系统 ROS 包的分片算法；超过新的上限或客户端主动请求更小分片仍需另行处理。仅重启桥接进程组，旧错误日志在新桥接启动时被覆盖，不保留完整二进制错误载荷。

通过 tools/check_rosbridge_visuals.py 直接使用 WebSocket CBOR-raw 订阅 15 秒，验证 /map 收到 1 条（3936420 B，静态地图无需持续发布）、/global_costmap/costmap 13 条（3936439 B，约 0.859 Hz）、/camera/image_raw 76 条（2764929 B，测试主动限速为 5 Hz，实测 4.993 Hz）。这验证传输路径，不等于读取或测量用户当前 Foxglove 面板渲染帧率。

并行 20 秒实测雷达墙钟约 9.46 Hz、仿真约 10 Hz，实时因子 0.946；发令入口与不存在节点查询复测均通过。桥接新日志只有约 24 KiB，未再复现上述分片序列化错误。证据 visuals-check.json 与 bridge-check.json 在 log/foxglove-command-20260930/。修改已同步，仿真继续运行，客户端可重新连接以获取静态地图。

现场 parser/main 日志没有新任务记录；rosapi 日志显示 get_node_details 对 proxy.get_node_info 返回 None 直接拆包，导致 TypeError 崩溃。桥接的服务调用默认在主线程执行且超时为零，因此已发出的失效服务请求有阻塞 WebSocket 消息处理的风险，不能仅凭端口 9090 开着判断发令正常。

另一个确定的配置不一致：demo1.json 的自然语言 Publish 面板仍发布到 /chat，而解析器订阅 /command；/chat 应为解析后的任务 JSON 数组。保存布局已改为 /command。现场面板当前实际话题未直接读取，不能断言用户本次必然发错话题；当前打开的布局需要手动改发布话题或重新导入。

新增 tools/rosapi_safe.py 包装已安装 rosapi，仅对查询不存在的节点返回三个空列表，保留 include_hidden 参数，不修改 /opt/ros。tools/rosbridge_safe.launch.py 启动桥接及包装节点，服务请求用工作线程执行、5 秒超时，动作请求同样在工作线程，进程异常后重启。启动脚本改用此 launch，已同步到运行副本。

仅重启桥接进程组，保留仿真、解析器、主控和导航。新进程组已更新到启动脚本的 log/run/pids 中。

tools/check_rosbridge_command.py 通过真实 ws://127.0.0.1:9090 查询不存在节点，并在 /command 发布 __foxglove_probe__。测试收到空列表成功响应及 ROS 话题回传；解析器日志也独立记录该标记（无有效颜色/区域，按设计拒绝）。没有执行测试抓取任务。证据：log/foxglove-command-20260930/bridge-check.json。该检查验证 WebSocket → ROS → parser 入口，未验证用户当前面板或完整抓放流程。

发令设置：话题 /command，类型 std_msgs/String（ROS 2 完整名称 std_msgs/msg/String），消息例如 {"data":"抓取1个蓝色去B"}。重新连接 localhost:9090 后重新发送；原未送达的任务不自动补发，以免重复执行用户任务。
