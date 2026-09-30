# Foxglove 底盘与轮子缺失

## 原因与修复

当前 `/robot_description` 只有 robot_state_publisher 一个发布者，完整模型中有 22 个 link。机械臂使用基础几何体，底盘和轮子使用 `package://mybot_description/meshes/...stl`。现有 rosbridge 连接没有 Foxglove Bridge 的资源获取通道，因此基础几何体可以显示，网格无法读取。资源解析规则见 [Foxglove 官方说明](https://docs.foxglove.dev/docs/visualization/panels/3d)。

新增 `tools/rosbridge_with_assets.py`，由现有 safe launch 启动：在同一个 9090 端口提供 `/assets/mybot_description/meshes/` HTTP 资源，加入跨域读取响应头；只允许该包中白名单网格/纹理文件。支持 colcon 的单文件符号链接安装。

在桥接出站回调中，将 URDF 的网格 URI 转为本机 HTTP 地址，兼容 Foxglove 使用的 CBOR-raw 序列化。ROS 内部的 robot_description、Gazebo、机器人几何与 TF 均未改动。现有 Foxglove 布局仍订阅原话题，无需导入新布局。该地址针对本机 Windows Foxglove；跨机器访问需改为客户端可访问的主机地址。

本轮只重启 rosbridge 进程组，仿真和任务节点继续运行。若客户端缓存旧 URDF，可刷新页面。

## 验证

### 网格朝向

资源加载后用户截图显示底盘竖起、轮子偏转。STL 按 ROS Z-up 坐标导出，Foxglove 的 Mesh up axis 默认设置与之不一致。项目 `demo1.json` 的 3D scene 已显式设置 `meshUpAxis: z_up`。当前已打开的布局需在 3D 面板设置 → Scene → Mesh up axis 选择 Z-up，随后刷新页面；修改磁盘布局不会自动更新已打开的布局。不旋转源网格或 URDF，避免破坏 Gazebo/RViz 中正确的几何关系。

- `tools/check_robot_assets.py`：真实 WebSocket 收到 22 link 的 URDF；15 个视觉网格全部 HTTP 200，二进制 STL 长度与三角面数量一致，总计 48,264 个三角面。
- Windows 侧直接请求底盘 STL：HTTP 200，21,484 B，跨域头为 `*`。
- `tools/check_rosbridge_visuals.py`：地图约 3.94 MB、相机约 2.76 MB 均成功收到；相机实测接收约 5.01 Hz。
- 未直接检查 Foxglove 渲染后的画面，以上为传输与资产内容验证。
