# USB Foxglove 无 Gazebo 窗口启动（2026-10-09）

## 原因与修改

U 盘部署入口 `usb_start.sh` 原先执行 `test_map.sh SCENE "$AIC_USB_ACTION"`，
只转发动作名，丢弃后续 `--headless`。底层 `project_launcher.py` 已正确处理
该选项并传入 `gui:=false`；Gazebo launch 用条件控制 `gzclient`。

修改 USB 入口为取出动作名后完整转发 `"$@"`，并支持省略 `start`：

```bash
cd /home/ros/AIC_USB_robottdream
bash usb_start.sh start --headless
# 已启动时，在任务结束后重新启动仿真：
bash usb_start.sh restart --headless
# 省略动作也支持：
bash usb_start.sh --headless
```

桌面源码保留同一份 `usb_start.sh` 和其环境检查伴随脚本 `usb_check.sh`，便于以后打包。
已同步到挂载的 USB `/home/ros/AIC_USB_robottdream`；同步范围为入口脚本、部署说明、
清单和校验和。未改机器人、地图、导航或抓放参数。

只关闭 Gazebo GUI 客户端，后台 `gzserver` 继续进行物理与传感器仿真。
ROS 传感器相机仍需可用的图形渲染环境；这里不删除 DISPLAY，也不关闭桌面显示服务。
Foxglove 独立打开并选择 Rosbridge，地址 `ws://127.0.0.1:8765`。
`start` 继续等待人工发令，不自动发送搬运任务。

## 验证与限制

- `bash -n` 通过。
- 7 个参数转发/非法动作检查通过，覆盖显式 start/restart、仅 --headless、默认启动、日志模块和停止。
- `bash usb_start.sh build` 从 src 构建 13 个包成功。
- 当前内置磁盘 Ubuntu 的隔离 ROS domain 141 / Gazebo master 11347 上进行无窗口传感器验证。
- Gazebo 没有启动 `gzclient`；收到 1441 射线 LaserScan、Clock、JointState 及有效 RGB/深度图像。
- RGB 1280×720，深度 640×480；传感器接收记录如下。
- 验证后停止此隔离仿真，没有执行搬运任务，也未在 U 盘启动后的系统验证。
- USB SHA256SUMS 的 214 项校验全部通过。

```json
{
  "scan": 33,
  "clock": 18,
  "joint_states": 88,
  "images": {
    "/camera_rgbd/camera_rgbd/depth/image_raw": {
      "count": 3,
      "width": 640,
      "height": 480,
      "encoding": "32FC1",
      "bytes": 1228800
    },
    "/camera/image_raw": {
      "count": 5,
      "width": 1280,
      "height": 720,
      "encoding": "rgb8",
      "bytes": 2764800
    },
    "/camera_rgbd/camera_rgbd/image_raw": {
      "count": 2,
      "width": 640,
      "height": 480,
      "encoding": "rgb8",
      "bytes": 921600
    }
  },
  "gzclient_started": false,
  "scan_rays": 1441,
  "passed": true
}
```
