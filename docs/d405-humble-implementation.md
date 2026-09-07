# D405 + Humble：官方驱动与曝光中点转换

当前实现按用户最后确认的要求：官方驱动用 YAML 直接启用自动曝光；不先设手动值、不在开流后切模式、不回读曝光/增益限制、不做逐帧曝光/增益验收。独立转接节点只处理时间戳和消息来源。

## 官方 Header 的源码核查

核查了 realsense-ros 4.55.1 与 4.58.3。当前本机实际启动日志为 ROS 驱动 4.58.3、librealsense 2.58.3。设备尚未在系统中出现。

- `frame_callback()` 使用 `frameSystemTimeSec(frame)`；输入来自 SDK `frame.get_timestamp()`。
- HARDWARE_CLOCK 路径：以初始 `_node.now()` 为基准，加设备帧时间的增量；4.58.3 增加硬件时间倒退后的重建基准。
- GLOBAL_TIME 路径：SDK 把设备帧时间映射到主机系统时间，ROS 驱动把毫秒换成纳秒。
- 同一 frameset 的 RGB、深度与点云发布沿用同一个 `t`；这个相同 Header 用于关联，并没有分别读取 RGB/depth 的 `sensor_timestamp` 后映射曝光中点。
- metadata 中 `frame_timestamp` 是 SDK 帧时间，单位 ms；`hw_timestamp` 是原始 UVC 帧时间，单位 μs；`sensor_timestamp` 是曝光中点，单位 μs。不能把这三个字段当同一个值。

依据：[官方 ROS 4.58.3 源码](https://github.com/realsenseai/realsense-ros/blob/4.58.3/realsense2_camera/src/base_realsense_node.cpp)、[SDK 时间戳读取](https://github.com/realsenseai/librealsense/blob/v2.58.3/src/ds/ds-timestamp.cpp)、[SDK 全局时钟映射](https://github.com/realsenseai/librealsense/blob/v2.58.3/src/global_timestamp_reader.cpp)、[SDK 字段语义](https://github.com/realsenseai/librealsense/blob/v2.58.3/include/librealsense2/h/rs_frame.h)。已核查 SDK 2.58.3 的 UVC 时间戳读取与全局时钟映射代码；设备实际输出仍需在有设备时核实。

## 转换方式

转接节点不打开相机，只订阅官方图像、metadata、CameraInfo 和 PointCloud2。先按原始驱动 Header 关联消息，再分别为 RGB/depth 计算：

```text
global_anchor_ns = metadata.frame_timestamp_ms × 1,000,000
local_delta_us = sensor_timestamp_us − hw_timestamp_us
capture_ros_ns = global_anchor_ns + local_delta_us × 1,000
```

原始 UVC 计数为 uint32 μs，差值按有符号局部差处理回绕。`sensor_timestamp` 已经是中点，不再加减半个曝光时间。每帧保留原始 JSON、原 Header、设备帧号以及该帧映射锚点。

这是使用 SDK GLOBAL_TIME 锚点的局部偏移模型，scale=1；其剩余时钟误差没有实测，因此 `uncertainty_ns=null`，不填写伪造的零误差，也不作为物理 1 ms 精度证明。若缺少必要 metadata 或仍不是 GLOBAL_TIME，节点报告无法转换，并保留官方原始话题，不用 `now()` 冒充曝光中点。ROS 默认系统时间域适用于此实现，不支持 `/clock` 仿真时间。

点云使用原始 frameset Header 找到来源配对，再把 Header 改为来源深度的映射中点；保存 source_depth_seq、source_rgb_seq 和 pair_seq。点云接收时间单独保存，官方消息没有计算完成时间，因此该字段为 null。

## 启动与话题

```bash
ROS_DOMAIN_ID=14 ./src/humanoid_camera/start_camera.sh
```

`start_camera.sh` 在系统未提供驱动时可加载已准备在 `~/.local/share/humanoid-camera/ros-overlay` 的官方二进制包；它不修改 `/opt/ros`。同一机器上已安装官方驱动时也可直接使用：

```bash
ros2 launch humanoid_camera d405.launch.py
```

- 官方输入：`/front/camera/color/image_raw`、`/front/camera/depth/image_rect_raw`、各自 metadata 和 camera_info、`/front/camera/depth/color/points`。
- 转接输出：`/front/normalized/rgbd`（现成的 realsense2_camera_msgs/RGBD）和 `/front/normalized/metadata`。
- 点云输出：`/front/normalized/points`、`/front/normalized/points_metadata`。
- `normalize_timestamps:=false` 仅启动官方相机驱动。

相机参数只放在本包 YAML。录制器 JSON 配置选择来源和对齐参数，不作为相机启动参数传入。
