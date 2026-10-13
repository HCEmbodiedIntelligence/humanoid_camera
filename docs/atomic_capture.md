# 原子相机采集

管理器的 RealSense 录制链路使用单条消息传输每帧图像与自己的元数据：

- 驱动 `/{namespace}/{camera_name}/color/image_metadata`、`depth/image_metadata`：`ImageMetadata` 包含 Image、CameraInfo 和同一 SDK frame 的原始 JSON 元数据。
- 时间戳节点 `/{namespace}/normalized/capture`：`RGBDMetadata` 包含 RGBD 与完整归一化时间/曝光/标定信息。D435 的 RGB-only 模式不等待、不伪造深度。
- 管理器配置 `transport: ros_rgbd`、`packet_topic` 后直接消费完整消息，不订阅两个独立输出话题进行配对。旧配置仍可使用原有话题。
- `originals.mcap` 独立保存驱动原子消息的 CDR 字节和完整 ROS schema。RGB 和深度尚未配成一组也保存，编码压缩不改变原始消息。只改变传输封装，不修改实际像素、原始 Header 或 SDK 元数据；深度仍使用驱动原有的毫米单位转换。

RGB 与深度的帧组关联仍使用官方驱动的 frameset Header；真实曝光中点由 SDK GLOBAL_TIME 和 SENSOR_TIMESTAMP/HW_TIMESTAMP 计算。图像与关节状态仍按时间对齐，不能用发布先后证明物理同步。SDK 未提供的字段保持缺失并触发质量检查，不用接收时间伪造曝光时间。USB/驱动未交付的帧与下游整体消息丢失仍可能存在；本改动消除的是图像与独立元数据的传输分离故障。

## 构建

基于官方 realsense-ros 4.57.7，固定提交 `5c2244ca5cd9867c9ee63769668891430f460dfd`。安装 librealsense2 >= 2.57.7、ROS Humble 及原驱动构建依赖后，在工作区运行：

```bash
git clone --depth 1 --branch 4.57.7 https://github.com/realsenseai/realsense-ros.git src/realsense-ros
git -C src/realsense-ros rev-parse HEAD
git -C src/realsense-ros apply --check ../humanoid_camera/patches/realsense-4.57.7-atomic-capture.patch
git -C src/realsense-ros apply ../humanoid_camera/patches/realsense-4.57.7-atomic-capture.patch
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=3 colcon build --packages-select realsense2_camera_msgs realsense2_camera humanoid_camera humanoid_manager --allow-overriding realsense2_camera_msgs realsense2_camera --executor sequential --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

已有补丁的工作区不重复 clone/apply。系统 `/opt/ros/humble` 驱动不修改，使用工作区 overlay。托管启动会检查补丁标记，避免意外启动无原子话题的系统驱动。

通用驱动的 Image、Metadata 和 RGBD 消息类型不变；新增 ImageMetadata/RGBDMetadata。传统话题在有人订阅时仍发布，但本录制链路不再依赖逐帧的独立 metadata 话题。

## 验证

安装 `humanoid_manager/requirements-test.txt` 中的附加测试库，source overlay 后运行管理器的 `test/test_atomic_recording.py`。测试在独立 ROS 域中只发布原子消息，不发布独立 Metadata；覆盖 RGBD、RGB-only、故意缺失部分深度、原始文件完整性和通用 MCAP 解码。测试不会连接硬件。

参考 HC_x1 提交 `d008790a1f4ad5df6e9ca9f2c5f50079d0cd13aa`：D405 节点从同一 SDK frameset 取 RGB/深度，使用主机 `now()` 同时标记后分别发布；这不等同于逐帧曝光中点或图像与关节数据同消息。本实现保留真实 SDK 时间证据，避免引入独立元数据传输依赖。
