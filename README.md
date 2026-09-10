# RealSense 多相机

网页“相机配置”按列表逐台添加、移除设备，数量由实际配置决定（当前最多 16 台）。
每台填写唯一 ID、型号（例如 d405、d435、d455）和序列号；同时启用多台时序列号不可缺失或重复。
序列号以字符串传入驱动，保留前导零。每台可设置独立命名空间、节点名，以及彩色、深度、RGB-D、
元数据和点云话题；话题留空时保存为该命名空间下的默认值，修改已有默认值后也可以清空重新生成。

保存配置并启动相机后，在该相机下点击“测试拍照”，页面读取其彩色话题的一帧新鲜图像并显示，
附带相机 ID、配置序列号、图像尺寸和来源话题。每台分别测试；超时或图像无效时显示失败原因。
这是实时视频取帧预览，不触发硬件快门、不启动录制；图片只返回浏览器，服务端不保存照片文件。
拍照可以独立于机械臂和夹爪测试，ROS 连接与对应相机节点需要已经启动。

例如四台设备可分别使用 front、left、right、rear，配不同真实序列号和 /front/rgb、/left/rgb 等话题。
这只是命名示例，并不固定相机数量；型号和序列号选择由官方驱动执行。
参考[官方设备选择实现](https://github.com/IntelRealSense/realsense-ros/blob/ros2-master/realsense2_camera/src/realsense_node_factory.cpp)。

本包使用官方 `realsense2_camera_node` 采集，并支持每个机器人配置任意数量和型号的 RealSense。网页逐台保存型号与唯一序列号，并统一配置曝光、增益补偿、RGB-D 成组同步和曝光中点时间戳对齐。D405 使用共享 `depth_module`，D435 等具有独立彩色传感器的型号使用 `depth_module` 与 `rgb_camera`。没有开流后切换、参数回读或逐帧曝光检查。

独立的 `timestamp_adapter.py` 只把官方 metadata 中的曝光中点转换到 ROS 时间，并为点云保留来源深度/RGB 关联。它不调用相机 SDK，也不修改官方驱动。`humanoid_manager` 只保存机器人相机清单并生成 launch 输入，相机采集仍在独立节点中执行。

```bash
ROS_DOMAIN_ID=14 ./src/humanoid_camera/start_camera.sh
```

或在已配置 ROS 环境时：

```bash
ros2 launch humanoid_camera d405.launch.py namespace:=front
```

单台 D435：

```bash
ros2 launch humanoid_camera d435.launch.py namespace:=right serial_no:=你的序列号
```

三台相机（先替换示例文件中的三个序列号）：

```bash
./src/humanoid_camera/start_cameras.sh ./src/humanoid_camera/config/three-camera.example.yaml
```

`multi_camera.launch.py` 接受管理器部署在机器人内部配置目录中的 `cameras.yaml`。同时启用多台 RealSense 时每台必须填写唯一序列号。型号字段不会限制为 D405/D435；其他 RealSense 型号使用官方驱动的通用参数映射，设备不支持的能力需要在真机启动时处理。D435 系列彩色流采用不超过 5 ms 的手动曝光，默认 4500 μs、增益 64。其原生 RGB 自动曝光无法设置这个上限，因此网页禁用该开关，加载旧版本时也关闭 RGB 自动曝光；深度自动曝光保持独立设置。没有新增软件自动曝光或图像提亮处理。

页面和 `cameras.yaml` 的 `color_exposure_us` 使用微秒。D435 系列 RGB 的官方驱动参数 `rgb_camera.exposure` 使用 100 μs 单位，因此 4500 μs 转换为 `45`，5000 μs 转换为 `50`；不能把页面微秒值直接填入高级官方驱动参数。非整百微秒值向下取整。深度模块的曝光参数仍使用微秒。

D435 的 RGB 曝光模式、曝光值和增益以上述专用字段为准，保存时移除对应的高级参数覆盖值。此修正仅在启动时下发相机参数，保留连续流的分辨率、配置帧率、QoS 和时间戳处理，不对已采集图像做亮度后处理。实际曝光和出帧情况仍需在真机验收中检查。

单相机可用参数：`serial_no`、`params_file`、`camera_name`、`namespace`、`normalize_timestamps`。管理器的 ROS domain_id 必须与相机一致。

[时间戳源码核查、转换公式与输出话题](docs/d405-humble-implementation.md)。

[手动验收脚本与结果说明](docs/manual_acceptance.md)：按需运行 `ros2 run humanoid_camera check_cameras.py`，
检查实际曝光、自动补偿响应、RGB/深度中点差和时间戳换算，输出终端、HTML、JSON 和 CSV 报告。
加 `--require-equal-exposure` 后，另要求每对 RGB/深度的实际曝光时长差为 0 μs；
曝光时长相等与曝光中点对齐分别验收。D435 若要求相同曝光设定，需将深度也改为手动曝光并与 RGB 设置一致，
保存、重启后再以实际帧元数据验证。验收脚本只读取数据，不改变相机参数和连续采集链路。

通用 `multi_camera.launch.py` 现在执行设备配置的 `startup`，其他品牌也随整机启动。
例如 `backend: vendor_camera` 可同时声明 `instance_parameters`、ROS launch 步骤、
`rgbd_topic` 与 `metadata_topic`；变量写作 `${name}`。不声明 startup 时连接外部已有话题。
RealSense 兼容配置独立在 `realsense_camera.launch.py` 和 `realsense_profile.py` 中；
使用该 profile 时请安装 `realsense2_camera` 和 `realsense2_camera_msgs`，通用相机包不再强制依赖它们。
