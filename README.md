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

本包使用官方 `realsense2_camera_node` 采集，并支持每个机器人配置任意数量和型号的 RealSense。网页逐台保存型号与唯一序列号，并统一配置曝光、增益补偿、RGB-D 成组同步和曝光中点时间戳对齐。D405 使用共享 `depth_module`，D435 等具有独立彩色传感器的型号使用 `depth_module` 与 `rgb_camera`。D435 的可选程序自动增益节点通过官方驱动参数服务调节增益。

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

`multi_camera.launch.py` 接受管理器部署在机器人内部配置目录中的 `cameras.yaml`。同时启用多台 RealSense 时每台必须填写唯一序列号。型号字段不会限制为 D405/D435；设备不支持的能力需要在真机启动时处理。

D435 系列深度和 RGB 均关闭原生自动曝光，默认使用同一帧率（30 FPS）和 3900 μs 手动曝光，起始增益分别为 64。加载仍启用深度自动曝光的旧 D435 配置时，会迁移到双路 3900 μs 手动曝光预设。已有双路手动曝光配置的明确曝光值予以保留，网页可继续调整。两路曝光、增益和帧率以专用字段为准，高级参数不能覆盖。

页面和 `cameras.yaml` 的曝光字段均使用微秒：D435 RGB 的 `color_exposure_us: 3900` 下发为官方 `rgb_camera.exposure: 39`（100 μs 单位），深度 `depth_exposure_us: 3900` 下发为 `depth_module.exposure: 3900`。RGB 非整百微秒值向下取整，配置仍要求不超过 5000 μs。

D435 USB RGB 的逐帧 `actual_exposure` 元数据也保留 100 μs 单位，验收时将原始值 `39` 换算为 `3900 μs`；深度和 D405 使用 μs。报告同时保留原始值与换算系数。此修正仅用于曝光时长统计，不改变时间戳或放宽同步阈值。依据见[验收说明](docs/manual_acceptance.md)。

D435 默认启用 `depth_auto_gain` 与 `color_auto_gain`，由 `camera_gain_controller` 分别读取红外图像亮度及 RGB 灰度亮度，通过官方参数服务只调整相机后续采集的增益。增益范围取驱动描述与 `depth_auto_gain_limit` / `color_auto_gain_limit` 的交集，新配置上限默认 128；旧配置中明确保存的上限保留。关闭对应开关后使用手动增益字段。D405 继续使用共享模块原生自动曝光及自动增益限制。

反馈目标为 8 位灰度 128，死区 ±10；每次最多移动四个硬件增益步长，异步读取参数后再调整，调节后等待新图像。图像过期、服务未就绪、参数失败或曝光模式改变时暂停调节并报告状态。曝光值保持手动设定；达到增益上限后仍可能偏暗。状态话题为 `/<namespace>/gain_control/status`。

深度自动增益需要额外启用 `infra1` 灰度流；反馈计算最多采样 64×64 个像素，最高每秒处理 10 次。原有 RGB/深度分辨率、帧率与 QoS 配置继续使用，红外传输、空间滤波与增益节点增加的开销需要在真机连续采集验收中检查。图像像素不做后期亮度增强，真实曝光中点继续保留。

单相机可用参数：`serial_no`、`params_file`、`camera_name`、`namespace`、`normalize_timestamps`。管理器的 ROS domain_id 必须与相机一致。
独立 `d435.launch.py` 用于手动曝光、手动增益测试；需要程序亮度反馈时，使用管理器或 `realsense_camera.launch.py camera_config:=...`，从机器人配置加载两路增益开关与上限。

本包的 RealSense 启动入口统一设置 `temporal_filter.enable: false` 和 `spatial_filter.enable: true`，关闭深度时间滤波、开启同帧空间滤波。
管理器保存、加载配置时会将对应高级参数归一化为上述状态，包含点分键和嵌套字典两种写法；
单台 D405/D435 启动也会覆盖自定义 `params_file` 中的相反值。更新代码并重新启动相机后生效。
验收脚本回读这两个参数，若与要求不符则报告失败，缺少回读值则报告无法确认。

[时间戳源码核查、转换公式与输出话题](docs/d405-humble-implementation.md)。

[手动验收脚本与结果说明](docs/manual_acceptance.md)：按需运行 `ros2 run humanoid_camera check_cameras.py`，
检查实际曝光、自动补偿响应、RGB/深度中点差和时间戳换算，输出终端、HTML、JSON 和 CSV 报告。
加 `--require-equal-exposure` 后，另要求每对 RGB/深度的实际曝光时长差为 0 μs；
曝光时长相等与曝光中点对齐分别验收。D435 默认双路 3900 μs；相同帧率、相同曝光时长和 enable_sync 成组发布
不能单独证明物理曝光同时发生，需以实际帧元数据验证。验收脚本只读取数据，不改变相机参数和连续采集链路。

需要照片证据时加 `--save-evidence`，默认每台相机每隔 10 秒抽样、最多保存 6 对 RGB/深度照片，并生成包含照片的 HTML 报告及 ZIP。RGB 保存为无损 PNG，深度 PNG 保留收到的 16 位像素数值，另附显示预览。每对照片附带逐帧来源元数据、曝光、增益、时间戳和 SHA-256；元数据未匹配时明确标记未知。失败或采样中按 Ctrl+C 也保留已收到的照片和部分结果。此选项同时启用图像验证，增加 RGBD 接收及后台写盘开销，不修改相机参数、QoS 或发布内容。

通用 `multi_camera.launch.py` 现在执行设备配置的 `startup`，其他品牌也随整机启动。
例如 `backend: vendor_camera` 可同时声明 `instance_parameters`、ROS launch 步骤、
`rgbd_topic` 与 `metadata_topic`；变量写作 `${name}`。不声明 startup 时连接外部已有话题。
RealSense 兼容配置独立在 `realsense_camera.launch.py` 和 `realsense_profile.py` 中；
使用该 profile 时请安装 `realsense2_camera` 和 `realsense2_camera_msgs`，通用相机包不再强制依赖它们。
