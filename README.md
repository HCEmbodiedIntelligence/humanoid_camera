# RealSense 多相机

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

`multi_camera.launch.py` 接受管理器部署在机器人内部配置目录中的 `cameras.yaml`。同时启用多台 RealSense 时每台必须填写唯一序列号。型号字段不会限制为 D405/D435；其他 RealSense 型号使用官方驱动的通用参数映射，设备不支持的能力需要在真机启动时处理。D435 彩色流默认使用 4500 μs 手动曝光；若在网页中启用彩色自动曝光，当前配置无法提供 5 ms 上限承诺。

单相机可用参数：`serial_no`、`params_file`、`camera_name`、`namespace`、`normalize_timestamps`。管理器的 ROS domain_id 必须与相机一致。

[时间戳源码核查、转换公式与输出话题](docs/d405-humble-implementation.md)。
