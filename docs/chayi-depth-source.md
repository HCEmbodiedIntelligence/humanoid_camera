# G1 R2 RealSense ROS 输出单位（候选）

当前确认的来源为 `realsense2_camera/4.57.7`、配置 Z16、`depth/image_rect_raw`。
[该版本官方 fix_depth_scale](https://github.com/realsenseai/realsense-ros/blob/4.57.7/realsense2_camera/src/base_realsense_node.cpp#L252-L287)
将传感器值按实际 `_depth_scale_meters` 转成 ROS 毫米；发布路径调用该函数。
因此适配器对这一个版本/格式记录 16UC1、mm、0.001 m/码、零无效和未注册 RGB 的原深度网格。
这不是任意 SDK 原始 Z16 的通用毫米假设，也不是实际精度或硬件同步验收。

从安装包元数据读取版本，从实际合并后的 launch 参数读取格式。未知版本/格式仍转发原图，source_profile/scale 为 null，不猜单位。
消息记录编码、大小端、row_step、驱动版本及格式；不重缩放像素。不访问设备获取这些声明。
平台最终对每帧实际编码、dtype、shape 和 scale 与声明进行校验。

隔离路径 `teleop_ws-chayi-work/install/ros` 完成 colcon 构建，未更改日常安装。
pipeline diagnostics、driver streams、depth processing 共 23 passed；测试使用官方 ROS 消息类型但未启动节点或硬件。
平台记录配套提交与来源模块哈希；真实设备、完整 R3 链路与 G2 部署均未执行。
