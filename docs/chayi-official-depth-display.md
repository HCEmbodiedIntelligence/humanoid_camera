# G1 官方深度显示（候选）

按用户要求，删除手写彩虹插值和灰度百分位拉伸，直接调用 `pyrealsense2.colorizer`。
默认官方 Dynamic/Jet，开启直方图均衡；灰度预览使用官方 White to Black。
零值由 SDK 显示为黑色，动态颜色不等于固定距离刻度；图例仅说明模式，不编造米制色带。
预览不更改原始数值 PNG、原始录制、平台灰度 HEVC、量化 profile 或 RGB H.265 原文件。

显式指定 min/max 时仍可使用 SDK 固定范围选项，但必须有逐帧匹配的、受支持 RealSense ROS 驱动的单位依据。
未知来源只作动态排序着色，不声称物理单位。SDK 接口必需的临时 units=1 在 profile 中标为显示占位值，绝不是源尺度。
缺少或版本不符的 SDK 会报告错误、保留原始 PNG，不回退到自定义显示。

依赖 `requirements-preview.txt` 固定 pyrealsense2 2.58.4.10922；已在独立路径
`teleop_ws-chayi-work/install/preview-python` 安装官方 cp310 wheel，SHA-256：
`809d48b329890594ab158c924537d7ddd6b15acab0aa22d6ba29e974f09ddc9d`。
不更新用户目录或日常 ROS 安装。colcon 构建安装到隔离 `install/ros`，SDK 依赖文件也随包安装。

依据：[官方色彩处理实现](https://github.com/realsenseai/librealsense/blob/master/src/proc/colorizer.cpp)、
[官方 software-device 示例](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/numpy_to_frame.py)、
[固定版本发行包](https://pypi.org/project/pyrealsense2/2.58.4.10922/)。

77 项隔离 Python 3.10/ROS 消息类型测试通过，无跳过：其中包含 R2 单位/原始录制和相机参数回归，以及 31 项显示/证据测试。
逐像素对照使用直接构造的 SDK 默认 colorizer；覆盖 Jet/灰度、动态/固定范围、大小端、全无效帧、原件不变、单位不足与 SDK 不匹配。
命令退出路径测试替换 ROS 回调，只证明故障时保存已有证据；不计作真实设备或真实服务验收。
另在平台 Python 3.12/固定 SDK 镜像验证 28 项显示测试；3 项 ROS 命令测试在该镜像缺少 rclpy，已由前述主机测试覆盖。

首次主机执行受用户目录旧 SDK 崩溃和沙箱 udev 限制影响。
现已禁用用户 site-packages、使用校验过的独立安装，并在获得沙箱外软件测试许可后通过。
SDK 初始化 udev 监听不等于启动相机采集；测试没有打开实际传感器、ROS 节点或机器人运动。

G2 条件：运行诊断命令的解释器必须能加载固定版本 SDK；先运行离线像素测试，再进行另外授权的实际验收。
本次只形成 G1 候选，不执行 G2/G3。回退使用前一应用版本并保留已有原件/预览证据；平台主文件不受此显示改动影响。
