# 相机手动验收

相机启动后按需运行本脚本。它只订阅 ROS 话题和读取驱动参数，检查指定时间段后退出，
不会修改曝光、增益、启动或停止相机。常规启动流程不执行此验收脚本。

## 当前实现的范围

- D405：默认启用共享 depth_module 的自动曝光及自动增益限制；默认自动曝光上限 4500 μs、
  自动增益上限 64，页面的实际曝光要求默认不超过 5000 μs。自动亮度调节由官方相机驱动/固件完成。
- D435 系列：深度和 RGB 均使用手动曝光，默认 3900 μs；两路增益默认由程序独立调节。
  USB RGB 参数和逐帧 metadata 的 `actual_exposure` 均使用 100 μs 单位（39 对应 3900 μs），
  深度元数据使用 μs；验收先换算到 μs 再比较。原始值保留在报告中。
  其他独立 RGB 型号若启用原生自动曝光，目前不承诺硬件曝光上限，必须检查实际 metadata。
- RGB/深度通过驱动 frameset Header 配对，分别计算和保留曝光中点。
  enable_sync 不保证两路物理曝光同时发生；align_depth 是空间对齐选项，不是时间同步。

帧实际输出需要真机检查；参数值本身不能作为曝光上限或同步误差已经满足的证据。

D435 RGB 曝光元数据单位依据：[librealsense 的 RGB 元数据注册](https://github.com/realsenseai/librealsense/blob/v2.58.3/src/ds/ds-color-common.cpp)
将 `md_rgb_control.manual_exp` 原样传给 `ACTUAL_EXPOSURE`，未乘以 100；
[UVC 曝光控制单位](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/ext-ctrls-camera.html)为 100 μs。
这项换算仅作用于曝光时长，`sensor_timestamp`、`hw_timestamp` 等时间戳字段不乘以 100。

## 使用

在与相机相同的 ROS 环境及 domain_id 下执行。`--config` 指向机器人已部署的 `cameras.yaml`：

```bash
source /opt/ros/humble/setup.bash
source /home/czy/teleop_ws/install/setup.bash

ros2 run humanoid_camera check_cameras.py \
  --config /实际插件目录/robots/机器人ID/cameras.yaml \
  --duration 30 \
  --verify-images \
  --output ./camera_checks
```

脚本默认采用 ROS_DOMAIN_ID 环境变量，也可以加 `--domain-id 14` 显式指定。
默认检查配置中全部启用的 RealSense，相机数量由配置决定。
只验收指定相机可加 `--camera front`，此选项可重复。

要观察自动补偿响应，加 `--exercise-auto`，在采样期间手动降低和恢复光照。
至少观察到该自动曝光流的曝光或增益变化，才把“响应已观察到”算作满足；光照不变导致没有变化时为 UNKNOWN，
不直接判定固件故障。该检查不替代人眼对图像亮度、噪声和清晰度的判断。

要同时验收 RGB/深度的曝光中点对齐和实际曝光时长相等，加 `--require-equal-exposure`：

```bash
ros2 run humanoid_camera check_cameras.py \
  --config /实际插件目录/robots/机器人ID/cameras.yaml \
  --duration 60 --verify-images --require-equal-exposure \
  --output ./camera_checks
```

这两个要求独立判断：曝光中点差遵循 `rgbd_max_midpoint_skew_ms`（默认 1 ms），
每对 RGB/深度的 `actual_exposure` 换算到 μs 后，差必须为 0 μs。只比较两路最大值或平均值不能证明逐帧相等。
未提供有效曝光元数据时不能通过；不加该选项仍统计时长差，但报告明确不要求相等。
相机时长的量化差异也会报告不一致，不会静默放宽容差。

D435 默认双路手动曝光 3900 μs（驱动分别接收 RGB 39、深度 3900），两路自动增益保持独立。
加载旧的深度自动曝光配置时会迁移到这个预设；已有双路手动曝光值继续保留，需要在网页确认相等。
`--exercise-auto` 也可检查程序自动增益在光照变化时的实际增益响应；回读时不要求动态增益一直等于启动值。
D405 的共享模块可继续使用原生自动曝光，但同样以每对实际元数据为准。
相同帧率和曝光设定不会自动证明两路物理曝光同步，也不会强行改成相同时间戳。

## 保存照片用于汇报

加 `--save-evidence` 后，脚本从收到的标准化 RGBD 消息中抽样照片，自动启用 `--verify-images`。
例如头部 D435 的配置 ID 为 `camera_hand`、ROS domain 为 14 时：

```bash
ros2 run humanoid_camera check_cameras.py \
  --config "$HOME/.local/share/humanoid-plugins/robots/openarmx_01/cameras.yaml" \
  --domain-id 14 --camera camera_hand \
  --warmup 5 --duration 60 \
  --require-equal-exposure --exercise-auto \
  --save-evidence --evidence-interval 10 --evidence-max-pairs 6 \
  --output ./camera_checks
```

保持原来相机运行，采样期间改变光照再恢复。默认每台相机最多保存 6 对、相邻抽样至少间隔 10 秒，
只保存实际收到的图像；没有图像时会明确提示未取得照片证据。PNG 编码和写盘在后台执行，队列有上限，
写盘来不及时跳过抽样并记录错误。检查器不修改相机曝光、增益、发布 QoS 或图像像素；接收和写盘仍有资源开销。

每次测试的目录中增加：

- `evidence/<相机ID>/<样本编号>/rgb.png`：原分辨率无损 RGB 照片。
- 同目录 `depth.png`：收到的 16 位深度数值，包含当前驱动滤波结果；不是未滤波的传感器数据。
- 同目录 `depth_preview.png`：供人眼查看的灰度预览，零值为黑色；显示范围记录在 `frame.json`，不表示米制距离。
- 同目录 `frame.json`：RGB/深度各自的来源帧号、图像时间戳、曝光原始值及 μs 换算值、增益、源元数据和文件 SHA-256。

RGB 与深度取自同一条 RGBD 消息，保留各自时间戳。只有照片和元数据逐帧匹配时才填写曝光与增益，
匹配失败仍保存照片并标明未知。单张照片可以说明当时画面，连续性、曝光限制及同步检查仍依赖采样统计。

`report.html` 直接展示照片与对应帧的信息；终端打印同名 ZIP 路径，压缩包包含报告和照片，
可用于传递，解压后打开 HTML。FAIL 结果也正常保存；采样中按 Ctrl+C 会保留已收到的照片和部分结果，
标明未完成预定时长，不能作为完整通过的验收。旧版未保存的照片不能从历史统计报告恢复，需重新采样。

保存 PNG 需要 `python3-pil`、`python3-numpy`（已声明为包依赖）。若提示缺少依赖，安装这两个系统包后重试。


## 结果

每次运行创建单独的结果目录，不覆盖上次结果：

- `summary.txt`：终端摘要，逐台显示状态、帧率、实际曝光、中点差和软件换算残差。
- `report.html`：直接用浏览器打开，包含彩色 PASS / FAIL / UNKNOWN、增益与曝光统计、具体失败项和参数回读。
- `report.json`：结构化原始结果。
- `samples.csv`：逐对 RGB/深度中点和曝光数值，保留最近最多 10000 对帧。

主要检查项：

| 检查 | 依据 |
| --- | --- |
| 深度空间滤波开启 | 驱动 GetParameters 回读 spatial_filter.enable，必须为 true。 |
| 深度时间滤波关闭 | 驱动 GetParameters 回读 temporal_filter.enable，必须为 false；为 true 则 FAIL，未返回则 UNKNOWN。 |
| 曝光/增益设置生效 | 驱动 GetParameters 与保存配置比较，再检查原始帧 metadata 的 actual_exposure、gain_level、auto_exposure。 |
| 实际曝光上限 | 每个观测帧不超过配置 max_actual_exposure_us；深度自动模式还检查自动曝光及增益限制。 |
| RGB/深度曝光时长一致性 | 逐对统计 actual_exposure 的绝对差；加 --require-equal-exposure 后，差值非零为 FAIL，缺少有效数据不能通过。 |
| RGB/深度时间差 | 分别换算曝光中点，比较绝对差是否超过 rgbd_max_midpoint_skew_ms，默认 1 ms。 |
| 软件时间戳换算 | 使用原始 metadata 独立重算，核对标准化输出与来源帧号；换算差超过 1000 ns 为 FAIL。 |
| 图像 Header | 使用 --verify-images 时订阅 RGBD，分别核对 RGB/depth Header 和中点元数据；会增加图像传输开销。 |
| 数据连续性 | 观测帧号缺口、重复/倒退、结束时数据中断；接收帧率要求不低于配置的 80%，配对校验覆盖率至少 95%。 |
| 时钟异常 | 必须有 GLOBAL_TIME 映射依据，拒绝倒退、时钟 epoch 变化及异常采集到接收间隔。默认间隔上限 500 ms，可用 --max-age-ms 调整。 |

退出码：0 表示本次观测项 PASS；1 表示 FAIL；2 表示 UNKNOWN 或脚本未能完成；130 表示手动中断。
缺少图像或关键 metadata 不会显示通过。元数据丢包也会形成帧号缺口，因此 FAIL 提示的是观测链路异常，
不直接认定 USB 相机硬件丢帧。

## 相机时间如何映射到 ROS 2

1. librealsense 的 GLOBAL_TIME 持续采集设备时钟与主机系统时钟的对应关系，通过其线性时钟模型输出主机时间域的帧时间。
2. ROS 驱动提供该帧的 frame_timestamp（ms）及原始 hw_timestamp、sensor_timestamp（μs）。
3. 独立时间戳适配节点计算：

```text
曝光中点_ROS_ns = frame_timestamp_ms × 1,000,000
                 + signed_wrap32(sensor_timestamp_us − hw_timestamp_us) × 1,000
```

`sensor_timestamp` 按 SDK 定义已经是曝光中点，不再减半个曝光时间。
局部差值处理 uint32 微秒计数回绕。此模型使用 SDK 全局锚点和短时局部差，局部 scale 取 1。
RGB 和深度分别计算，不能直接把相同 frameset Header 当成两个曝光中点完全相同。

源码依据：[SDK 字段定义](https://github.com/realsenseai/librealsense/blob/v2.58.3/include/librealsense2/h/rs_frame.h)、
[SDK 全局时钟映射](https://github.com/realsenseai/librealsense/blob/v2.58.3/src/global_timestamp_reader.cpp)、
[ROS 驱动](https://github.com/realsenseai/realsense-ros/blob/4.58.3/realsense2_camera/src/base_realsense_node.cpp)。

这个验收使用设备/驱动报告的时间戳，**不能独立证明设备曝光中点的绝对误差小于 1 ms**。
采集到接收间隔包含传输和调度延迟，也不能当成时钟误差。报告因此始终保留
`physical_clock_accuracy_verified: false`。若需验证物理曝光同步或跨机绝对时间精度，
还需要公共光学事件、外部触发/测量设备或其他独立时间基准。
