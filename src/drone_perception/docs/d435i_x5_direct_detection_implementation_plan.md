# D435i + RDK X5 直采硬件预处理检测实施方案

## 1. 文档用途

本文档是面向代码实现 AI 或开发人员的工程交接说明，用于把当前 `warehouse_ws`
中的 D435i 检测链从“ROS 2 相机节点发布原始 RGB 图像”改为“感知进程内部直接采集”。

本阶段只完成：

1. D435i RGB `640x480@30` 直采。
2. Nano2D 硬件完成 `YUYV -> NV12` 和 `640x640` letterbox。
3. 现有 BPU YOLO 检测继续工作。
4. 现有二维码、OCR、抓拍和 ROS 2 检测结果接口保持可用。
5. 测量并降低相机采集、图像转换和 ROS 原始图传输产生的 CPU 开销。
6. 保留当前 `realsense2_camera_node` 路径作为回滚方案。

本阶段不完成实时图传、RTSP、RTP、H.264/H.265 编码或远端显示。图传只在本文末尾作为后续计划记录。

---

## 2. 结论和目标流程

### 2.1 本阶段目标流程

```text
D435i RGB Sensor
  |
  | USB 3.0, 640x480@30, YUYV
  v
qr_vision_node 进程内 librealsense 采集线程
  |
  | 最新帧覆盖队列，容量固定为 1
  v
Nano2D / GC820 硬件预处理
  |
  +--> 640x640 NV12 letterbox
  |      |
  |      v
  |    BpuYoloDetector::inferNv12()
  |      |
  |      v
  |    检测框、类别、置信度、OCR/二维码业务
  |
  +--> 按需生成 BGR/灰度图
         |
         +--> 二维码/ZBar、OCR、调试窗口、抓拍保存

ROS 2 只继续发布：检测结果、任务状态、抓拍结果等业务消息
ROS 2 不再传输：30 FPS 的 /camera/camera/color/image_raw
```

### 2.2 后续图传预留流程

本阶段只预留 `640x480 NV12` 输出接口，不接编码器：

```text
D435i YUYV
  -> Nano2D 640x480 NV12
  -> [后续] X5 VPU H.264 编码
  -> [后续] RTP/UDP 或 RTSP
```

不得为了图传提前引入 MediaMTX、GStreamer 管线或网络线程，以免扩大当前检测改动范围。

---

## 3. 已完成的实机验证

测试目标机：

- SSH：`sunrise@192.168.3.114`
- SoC：Horizon X5
- 架构：aarch64
- 内核：Linux 6.1.83
- D435i librealsense 设备序列号：`327122074056`
- D435i USB/UVC 字符串序列号：`302623061458`
- librealsense：2.57.7
- realsense2_camera：4.57.7
- D435i 固件：5.17.3.10
- USB：SuperSpeed，`5000M`，wrapper 报告 USB 3.2
- RGB V4L2 节点：`/dev/video4`

注意：librealsense 的设备序列号和 `/dev/v4l/by-id` 中的 UVC 字符串不同。调用
`rs2::config::enable_device()` 必须使用 `327122074056`，不能使用 `302623061458`。

### 3.1 已验证格式

`/dev/video4` 明确支持：

```text
YUYV 640x480 @ 60 FPS
YUYV 640x480 @ 30 FPS
YUYV 640x480 @ 15 FPS
YUYV 640x480 @ 6 FPS
```

本方案固定使用 `YUYV 640x480@30`。

### 3.2 CPU 和帧率实测

| 测试路径 | 稳态帧率 | CPU（单核口径） |
|---|---:|---:|
| 当前 `realsense2_camera_node`，RGB8 ROS 发布 | 30 FPS | 15.17%～15.58% |
| 直接 V4L2 YUYV 采集 | 29.98 FPS | 0.39% |
| librealsense 串号直采 YUYV | 29.98 FPS | 3.30% |
| librealsense + Nano2D，输出 640x480 NV12 | 29.98 FPS | 4.03% |
| librealsense + Nano2D，输出 640x640 letterbox NV12 | 29.98 FPS | 4.35% |

以上结果只比较相机采集和图像转换段，不包含完整 QR/OCR 业务 CPU。

### 3.3 BPU 闭环实测

已将 Nano2D 产生的 `640x640 NV12` 文件直接传给当前
`BpuYoloDetector::inferNv12()`：

- 输入大小：`614400` 字节，即 `640 * 640 * 3 / 2`。
- 连续完成 30 次 BPU 推理。
- 单次完整检测：约 9.69 ms。
- BPU wait：约 7.65 ms。
- 当时画面没有模型目标，检测框为 0；这不影响输入和推理链验证结论。

### 3.4 JPEG 保存实测

- Nano2D 输出的 NV12 使用软件 JPEG 编码后，画面和颜色正确。
- X5 `sample_codec` 使用 JPEG/JPU 内部缓冲模式时，画面正确。
- `external_buffer=1` 时出现紫色横纹，说明外部缓冲的 stride/cache/布局尚未正确配置。

本阶段保存图片默认保持现有 OpenCV/JPEG 路径。JPU 优化只作为可选后续项，不能直接使用已发现异常的 external buffer 模式。

---

## 4. 当前工程真实状态

### 4.1 当前相机启动入口

文件：

```text
/home/gjl/warehouse_ws/src/drone_bringup/scripts/start_d435.sh
```

当前行为：

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  rgb_camera.color_profile:=640,480,30 \
  enable_depth:=false \
  enable_rgbd:=false \
  enable_sync:=false \
  align_depth.enable:=false
```

目标机由以下 service 拉起：

```text
/etc/systemd/system/D435I_start.service
```

该服务运行 `start_d435.sh`，并且脚本前有固定 `sleep 10`。实现后继续复用这个
`D435I_start.service`，只把它调用的现有脚本改为启动直采检测链；不新增第二个 service。
手动测试脚本前必须先停止该服务，避免两个进程同时占用相机。

### 4.1.1 本地开发与 RDK X5 验证边界

本项目采用“本地 PC 修改，RDK X5 拉取验证”的固定流程：

1. 所有 C++、CMake、launch、Shell 和 systemd 相关文件，先只在本地 PC 的
   `warehouse_ws` 工作区修改。
2. 本地完成代码审查、可执行的构建检查后，提交一个可追踪的 Git commit。
3. 将该 commit 推送到双方可访问的 Git remote/branch；否则 RDK X5 无法拉取只存在
   于本地的 commit。
4. RDK X5 不接受通过 SSH 直接编辑、覆盖或临时修改源码；板端只通过 Git 拉取
   已推送的内容。
5. RDK X5 上的操作仅限依赖检查、编译、安装、运行测试、日志采集和 systemd
   启停验证。测试日志、图片和构建产物不得反向覆盖源码。
6. 每次板端验证必须记录 commit ID，使测试结果对应到确切代码版本。

`start_d435.sh` 可以在后续实现中直接改为新的直采检测启动入口，但修改动作必须发生
在本地 PC。提交后由 RDK X5 通过 Git 拉取；禁止直接在 RDK X5 上用编辑器、`sed -i`
或重定向命令修改该脚本。

### 4.2 当前图像订阅入口

文件：

```text
/home/gjl/warehouse_ws/src/drone_perception/src/qr_vision_node.cpp
/home/gjl/warehouse_ws/src/drone_perception/include/drone_perception/qr_vision_node.hpp
```

当前参数：

```text
color_topic = /camera/camera/color/image_raw
camera_info_topic = /camera/camera/color/camera_info
```

当前订阅：

```cpp
color_sub_ = create_subscription<sensor_msgs::msg::Image>(...);
camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(...);
```

当前工作线程会执行：

```cpp
cv_bridge::toCvShare(color_frame.message, "bgr8");
processFrame(color_bridge, ...);
```

### 4.3 当前 BPU 预处理

`QrVisionNode::prepareBpuInput()` 当前执行：

```text
BGR 640x480
  -> OpenCV resize
  -> BGR 640x640 letterbox
  -> cv::cvtColor(BGR -> I420)
  -> CPU 循环打包 NV12
  -> BPU input vector
```

直采模式的目标是跳过这条 CPU BGR 预处理链，由 Nano2D 直接生成
`640x640 NV12`。

### 4.4 当前 BPU 接口

文件：

```text
/home/gjl/warehouse_ws/src/drone_perception/include/drone_perception/bpu_yolo_detector.hpp
/home/gjl/warehouse_ws/src/drone_perception/src/bpu_yolo_detector.cpp
```

可直接复用：

```cpp
std::vector<BpuYoloDetection> inferNv12(
    const uint8_t *nv12_data,
    std::size_t nv12_size);
```

该函数要求输入长度严格等于模型 input tensor 的 `alignedByteSize`。当前模型输入实测为 614400 字节。

---

## 5. 设计原则和禁止事项

### 5.1 必须遵守

1. 保持 `640x480@30` 相机 profile 不变。
2. 只启用 `RS2_STREAM_COLOR`，格式固定 `RS2_FORMAT_YUYV`。
3. 采集线程不得执行 BPU、OCR、二维码或 JPEG 编码。
4. 帧队列容量必须固定为 1，新帧覆盖旧帧，禁止积压。
5. 业务线程处理不过来时丢旧帧，不能阻塞相机回调。
6. 退出时必须先停止采集，再释放 Nano2D/BPU 缓冲。
7. 相机断开后必须有明确错误状态和有限频率重连，不得忙等。
8. 直采模式与 `realsense2_camera_node` 不能同时启用。
9. 首版允许一次 YUYV CPU memcpy 到 Nano2D 输入缓冲，不强做零拷贝。
10. 现有 ROS 业务消息、检测结果和任务逻辑不能因采集方式改变而改名。

### 5.2 本阶段禁止

1. 不删除 librealsense 或 realsense2_camera apt 包。
2. 不裁剪 SDK 源码。
3. 不加入 H.264/H.265、RTSP、RTP 或 MediaMTX。
4. 不把 `sample_codec` 作为长期运行子进程嵌入检测节点。
5. 不直接启用 `external_buffer=1` 的 JPU 路径。
6. 不硬编码 `/dev/video4` 作为 librealsense 设备选择依据。
7. 不把相机采集塞进 ROS subscription callback。
8. 不把调试窗口作为检测链的必需条件。

---

## 6. 建议代码结构

### 6.1 新增文件

建议新增：

```text
src/drone_perception/include/drone_perception/d435_color_capture.hpp
src/drone_perception/src/d435_color_capture.cpp

src/drone_perception/include/drone_perception/nano2d_preprocessor.hpp
src/drone_perception/src/nano2d_preprocessor.cpp
```

可选新增测试：

```text
src/drone_perception/src/d435_capture_probe.cpp
src/drone_perception/src/nano2d_preprocess_probe.cpp
```

### 6.2 `D435ColorCapture` 职责

该类只负责：

- 按 librealsense 序列号选择相机。
- 配置 color-only `640x480@30 YUYV`。
- 启动和停止 pipeline。
- 在独立线程中等待帧。
- 保存最新一帧及其时间戳。
- 暴露最新帧获取接口。
- 处理断开、超时和有限频率重连。

建议接口：

```cpp
struct D435ColorFrame
{
  std::uint64_t sequence{0};
  double device_timestamp_ms{0.0};
  std::chrono::steady_clock::time_point received_at{};
  int width{0};
  int height{0};
  int stride_bytes{0};
  std::vector<std::uint8_t> yuyv;
};

class D435ColorCapture
{
public:
  struct Config
  {
    std::string serial;
    int width{640};
    int height{480};
    int fps{30};
    int wait_timeout_ms{2000};
    int reconnect_delay_ms{2000};
  };

  explicit D435ColorCapture(Config config);
  ~D435ColorCapture();

  bool start();
  void stop();
  bool waitForLatest(D435ColorFrame &frame, std::chrono::milliseconds timeout);
  bool running() const;
  std::string lastError() const;
};
```

实现要求：

```cpp
rs2::config config;
config.disable_all_streams();
config.enable_device(serial);
config.enable_stream(
    RS2_STREAM_COLOR,
    640,
    480,
    RS2_FORMAT_YUYV,
    30);
```

不要把 `rs2::video_frame` 跨线程长期保存。首版在采集线程内把 YUYV 拷贝进固定容量缓冲，再通过双缓冲或 latest-frame 交换给处理线程，避免 SDK frame 生命周期不清晰。

### 6.3 `Nano2DPreprocessor` 职责

该类负责：

- 初始化 `n2d_open()`、device 0、core 0。
- 一次性分配 YUYV 输入缓冲。
- 一次性分配 `640x640 NV12` BPU 缓冲。
- 初始化 letterbox 填充色。
- 每帧复制 YUYV 到 Nano2D 输入缓冲。
- 使用 `n2d_blit()` 把图像放入 `y=80`、高度 480 的目标区域。
- 调用同步 commit，确保 BPU 读取前转换完成。
- 返回连续的 614400 字节 NV12 数据。

建议接口：

```cpp
struct Nv12FrameView
{
  const std::uint8_t *data{nullptr};
  std::size_t size{0};
  int width{0};
  int height{0};
  int stride{0};
};

class Nano2DPreprocessor
{
public:
  Nano2DPreprocessor();
  ~Nano2DPreprocessor();

  bool initialize(int source_width, int source_height,
                  int model_width, int model_height);
  bool convertYuyvToLetterboxNv12(
      const std::uint8_t *yuyv,
      int source_stride,
      Nv12FrameView &output);
  void shutdown();
  std::string lastError() const;
};
```

首版固定几何参数：

```text
source = 640x480
model  = 640x640
scale  = 1.0
pad_x  = 0
pad_y  = 80
```

`bpu_letterbox_` 必须同步设置成：

```text
scale = 1.0
pad_x = 0
pad_y = 80
```

否则检测框从模型坐标还原到原始图像坐标时会出现 80 像素纵向偏差。

### 6.4 Nano2D 资源生命周期

初始化顺序：

```text
n2d_open
 -> n2d_switch_device(N2D_DEVICE_0)
 -> n2d_switch_core(N2D_CORE_0)
 -> allocate YUYV 640x480
 -> allocate NV12 640x640
 -> fill NV12 letterbox background
```

每帧：

```text
copy YUYV rows respecting source_stride and n2d stride
 -> n2d_blit(destination_rect={0,80,640,480})
 -> n2d_commit_ex(N2D_TRUE)
 -> provide NV12 to inferNv12
```

释放顺序：

```text
n2d_free(NV12)
 -> n2d_free(YUYV)
 -> n2d_close
```

错误路径也必须按逆序释放已经成功创建的资源。

---

## 7. `QrVisionNode` 改造方案

### 7.1 新增参数

建议增加：

```text
camera_input_mode: "ros" | "d435_direct"
d435_serial: "327122074056"
d435_width: 640
d435_height: 480
d435_fps: 30
d435_wait_timeout_ms: 2000
d435_reconnect_delay_ms: 2000
direct_input_debug_bgr: true
```

默认值建议先保持：

```text
camera_input_mode = "ros"
```

完成实机验收后，再把部署配置切换为 `d435_direct`。不要在第一次代码提交时直接删除 ROS 输入模式。

### 7.2 初始化分支

构造顺序建议调整为：

```text
declareParameters
 -> initializeBpuDetector
 -> initializeBpuOcrPipeline
 -> initializeVisualCodeDecoder
 -> 根据 camera_input_mode 初始化输入
 -> startVisionWorker
```

模式行为：

```text
ros:
  保持 color_sub_ 和 camera_info_sub_

d435_direct:
  不创建 color_sub_
  不要求 camera_info_sub_
  创建 D435ColorCapture 和 Nano2DPreprocessor
  启动直采线程
```

### 7.3 工作项结构

当前 `PendingColorFrame` 只保存 ROS Image。建议改为统一工作项：

```cpp
enum class CameraInputMode
{
  RosImage,
  D435Direct
};

struct PendingVisionFrame
{
  std::uint64_t sequence{0};
  CameraInputMode mode{CameraInputMode::RosImage};
  sensor_msgs::msg::Image::ConstSharedPtr ros_image;
  std::shared_ptr<D435ColorFrame> direct_frame;
  std::chrono::steady_clock::time_point enqueue_time{};
};
```

两种模式都只保留最新工作项。

### 7.4 BPU 路径拆分

当前 `processFrame()` 同时负责 BGR 图、BPU、QR、OCR 和显示。建议拆成：

```cpp
void processRosFrame(...);
void processDirectFrame(const D435ColorFrame &frame);

void runBpuDetection(
    const std::uint8_t *nv12,
    std::size_t nv12_size,
    const BpuLetterboxState &letterbox);

void runVisualCodeAndOcr(const cv::Mat &bgr);
void updateDebugView(const cv::Mat &bgr);
```

直采模式：

```text
YUYV
 -> Nano2D 640x640 NV12
 -> runBpuDetection

只有当 QR/OCR/debug/save 确实需要时：
YUYV -> BGR 或 YUYV Y分量 -> gray
```

### 7.5 QR/OCR 兼容策略

当前代码不仅做 YOLO，还做二维码、条码、OCR 和调试显示。因此不能简单删除 BGR。

第一阶段为了功能正确性，可以：

```cpp
cv::Mat yuyv(height, width, CV_8UC2, frame.yuyv.data(), frame.stride_bytes);
cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);
```

但 BPU 必须直接使用 Nano2D NV12，不得再从 BGR 走 `prepareBpuInput()`。

第二阶段再优化：

- ZBar/QR 优先直接使用 YUYV 的 Y 分量生成灰度图。
- OCR 只对候选 ROI 生成 BGR/灰度。
- `debug_view=false` 时不生成全帧 BGR 显示副本。
- 抓拍触发时才生成 JPEG。

这样可以先保持正确性，再逐步减少 CPU，避免一次改动过大。

### 7.6 CameraInfo 和标定

本阶段只做二维 BPU 检测、二维码/OCR 和保存图片，不依赖内参。

直采模式不发布或订阅 `CameraInfo` 不会擦除 D435i 内部标定。未来重新启动
`realsense2_camera_node` 时，`/camera/camera/color/camera_info` 会恢复。

不要在直采模式下伪造 CameraInfo。如果未来加入去畸变、PnP、角度计算或深度对齐，再按相机序列号和 profile 单独保存标定 YAML。

---

## 8. 线程和缓冲设计

### 8.1 推荐线程

```text
ROS executor thread
  - 业务 topic、参数、控制回调

D435 capture thread
  - wait_for_frames
  - YUYV copy to latest buffer
  - notify vision worker

vision worker thread
  - Nano2D conversion
  - BPU inference
  - QR/OCR
  - result publish
  - optional debug/save
```

### 8.2 latest-frame 语义

采集速度大于处理速度时：

```text
frame N waiting
frame N+1 arrives -> overwrite N
worker always takes newest frame
```

统计项必须保留：

- input frame count
- processed frame count
- overwritten/dropped frame count
- capture wait timeout count
- reconnect count
- Nano2D failure count
- BPU failure count
- latest frame age
- preprocessing latency
- BPU latency
- total processing latency

### 8.3 锁范围

锁内只允许：

- 交换 shared pointer 或 buffer index
- 更新短小状态
- 递增计数器

锁内禁止：

- `wait_for_frames()`
- `n2d_blit()`
- BPU infer
- OpenCV conversion
- JPEG save
- ROS publish

---

## 9. 构建系统修改

文件：

```text
/home/gjl/warehouse_ws/src/drone_perception/CMakeLists.txt
/home/gjl/warehouse_ws/src/drone_perception/package.xml
```

### 9.1 librealsense

目标机真实库位置：

```text
/opt/ros/humble/include/librealsense2
/opt/ros/humble/lib/aarch64-linux-gnu/librealsense2.so.2.57.7
```

目标机的 `realsense2.pc` 曾发现错误地指向 `x86_64-linux-gnu`。因此实现时不要只依赖当前 pkg-config 输出。

优先：

```cmake
find_package(realsense2 REQUIRED)
target_link_libraries(qr_vision_node realsense2::realsense2)
```

如果目标机导出的 CMake target 不可用，再使用 `find_path()` 和 `find_library()`，并显式搜索：

```text
/opt/ros/humble/include
/opt/ros/humble/lib/aarch64-linux-gnu
```

禁止在 CMake 中写死版本文件名 `librealsense2.so.2.57.7`。

### 9.2 Nano2D

目标机真实内容：

```text
/usr/include/GC820/nano2D.h
/usr/include/GC820/nano2D_util.h
/usr/hobot/lib/libNano2D.so
/usr/hobot/lib/libNano2Dutil.so
```

建议：

```cmake
find_path(NANO2D_INCLUDE_DIR GC820/nano2D.h PATHS /usr/include)
find_library(NANO2D_LIBRARY Nano2D PATHS /usr/hobot/lib)
find_library(NANO2D_UTIL_LIBRARY Nano2Dutil PATHS /usr/hobot/lib)
```

只在三项都找到时定义：

```text
DRONE_PERCEPTION_HAS_NANO2D=1
```

否则：

- `camera_input_mode=ros` 仍可构建和运行。
- 用户选择 `d435_direct` 时必须明确报错并退出。
- 不允许静默退回低性能路径而不记录日志。

### 9.3 新增源码

`qr_vision_node` target 增加：

```text
src/d435_color_capture.cpp
src/nano2d_preprocessor.cpp
```

保留当前 `bpu_yolo_detector.cpp`、OCR 和 ZBar 依赖。

### 9.4 package.xml

如果 ROS 环境提供对应 package metadata，增加 librealsense 的依赖声明。Nano2D 属于板端厂商系统库，若没有 rosdep key，应在 README/部署文档明确列为系统前置依赖，不得编造不存在的 rosdep 包名。

---

## 10. 参数和配置文件

建议新增：

```text
src/drone_perception/config/d435_direct_detection.yaml
```

建议内容：

```yaml
qr_vision_node:
  ros__parameters:
    camera_input_mode: d435_direct
    d435_serial: "327122074056"
    d435_width: 640
    d435_height: 480
    d435_fps: 30
    d435_wait_timeout_ms: 2000
    d435_reconnect_delay_ms: 2000

    enable_bpu: true
    debug_view: false
    camera_controls_enabled: false
```

相机序列号必须是字符串，避免 YAML 将其按整数处理。

增加启动参数校验：

- width 不是 640：当前阶段拒绝启动。
- height 不是 480：当前阶段拒绝启动。
- fps 不是 30：当前阶段拒绝启动。
- serial 为空：枚举设备；只有一个 D435i 时允许选择并打印其序列号，多台时拒绝自动选择。

---

## 11. 使用现有 systemd 服务启动

### 11.1 开发阶段

先停止现有相机服务，释放设备给手动测试：

```bash
sudo systemctl stop D435I_start.service
```

确认没有相机占用：

```bash
pgrep -a -f realsense2_camera_node
fuser /dev/video4
```

启动直采检测：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/warehouse_ws/install/setup.bash

ros2 run drone_perception qr_vision_node --ros-args \
  --params-file /home/sunrise/warehouse_ws/src/drone_perception/config/d435_direct_detection.yaml
```

### 11.2 首版部署方式

首版允许直接修改并复用以下启动脚本：

```text
/home/gjl/warehouse_ws/src/drone_bringup/scripts/start_d435.sh
```

但必须在本地 PC 修改、检查并提交，再由 RDK X5 拉取。脚本需要：

- 加载 ROS 2 和 `warehouse_ws/install` 环境。
- 启动 `qr_vision_node` 的 `d435_direct` 模式，不再启动 `realsense2_camera_node`。
- 将节点退出码传给 systemd，启动失败时输出明确错误。
- 收到 SIGINT/SIGTERM 后让节点正常释放 librealsense、Nano2D 和相机句柄。
- 保证同一时间只有一个进程打开 D435i。

本地提交后先推送到双方可访问的远端分支：

```bash
# 本地 PC
git push origin <development-branch>
```

推送成功后，RDK X5 只拉取指定提交并构建：

```bash
cd ~/warehouse_ws
git fetch origin
git checkout <tested-commit>
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_perception drone_bringup --symlink-install
```

先停止 service，再手动验证脚本：

```bash
sudo systemctl stop D435I_start.service
bash -x ~/warehouse_ws/src/drone_bringup/scripts/start_d435.sh
```

手动验证必须确认：相机序列号正确、`YUYV 640x480@30`、输入接近 30 FPS、Nano2D
和 BPU 日志均出现；按 `Ctrl-C` 后节点与相机句柄正常释放。手动验证不通过时不得继续
自启动测试。

手动验证通过后，再测试现有 service：

```bash
sudo systemctl restart D435I_start.service
systemctl --no-pager --full status D435I_start.service
journalctl -u D435I_start.service -b --no-pager -n 200
pgrep -a qr_vision_node
fuser -v /dev/video4
```

验收标准：service 为 `active (running)`；只有预期的直采检测进程持有相机；日志中
没有 `realsense2_camera_node`、`Device or resource busy` 或反复重启。最后重启 RDK X5，
再次执行上述状态、日志、进程和设备占用检查，确认它能真正随系统自动启动并持续运行。

若需要回滚，RDK X5 切回旧 Git commit，重新构建后重启原 service：

```bash
cd ~/warehouse_ws
git checkout <previous-working-commit>
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_perception drone_bringup --symlink-install
sudo systemctl restart D435I_start.service
```

本方案不新增、替换、重命名或删除任何 systemd service。正式运行和开机自启动始终只
使用原有 `D435I_start.service`；`status`、`journalctl`、`pgrep` 和 `fuser` 仅是验收与
排错命令，不构成新的启动入口。

---

## 12. 分阶段实现顺序

### 阶段 A：只增加直采能力

改动：

1. 增加 `D435ColorCapture`。
2. 增加 `camera_input_mode` 参数。
3. 直采 YUYV 后用 OpenCV 转 BGR，送入现有 `processFrame()`。
4. 保持现有 BPU CPU 预处理不变。

目的：

- 先证明不经过 ROS Image，业务检测、QR、OCR、保存都仍正确。
- 隔离“相机输入改造”和“Nano2D/BPU预处理改造”。

验收后再进入阶段 B。

### 阶段 B：Nano2D 直供 BPU

改动：

1. 增加 `Nano2DPreprocessor`。
2. 输出 `640x640 NV12`，上下 padding 各 80。
3. BPU 直接调用 `inferNv12()`。
4. 跳过当前 `prepareBpuInput()` 的 BGR/I420/NV12 CPU 转换。
5. 修正检测框反 letterbox 参数。

目的：

- 获得主要 CPU 收益。
- 保持模型输入和现有模型文件不变。

### 阶段 C：按需生成 BGR/灰度

改动：

1. `debug_view=false` 时不生成显示用全帧 BGR。
2. QR/ZBar 优先使用 Y 分量灰度。
3. OCR 只转换检测 ROI。
4. 抓拍时才编码 JPEG。

目的：

- 继续降低 CPU。
- 不改变识别结果语义。

### 阶段 D：现有服务自启动验收

改动：

1. 不新增 service，确认原有 `D435I_start.service` 继续调用现有 `start_d435.sh`。
2. 做 30 分钟稳定性测试。
3. 做相机拔插恢复测试。
4. 做断电重启测试。
5. 确认原有服务在开机后自动启动直采检测链。

---

## 13. 测试和验收标准

### 13.1 构建验收

目标机执行：

```bash
cd /home/sunrise/warehouse_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_perception --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

必须满足：

- 无编译错误。
- 无未解析 `realsense2`、`Nano2D`、`Nano2Dutil`、`dnn` 符号。
- x86/无 Nano2D 环境仍能构建 ROS 输入模式，或给出明确的构建选项说明。

### 13.2 相机验收

```bash
lsusb -t
```

必须看到 D435i 位于 `5000M`。

运行日志必须打印：

```text
serial=327122074056
format=YUYV
width=640
height=480
fps=30
```

30 秒统计：

- 输入平均帧率：29.5～30.5 FPS。
- 无持续 wait timeout。
- 无 USB disconnect、URB、`-71` 错误。
- 最新帧队列允许丢旧帧，但不得持续增长。

### 13.3 Nano2D 验收

必须记录：

```text
source format=YUYV 640x480
destination format=NV12 640x640
destination size=614400
pad_x=0 pad_y=80 scale=1.0
```

保存一帧测试图并人工确认：

- 颜色正常。
- 无紫色横纹。
- 上下 padding 正常。
- 图像未拉伸。
- 左右没有异常裁剪。

### 13.4 BPU 检测验收

必须使用包含已知目标的实景测试，不能只以“推理函数返回成功”作为检测正确性结论。

验收内容：

- 模型能检测出已知目标。
- 检测框与目标重合。
- 上下坐标没有约 80 像素偏移。
- 类别和置信度与旧 ROS 路径基本一致。
- 同场景旧路径和新路径分别保存至少 30 帧结果做对比。

### 13.5 业务回归

逐项验证：

- BPU YOLO 检测。
- QR/条形码识别。
- OCR。
- hover/任务触发。
- `BarcodeCapture` 发布。
- 抓拍图片保存和内容正确性。
- `debug_view=false` 无窗口运行。
- 节点 SIGINT 后正常退出。

### 13.6 性能验收

命令：

```bash
pidstat -p $(pgrep -n qr_vision_node) 1 60
```

同时记录节点现有性能日志：

- input_fps
- process_fps
- dropped frames
- frame_age p50/p95
- Nano2D ms
- BPU total/wait ms
- QR/OCR ms
- total process ms

目标：

- 相机和 Nano2D 段维持约 4%～6% 单核 CPU。
- 完整节点 CPU 必须低于旧的“realsense2_camera_node + qr_vision_node”总和。
- 稳态输入接近 30 FPS。
- 处理跟不上时延迟不能无限增加，应丢旧帧保持实时性。

### 13.7 恢复测试

测试顺序：

1. 正常运行 2 分钟。
2. 拔出 D435i。
3. 确认节点进入 camera unavailable 状态，不崩溃、不忙等。
4. 等待 5 秒后插回同一 USB 3.0 口。
5. 确认节点按有限频率重连并恢复 30 FPS。

若第一版决定“相机断开即退出并交给 systemd 重启”，也可以接受，但必须明确实现这一策略，不能处于半重连状态。

---

## 14. 风险和处理

### 14.1 相机争用

现象：

```text
Device or resource busy
No device connected
```

处理：

- 确认 `D435I_start.service` 已停止。
- 确认没有 `realsense2_camera_node`。
- 使用 `fuser /dev/video4` 检查持有者。
- 停止后等待 1～3 秒再打开，避免快速启停进入坏状态。

### 14.2 SDK 序列号混淆

librealsense 使用：

```text
327122074056
```

不能使用 UVC by-id 字符串中的：

```text
302623061458
```

更换相机后必须重新枚举，不得沿用旧序列号。

### 14.3 Nano2D cache/stride

首版使用 Nano2D 自己分配的内部 buffer，并同步 commit。每行复制必须分别使用源 stride 和 Nano2D stride。

若出现：

- 紫色横纹
- 颜色错位
- 上下撕裂
- 每隔固定行重复

优先检查：

- YUYV stride
- NV12 Y/UV plane 地址
- UV stride
- buffer cache flush/invalidate
- external buffer 是否错误启用

### 14.4 检测框偏移

若框整体向上或向下偏移约 80 像素，说明 `bpu_letterbox_` 没有使用
`scale=1.0, pad_y=80` 做反变换。

### 14.5 x86 开发机兼容

Nano2D 和 BPU 只存在于 RDK X5。代码必须使用编译宏隔离板端实现，不得让普通 x86 阅读/构建环境因为缺少厂商头文件完全失效。

---

## 15. 回滚方案

代码层必须保留：

```text
camera_input_mode=ros
```

出现任何功能性回归时：

1. 停止原有 `D435I_start.service`。
2. 将参数切回 `camera_input_mode=ros`。
3. 将本地代码和 `start_d435.sh` 切回旧 Git commit，推送后由 RDK X5 拉取并重建。
4. 重新启动同一个 `D435I_start.service`。
5. 验证 `/camera/camera/color/image_raw` 约 30 FPS。
6. 启动旧版 `qr_vision_node`。

回滚不能依赖重新安装 SDK，也不应需要重新插拔相机，除非设备本身进入 USB 错误状态。

---

## 16. 后续图传计划（本阶段不实现）

检测链稳定后，再设计：

```text
D435i YUYV
 -> Nano2D 640x480 NV12
 -> X5 VPU H.264/H.265 hardware encoder
 -> RTP/UDP or RTSP
 -> remote hardware decoder/display
```

后续原则：

1. 检测和图传共享相机采集，不允许第二次打开 D435i。
2. BPU 使用 `640x640 NV12`，图传使用 `640x480 NV12`，由预处理层输出两路。
3. 编码参数初值：H.264、CBR 2～4 Mbit/s、30 FPS、GOP 15 或 30、B 帧 0。
4. 使用 X5 `hb_media`/VPU API，不把 `sample_codec` CLI 当生产服务。
5. 第一版使用内部编码缓冲，稳定后再处理 hbmem/DMA-BUF 零拷贝。
6. 网络发送队列容量固定为 1，丢旧帧，禁止延迟积压。
7. 图传失败不能阻塞 BPU 检测。

只有本阶段所有检测验收项通过后，才开始图传实现。

---

## 17. 给代码 AI 的执行清单

代码 AI 接手后按以下顺序执行，不得跳步：

1. 阅读完整文件：
   - `qr_vision_node.hpp`
   - `qr_vision_node.cpp`
   - `bpu_yolo_detector.hpp`
   - `bpu_yolo_detector.cpp`
   - `CMakeLists.txt`
   - `package.xml`
   - `start_d435.sh`
2. 确认当前 git 状态，保留用户已有修改和 `test_logs/`；不得直接修改 RDK X5 工作区。
3. 只在本地 PC 实现阶段 A、检查并提交；RDK X5 只拉取该 commit 后构建验证。
4. 阶段 A 功能回归通过后，再实现阶段 B。
5. 用真实目标验证检测框和 letterbox 坐标，不只看函数返回值。
6. 阶段 B 稳定后再做阶段 C 的按需 BGR/灰度优化。
7. 在本地 PC 修改现有 `start_d435.sh`，提交后再由 RDK X5 拉取；不得通过 SSH
   直接编辑板端脚本或源码。
8. 先手动运行并验证 `start_d435.sh`，再验证 `D435I_start.service` 的重启和开机自启动。
9. 每阶段单独提交，记录 RDK X5 实测对应的 commit ID，确保可以局部回滚。
10. 不实现本文第 16 节图传计划。

完成定义：

```text
在 RDK X5 上，D435i 以 YUYV 640x480@30 被 qr_vision_node 直接采集；
Nano2D 生成正确的 640x640 NV12 letterbox；
BPU、QR/OCR、抓拍和 ROS 业务结果通过回归；
完整检测链稳定运行且 CPU 低于旧的 ROS RGB8 链；
旧 ROS 相机路径仍可一键回滚。
```
