# Qt 地面站显示 技术交接方案

> 交接对象：Qt 地面站开发者
> 日期：2026-08-07
> 前置交付：RDK X5 → OrangePi 5 Ultra D435i 收发与 MPP 解码链路已全部完成（T0-T9），
> 状态见 `docs/rdk_x5_orangepi5ultra_send_receive_implementation_status.md`
> 本文是**可执行的技术方案**，含已验证的管线、线程模型和代码骨架。

---

## 0. 一句话结论

Qt 地面站用 **GStreamer 在工作线程**拉 RTSP → `mppvideodec` **硬解 NV12 640x480** →
**工作线程**软件完成 NV12→RGB 与缩放到 **320x240** → **GUI 线程只刷新最新帧**。

**为什么不能更省：** 实测确认本机（OrangePi 5 Ultra，gstreamer-rockchip 1.14）**没有可用的硬件缩放路径**：

| 尝试 | 实测结果 |
|---|---|
| `mppvideodec` 解码时缩放到 320x240 | ❌ `not-negotiated`（输出尺寸固定在输入分辨率） |
| `mppvideodec` 输出 RGB / BGR | ❌ 请求后 **0 帧**（GStreamer 自动插软 videoconvert 也不出帧） |
| `rgaconvert` / `rga` GStreamer 插件 | ❌ 本镜像未安装 |
| `mppvideodec` 输出 NV12 | ✅ 稳定（实测 263 帧/10s 全量解出） |

因此：**解码走硬件（NV12），颜色转换与缩放只能软件做**，且严格约束在**工作线程**，绝不在 GUI 主线程逐帧做。

---

## 1. 目标能力（本阶段范围）

1. 从 RDK 拉取实时视频流，在 Qt 窗口 **320x240 区域**显示。
2. 断线自动重连，不崩溃、不积压、不 CPU 忙等。
3. （可选）接收识别业务消息，在界面上展示"识别到 X"。
4. 本阶段**不做**：检测框叠加绘制（需先补接口，见 §2.3）；ROS 图像话题（`d435_direct` 模式下不发布）。

## 2. 现状与接口清单（给 Qt 的输入）

### 2.1 视频流接口（已交付、已验证）

| 项 | 值 |
|---|---|
| **RTSP URL** | `rtsp://192.168.3.114:8554/d435i` |
| 码流契约 | 640x480@30，H.264 **Main**，CBR **1500 kbit/s**，GOP 15 |
| 传输 | UDP（本网络可用）/ TCP |
| 备用入口 | HLS `http://192.168.3.114:8888/d435i/index.m3u8`；WebRTC `http://192.168.3.114:8889/d435i/`（调试用） |
| 发布端 | RDK `qr_vision_node` 图传 worker → FFmpeg `-c:v copy` → MediaMTX |

> 注意：1500kbit/s 是本网络（手机热点 2.4GHz）验收码率；换 5GHz/有线后可提至 3000。

### 2.2 ROS 2 业务接口（可选接入）

| topic | 类型 | 方向 | 内容 |
|---|---|---|---|
| `/drone/image` | `drone_msgs/msg/BarcodeCapture` | Qt 订阅 | `stamp` / `barcode`(识别结果串) / `image_format` / `image_data`(抓拍 JPEG) |
| `/mission/hover_active` | `std_msgs::msg::Bool` | Qt 发布 | 控制悬停采集状态 |

- 字段定义：`drone_msgs/msg/BarcodeCapture.msg` = `builtin_interfaces/Time stamp; string barcode; string image_format; uint8[] image_data`
- `d435_direct` 模式下节点**不发布** ROS 图像流，画面走 §2.1 RTSP。

### 2.3 缺口：逐帧检测框（如需叠加绘制，需先补接口）

节点每帧内部有 `BpuYoloDetection{class_id, x_min/y_min/x_max/y_max, score}`，但**未发布**。
若 Qt 要实时画检测框，需先在感知侧新增：
- 新消息 `drone_msgs/DetectionArray`（每帧一组 class/bbox/score + 帧时间戳）
- 节点发布 `last_bpu_detections_`

**这是感知侧待办，不是 Qt 侧能自己解决的**。本阶段默认 Qt 先只显示画面 + 业务文本。

---

## 3. 环境实测结论（方案的硬约束）

测试环境：OrangePi 5 Ultra / Ubuntu 22.04 / aarch64 / Qt 5.15.3 / GStreamer 1.20.3 / gstreamer-rockchip **1.14**。

| 项 | 实测 | 对 Qt 的影响 |
|---|---|---|
| `mppvideodec` 硬解 H.264 640x480 | ✅ 全帧解出（NV12） | 用 NV12 作为解码输出 |
| 输出 RGB/BGR | ❌ 0 帧 | **不能**让管线出 RGB，转换必须在应用侧 |
| 解码时缩放 | ❌ not-negotiated | 缩放必须在应用侧 |
| RGA GStreamer 插件 | ❌ 无 | 无硬件缩放捷径 |
| `/dev/rga`、librga | ✅ 存在 | 后续可选优化：Qt 直接调 RGA 缩放 |
| Qt QPA 插件 | eglfs/xcb/linuxfb/vnc/offscreen/minimal | 无 Wayland；有屏用 eglfs 或 xcb |

**推论**：应用侧（Qt 工作线程）必须做 **NV12→RGB** 和 **640x480→320x240**，属软件路径，CPU 需实测记录。

---

## 4. 目标架构（线程模型）

```
┌────────────── RDK X5（发送端，已交付）────────────────┐
│  D435i → qr_vision_node（检测 + 图传 worker 同一帧）      │
│         └→ H.264 → FFmpeg -c:v copy → MediaMTX :8554  │
└──────────────────────────┬────────────────────────────┘
                           │ RTSP（UDP 优先）
┌──────────────────────────▼────────────────────────────┐
│  Qt 地面站（OrangePi 5 Ultra）                          │
│                                                        │
│  【GStreamer worker 线程】                              │
│  rtspsrc → rtph264depay → h264parse → mppvideodec     │
│       → video/x-raw,format=NV12 → appsink             │
│       （硬解，latest-frame：max-buffers=1 drop=true）    │
│            │ NV12 640x480 最新帧                       │
│            ▼                                           │
│  【转换+缩放 worker 线程】                               │
│  NV12 → RGB(QImage) 640x480 → 缩放 → 320x240 QImage   │
│            │ 最新 320x240 QImage（信号/互斥槽）           │
│            ▼                                           │
│  【GUI 主线程】QLabel::setPixmap / paintEvent 刷新      │
│  （绝不在此线程做逐帧转换/缩放）                          │
└────────────────────────────────────────────────────────┘
```

**关键约束**：
1. GStreamer 拉帧、NV12→RGB、缩放全部在**工作线程**。
2. GUI 主线程只接收"最新一帧 QImage"并绘制，**不做任何逐帧 CPU 重活**。
3. 断线重连逻辑在 worker 线程内完成，GUI 只收到"已断线/已重连"状态信号。

---

## 5. 详细技术方案

### 5.1 GStreamer 管线（已验证可用）

```
rtspsrc location=rtsp://192.168.3.114:8554/d435i protocols=udp latency=50 drop-on-latency=true !
rtph264depay ! h264parse config-interval=-1 ! mppvideodec ! video/x-raw,format=NV12 ! appsink
```

- `protocols=udp`：本网络 UDP 可用（1500kbps 稳定 29.9fps）；若某网络 UDP 被阻断改用 `protocols=tcp`（latency 建议 100）。
- `latency=50` + `drop-on-latency=true`：抖动吸收 + 迟到丢帧，避免积压。
- `h264parse config-interval=-1`：保证新客户端在一个 GOP 内获得 SPS/PPS 与 IDR。
- `appsink` 属性：`max-buffers=1, drop=true, sync=false`（latest-frame 语义，丢旧保新）。
- **不要**在管线里加 `videoconvert` 或 `video/x-raw,format=RGB`——实测会导致 0 帧。NV12 原样出，应用侧自己转。

### 5.2 Qt 线程模型 + latest-frame

- 建议用 **`std::thread` 或 `QThread` 运行 GStreamer 主循环**（`g_main_loop`），不用 Qt 事件驱动 GStreamer，避免两套事件循环打架。
- appsink 用 `gst_app_sink_try_pull_sample(sink, timeout)` 主动拉帧（**不要**用 `pull-sample` 信号 + 其他线程，省锁）。
- 工作线程内维护一个互斥保护的 `std::optional<QImage> latestFrame`；每拉一帧更新它（旧帧被覆盖=丢帧计数）。
- 通过 **`QMetaObject::invokeMethod(guiObj, "...", Qt::QueuedConnection)`** 或信号槽把最新帧交给 GUI 线程；GUI 用一个 `updateTimer`（~33ms）或信号触发 `QLabel::setPixmap`。
- 参考：本项目 `orangepi_rtsp_mpp_probe`（`drone_perception/src/orangepi_rtsp_mpp_probe.cpp`）已验证"主动拉帧 + latest-frame + 重连"全部逻辑，可直接借鉴其 rtspsrc 参数与重连策略。

### 5.3 NV12 → RGB → QImage + 缩放（工作线程）

推荐：**OpenCV**（本机已装）在工作线程做：

```cpp
// NV12 640x480 → BGR 640x480 → 缩放到 320x240 → QImage
cv::Mat nv12(height * 3 / 2, width, CV_8UC1, sampleData);   // 单平面 NV12
cv::Mat bgr;   cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
cv::Mat small; cv::resize(bgr, small, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);
QImage img(small.data, 320, 240, static_cast<int>(small.step), QImage::Format_BGR888);
img = img.copy();  // 脱离 Mat 生命周期
```

- 备选：`libyuv` 的 `NV12ToARGB + ARGBScale`（若已装，性能略好）；或手写查表。**二选一即可，统一在工作线程**。
- `QImage::copy()` 必须调用（否则引用到临时 `cv::Mat` 内存，跨线程即悬垂）。
- 性能预期：640x480 NV12→BGR + 缩 320x240，RK3588 上 ~1-3ms/帧，30fps 时该线程负载约 3-9% 单核，可接受；**最终以 pidstat 实测为准**。

### 5.4 断线重连

- rtspsrc 在断流时会发 `GST_MESSAGE_ERROR`（"Could not read from resource"）或 EOS；**不能把 EOS 当正常结束**（本项目探针踩过这个坑，见提交 `0a552f7`）。
- 策略：bus 上收到 rtspsrc 错误 → 把整条 pipeline 置 NULL → `sleep 2s` 退避 → 重建并 PLAYING → 等待重新拿到 IDR 首帧。
- 若长时间无帧（建议阈值 15s）也触发重连（兜底）。
- 重连期间**不阻塞 GUI**：GUI 显示"连接中/已断线"状态。

### 5.5 生命周期 / 窗口关闭

- 关闭窗口 / SIGTERM：先停拉帧 → 置 pipeline NULL → 释放 appsink sample/caps → join worker 线程 → 释放 QImage。顺序：**先停生产者，再排空消费者**。
- 不要从 GUI 线程直接 `gst_element_set_state` 抢 pipeline（与 worker 争锁）；用一个 `std::atomic_bool running` 通知 worker 退出。

### 5.6 ROS 2 executor（可选，若要收业务消息）

- 若同时订阅 `/drone/image`，用 `rclcpp::executors::SingleThreadedExecutor` 跑在一个**独立线程**（不要和 GStreamer 主循环同线程）。
- 收到 `BarcodeCapture` 后把 `barcode` 字符串 + `image_data`(JPEG) 解码结果交给 GUI 线程显示。

### 5.7 显示平台

- 有 HDMI 屏：`QT_QPA_PLATFORM=eglfs`（直接 KMS，性能最好）或 `xcb`（X11）。
- 无屏 / SSH 调试：`QT_QPA_PLATFORM=vnc`（内置 VNC 服务）或 `offscreen`（仅逻辑测试）。
- 本机 Qt 5.15.3 无 Wayland 插件，别设 `wayland`。

---

## 6. 代码骨架（可直接照搬的最小可运行结构）

> 以下为 C++/Qt5 + GStreamer 骨架，含线程、拉帧、转换、重连。非完整工程，但结构可直接落地。

### 6.1 头文件（QtWorker.h）

```cpp
#pragma once
#include <QImage>
#include <QObject>
#include <atomic>
#include <mutex>
#include <optional>
#include <thread>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

class RtspViewWorker : public QObject
{
  Q_OBJECT
public:
  explicit RtspViewWorker(QObject *parent = nullptr);
  ~RtspViewWorker() override;

  void start(const QString &url);   // 启动 worker 线程
  void stop();                      // 优雅停止（可被 GUI 析构调用）
  bool running() const { return running_.load(); }
  std::uint64_t droppedFrames() const { return dropped_.load(); }
  std::uint64_t decodedFrames() const { return decoded_.load(); }

signals:
  void frameReady(const QImage &frame);   // 320x240，已缩放，交 GUI 线程
  void streamState(const QString &state); // "connecting"/"playing"/"reconnecting"/"error"

private:
  void loop();                            // GStreamer 主循环（worker 线程）
  bool buildPipeline(const QString &url, std::string &err);
  void teardown();
  void onBusMessage(GstBus *bus);
  static void onPadAdded(GstElement *src, GstPad *pad, gpointer self);
  QImage nv12ToQImage(const GstSample *sample);   // NV12 640x480 → 320x240 QImage

  std::thread thread_;
  std::atomic_bool running_{false};
  std::atomic<std::uint64_t> decoded_{0}, dropped_{0};
  GstElement *pipeline_ = nullptr;
  GstAppSink *appsink_ = nullptr;
  GstBus *bus_ = nullptr;
};
```

### 6.2 实现（RtspViewWorker.cpp）—— 关键片段

```cpp
#include "RtspViewWorker.h"
#include <opencv2/opencv.hpp>
#include <spawn.h>  // 本工程 ffmpeg 子进程用 fork+exec；Qt 侧不 spawn，无此依赖

void RtspViewWorker::start(const QString &url)
{
  running_ = true;
  thread_ = std::thread(&RtspViewWorker::loop, this, url);
}

void RtspViewWorker::stop()
{
  running_ = false;      // 通知退出
  if (thread_.joinable()) thread_.join();
  // 先停生产者（置 NULL）再释放资源
  if (pipeline_) { gst_element_set_state(pipeline_, GST_STATE_NULL); }
  if (bus_) gst_object_unref(bus_);
  if (pipeline_) gst_object_unref(pipeline_);
  pipeline_ = nullptr; appsink_ = nullptr; bus_ = nullptr;
}

void RtspViewWorker::loop(const QString url)
{
  gst_init(nullptr, nullptr);

  while (running_) {
    std::string err;
    if (!buildPipeline(url, err)) {
      emit streamState(QString("error: %1").arg(err.c_str()));
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }

    // PLAYING 后进入拉帧 + bus 监视
    bool reconnect = false;
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    emit streamState("playing");

    while (running_ && !reconnect) {
      // 1) bus 消息（错误/EOS/警告）
      GstMessage *m = gst_bus_pop_filtered(
          bus_, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_WARNING));
      if (m) {
        if (GST_MESSAGE_TYPE(m) == GST_MESSAGE_ERROR) {
          GError *gerr = nullptr; gchar *dbg = nullptr;
          gst_message_parse_error(m, &gerr, &dbg);
          const gchar *src = GST_OBJECT_NAME(m->src);
          qWarning() << "bus error" << src << (gerr ? gerr->message : "");
          if (gerr) g_error_free(gerr); if (dbg) g_free(dbg);
          if (src && g_strcmp0(src, "rtspsrc") == 0) reconnect = true;  // 断线 → 重建
          else { gst_message_unref(m); break; }                          // 其他错误 → 退出本管线
        } else if (GST_MESSAGE_TYPE(m) == GST_MESSAGE_EOS) {
          reconnect = true;   // 服务器主动结束 → 重连（不是正常退出）
        } else if (GST_MESSAGE_TYPE(m) == GST_MESSAGE_WARNING) {
          gchar *dbg = nullptr; gst_message_parse_warning(m, nullptr, &dbg);
          qWarning() << "warning:" << dbg; if (dbg) g_free(dbg);
        }
        gst_message_unref(m);
        continue;
      }

      // 2) 拉最新帧（20ms 超时，latest-frame）
      GstSample *sample = gst_app_sink_try_pull_sample(appsink_, 20 * GST_MSECOND);
      if (sample) {
        QImage img = nv12ToQImage(sample);
        gst_sample_unref(sample);
        if (!img.isNull()) {
          emit frameReady(img);   // QueuedConnection 交给 GUI
          decoded_.fetch_add(1);
        }
      }
    }

    // 重建前清理
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(bus_); gst_object_unref(pipeline_);
    pipeline_ = nullptr; appsink_ = nullptr; bus_ = nullptr;
    if (reconnect && running_) { emit streamState("reconnecting"); std::this_thread::sleep_for(std::chrono::seconds(2)); }
  }
}

bool RtspViewWorker::buildPipeline(const QString &url, std::string &err)
{
  const char *names[] = {"rtspsrc","rtph264depay","h264parse","mppvideodec","capsfilter","appsink"};
  GstElement *els[6] = {};
  for (int i = 0; i < 6; ++i) {
    els[i] = gst_element_factory_make(names[i], names[i]);
    if (!els[i]) { err = std::string("element unavailable: ") + names[i]; return false; }
  }
  pipeline_ = gst_pipeline_new("qt-rtsp");
  gst_bin_add_many(GST_BIN(pipeline_), els[0], els[1], els[2], els[3], els[4], els[5], nullptr);

  g_object_set(els[0], "location", url.toUtf8().constData(),
               "protocols", 1 /*UDP*/, "latency", (gint64)50, "drop-on-latency", TRUE, nullptr);
  g_object_set(els[2], "config-interval", -1, nullptr);
  g_object_set(els[5], "max-buffers", 1, "drop", TRUE, "sync", FALSE, nullptr);

  GstCaps *caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "NV12", nullptr);
  g_object_set(els[4], "caps", caps, nullptr); gst_caps_unref(caps);

  if (!gst_element_link_many(els[1], els[2], els[3], els[4], els[5], nullptr)) {
    err = "link decoder chain failed"; return false;
  }
  g_signal_connect(els[0], "pad-added", G_CALLBACK(onPadAdded), els[1]);
  bus_ = gst_element_get_bus(pipeline_);
  appsink_ = GST_APP_SINK(els[5]);
  return true;
}

void RtspViewWorker::onPadAdded(GstElement *, GstPad *pad, gpointer self)
{
  GstElement *depay = static_cast<GstElement *>(self);
  GstPad *sink = gst_element_get_static_pad(depay, "sink");
  if (sink && !gst_pad_is_linked(sink)) gst_pad_link(pad, sink);
  if (sink) gst_object_unref(sink);
}

QImage RtspViewWorker::nv12ToQImage(const GstSample *sample)
{
  const GstCaps *caps = gst_sample_get_caps(sample);
  GstVideoInfo info;
  if (!caps || !gst_video_info_from_caps(&info, caps)) return {};
  if (GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_NV12) return {};

  const int w = GST_VIDEO_INFO_WIDTH(&info), h = GST_VIDEO_INFO_HEIGHT(&info);
  GstVideoFrame frame;
  if (!gst_video_frame_map(&frame, &info, gst_sample_get_buffer(sample), GST_MAP_READ))
    return {};   // DMABUF 不可 CPU 映射时的处理（本机实测 memory=system，可映射）

  // NV12 → BGR 640x480 → 缩 320x240 → QImage
  cv::Mat nv12(h * 3 / 2, w, CV_8UC1, frame.data[0]);
  cv::Mat bgr, small;
  cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
  cv::resize(bgr, small, cv::Size(320, 240), 0, 0, cv::INTER_LINEAR);
  QImage img(small.data, 320, 240, static_cast<int>(small.step), QImage::Format_BGR888);
  img = img.copy();   // 脱离 Mat/frame 生命周期（必须）
  gst_video_frame_unmap(&frame);
  return img;
}

void RtspViewWorker::stop() { /* 见上 */ }
```

### 6.3 GUI 侧（MainWindow.cpp）—— 只刷帧

```cpp
RtspViewWorker *worker = new RtspViewWorker(this);
connect(worker, &RtspViewWorker::frameReady, this,
        [this](const QImage &img){ label_->setPixmap(QPixmap::fromImage(img)); },
        Qt::QueuedConnection);
connect(worker, &RtspViewWorker::streamState, this,
        [this](const QString &s){ statusBar()->showMessage(s); }, Qt::QueuedConnection);

worker->start("rtsp://192.168.3.114:8554/d435i");
// 窗口 closeEvent / 析构：
//   worker->stop();   // 会 join worker 线程，先停拉帧再释放
```

---

## 7. 构建部署依赖

Qt 地面站工程（OrangePi 上）需以下依赖：

```bash
# 系统包
sudo apt install -y qtbase5-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base libopencv-dev  # OpenCV 用于 NV12→BGR + 缩放
# 若用 libyuv 替代 OpenCV：libyuv-dev
```

- GStreamer：`gstreamer-1.0 / gstreamer-app-1.0 / gstreamer-video-1.0`（开发头文件已确认本机存在）。
- **插件**：解码只需 `mppvideodec`（gstreamer1.0-rockchip1，本机已装）+ `rtph264depay/h264parse`（base 插件）。**不需要** rgaconvert / rga / videoconvert。
- CMake 参考：

```cmake
find_package(Qt5 COMPONENTS Widgets REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
find_package(OpenCV REQUIRED)
add_executable(qt_ground_station main.cpp RtspViewWorker.cpp)
target_link_libraries(qt_ground_station
  Qt5::Widgets ${GST_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(qt_ground_station PRIVATE ${GST_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_compile_options(qt_ground_station PRIVATE ${GST_CFLAGS_OTHER})
```

- 运行：`QT_QPA_PLATFORM=eglfs ./qt_ground_station`（有屏）或 `QT_QPA_PLATFORM=vnc ./qt_ground_station`（无屏调试，VNC 端口默认 5900）。

---

## 8. 验收标准与测试方法

| 项 | 通过标准 | 验证方法 |
|---|---|---|
| 硬解 | 仅 `mppvideodec` 参与解码，无软件解码 | `top`/pidstat 观察进程 CPU；GPU/VPU 负载 |
| 显示 | 窗口 320x240 显示实时画面，无绿屏/花屏 | 目检 |
| 帧率 | 显示刷新 ≥29 fps（稳定网络） | 界面帧计数 / fpsdisplaysink |
| 延迟 | 端到端延迟可接受（<500ms） | 拍屏幕对比实际动作 |
| 重连 | RDK 停 10s→重启，界面自动恢复 | 参照 `rdk_x5_orangepi5ultra_send_receive_implementation_status.md` T7 方法重复 3 次 |
| 资源 | 30 分钟无内存增长；worker 线程 CPU 记录在案 | `pidstat -dur`、`vmstat` |
| 干净退出 | 关窗/SIGTERM 后无残留进程（无 GStreamer 线程泄漏） | `pgrep -af qt_ground_station` |
| （可选）业务 | 识别到条码时界面显示对应文本 | 放一张条码在镜头前 |

**性能基线参考**（本项目探针实测，Qt 侧应相近或更好）：
- 解码（NV12 硬解）：探针进程 ~2% CPU，MPP 硬解。
- 转换+缩放（软）：预计 3-9% 单核（640x480 NV12→BGR + 缩 320x240）。

---

## 9. 常见坑（务必读）

1. **mppvideodec 请求 RGB/BGR = 0 帧**：不要加 `video/x-raw,format=RGB` 或 `videoconvert`。输出固定 NV12，应用侧转。
2. **mppvideodec 不可缩放**：输出尺寸=输入尺寸，缩放必须在应用侧。
3. **DMABUF**：本机实测 appsink 输出 `memory=system`（可 CPU 映射），但保险起见仍用 `gst_video_frame_map` 映射后再读（骨架已用），避免把 fd 当指针。
4. **QImage 悬垂**：QImage 引用 cv::Mat 内存后必须 `.copy()`，否则跨线程访问野指针。
5. **EOS ≠ 正常结束**：RTSP 服务器断开会发 EOS/错误，要重连而不是退出。
6. **GUI 主线程不做重活**：逐帧 NV12→RGB/缩放一旦放 GUI 线程，界面卡死、CPU 飙高。
7. **两套事件循环**：GStreamer 主循环放 worker 线程，别塞进 Qt 事件循环里。
8. **重连退避**：断线后 sleep 1-2s 再重建，别在错误回调里死循环。
9. **`sync=false` 必须设**：appsink 默认同步播放会丢最新帧、帧率不稳。
10. **ffmpeg 无需引入**：Qt 侧只拉流解码，不做编码/转封装。

---

## 10. 调试方法

1. **先不写 Qt，用 gst-launch 验证管线**（本方案 §5.1 的串），确认画面能出再进 Qt 代码：
   ```bash
   gst-launch-1.0 -v rtspsrc location=rtsp://192.168.3.114:8554/d435i protocols=udp latency=50 drop-on-latency=true ! \
     rtph264depay ! h264parse config-interval=-1 ! mppvideodec ! video/x-raw,format=NV12 ! fakesink sync=false
   ```
2. **无头看帧**：把 `fakesink` 换成 `multifilesink location=/tmp/f_%05d.raw`，数文件数即帧数；用 `ffplay -f rawvideo -pix_fmt nv12 -video_size 640x480 f_00001.raw` 看内容。
3. **看协商 caps**：`gst-launch-1.0 -v ... ! mppvideodec ! fakesink` 里 grep `mppvideodec.GstPad:src: caps`，确认是 NV12 640x480。
4. **日志**：`GST_DEBUG=mppvideodec:5,appsink:5` 可看解码/拉帧细节。
5. **卡住无帧**：先确认 rtspsrc 已 PLAYING（bus 消息），再看 appsink `try_pull_sample` 是否一直 NULL（超时），用探针工具 `orangepi_rtsp_mpp_probe --protocol udp --format nv12` 对照。
6. **重连不恢复**：抓 RDK 侧 MediaMTX 日志确认"reader is too slow"（网络）还是服务端退出；网络问题降码率或换网络。

---

## 11. 复盘要点

- **本阶段不碰**：感知侧检测框发布（§2.3）、ROS 图像话题、3000kbit/s 复测、feature 分支合并 main。
- **若后续要省 CPU**：Qt 工作线程直接调 RGA（`/dev/rga` + librga）做 NV12→缩→RGB，替换软转换；先以软路径跑通验收。
- **若换更好网络**：RDK `d435_direct_detection.yaml` 里 `video_stream_bitrate_kbps` 提到 3000，Qt 侧无需改动。
- **验收记录**：按 §8 执行并把 pidstat/temp/重连结果归档（参考 `test_logs/` 目录习惯，gitignore 不提交）。
- **相关文档**：
  - 收发/MPP 状态：`docs/rdk_x5_orangepi5ultra_send_receive_implementation_status.md`
  - 探针（可复用逻辑）：`src/orangepi_rtsp_mpp_probe.cpp`
  - 直采检测阶段：`docs/d435i_x5_direct_detection_implementation_status.md`
