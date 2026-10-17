#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <librealsense2/rs.hpp>

namespace drone_perception
{

// Latest captured D435i color frame (YUYV). The SDK frame is copied into this
// self-contained buffer on the capture thread so no rs2::video_frame is ever
// held across threads.
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

// Direct color capture from the D435i via librealsense, bypassing the ROS
// realsense2_camera_node. Runs its own capture thread and keeps a single
// latest-frame slot: a new frame overwrites any unread frame, so consumers
// always see the newest frame and never accumulate backlog.
//
// Shutdown contract: stop() waits a bounded time for the capture thread, then
// detaches it if it is stuck inside a blocking SDK call (device wedge). All
// mutable state and the rs2::pipeline live in a shared Impl that the capture
// thread owns by shared_ptr, so a detached thread never touches freed memory.
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

  D435ColorCapture(const D435ColorCapture &) = delete;
  D435ColorCapture &operator=(const D435ColorCapture &) = delete;

  // Resolves the D435i serial when Config::serial is empty. Picks the single
  // device only when exactly one RealSense device is present; otherwise returns
  // an empty string so the caller can refuse to auto-select.
  static std::string detectSingleDeviceSerial();

  bool start();
  void stop();
  bool waitForLatest(D435ColorFrame &frame, std::chrono::milliseconds timeout);
  bool running() const;
  std::string lastError() const;

  std::uint64_t capturedCount() const;
  std::uint64_t droppedCount() const;
  std::uint64_t timeoutCount() const;
  std::uint64_t reconnectCount() const;

private:
  struct Impl;
  void captureLoop(std::shared_ptr<Impl> impl);
  bool openPipeline(Impl &impl, const Config &config);
  void closePipeline(Impl &impl);
  void setError(Impl &impl, const std::string &message);

  Config config_;
  std::shared_ptr<Impl> impl_;
  std::thread capture_thread_;
};

}  // namespace drone_perception
