#include "drone_perception/d435_color_capture.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <utility>

namespace drone_perception
{

struct D435ColorCapture::Impl
{
  std::mutex mutex;
  std::condition_variable frame_cv;
  D435ColorFrame latest_frame;
  bool frame_pending{false};
  std::string last_error;

  std::atomic<std::uint64_t> captured_count{0};
  std::atomic<std::uint64_t> dropped_count{0};
  std::atomic<std::uint64_t> timeout_count{0};
  std::atomic<std::uint64_t> reconnect_count{0};
  std::uint64_t next_sequence{1};

  // Owned by the capture thread only; other threads must not touch it. Kept in
  // the shared Impl so a detached capture thread can still release it safely.
  std::unique_ptr<rs2::pipeline> pipeline;

  std::atomic_bool running{false};
  std::atomic_bool exited{false};
};

D435ColorCapture::D435ColorCapture(Config config)
    : config_(std::move(config)), impl_(std::make_shared<Impl>())
{
}

D435ColorCapture::~D435ColorCapture()
{
  stop();
}

std::string D435ColorCapture::detectSingleDeviceSerial()
{
  try {
    rs2::context context;
    rs2::device_list devices = context.query_devices();

    if (devices.size() != 1U) {
      return {};
    }

    const char *serial =
        devices[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    return serial == nullptr ? std::string{} : std::string(serial);
  } catch (const rs2::error &) {
    return {};
  } catch (const std::exception &) {
    return {};
  }
}

bool D435ColorCapture::start()
{
  if (impl_->running.exchange(true)) {
    return true;
  }

  setError(*impl_, {});

  if (!openPipeline(*impl_, config_)) {
    impl_->running = false;
    return false;
  }

  capture_thread_ = std::thread(&D435ColorCapture::captureLoop, this, impl_);
  return true;
}

void D435ColorCapture::stop()
{
  if (!impl_) {
    return;
  }

  impl_->running = false;
  impl_->frame_cv.notify_all();

  if (!capture_thread_.joinable()) {
    return;
  }

  // Bounded wait for the capture thread. It may be blocked inside a blocking
  // SDK call (pipeline start/stop) on a wedged device; detach in that case so
  // node shutdown never hangs. The thread keeps Impl alive via its own
  // shared_ptr copy, so a detached thread still cleans up safely.
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(config_.wait_timeout_ms + 1000);

  while (!impl_->exited.load()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      std::fprintf(
          stderr,
          "[D435ColorCapture] capture thread did not exit within %d ms; "
          "detaching (device may be wedged in an SDK call)\n",
          config_.wait_timeout_ms + 1000);
      capture_thread_.detach();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  capture_thread_.join();
}

bool D435ColorCapture::running() const
{
  return impl_->running.load();
}

std::string D435ColorCapture::lastError() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->last_error;
}

std::uint64_t D435ColorCapture::capturedCount() const
{
  return impl_->captured_count.load();
}

std::uint64_t D435ColorCapture::droppedCount() const
{
  return impl_->dropped_count.load();
}

std::uint64_t D435ColorCapture::timeoutCount() const
{
  return impl_->timeout_count.load();
}

std::uint64_t D435ColorCapture::reconnectCount() const
{
  return impl_->reconnect_count.load();
}

void D435ColorCapture::setError(Impl &impl, const std::string &message)
{
  std::lock_guard<std::mutex> lock(impl.mutex);
  impl.last_error = message;
}

bool D435ColorCapture::openPipeline(Impl &impl, const Config &config)
{
  try {
    // Fail fast on a missing device. pipeline::start() with enable_device()
    // blocks waiting for the requested device to appear, so verify presence
    // here to turn "no camera / wrong serial" into an immediate error instead
    // of an unbounded hang.
    {
      rs2::context context;
      const rs2::device_list devices = context.query_devices();

      if (config.serial.empty()) {
        if (devices.size() == 0U) {
          setError(impl, "open pipeline: no RealSense device present");
          return false;
        }
      } else {
        bool found = false;

        for (std::size_t i = 0U; i < devices.size(); ++i) {
          const char *serial =
              devices[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);

          if (serial != nullptr && config.serial == serial) {
            found = true;
            break;
          }
        }

        if (!found) {
          setError(
              impl,
              "open pipeline: device with serial " + config.serial +
                  " is not present");
          return false;
        }
      }
    }

    rs2::config config_rs;
    config_rs.disable_all_streams();

    if (!config.serial.empty()) {
      config_rs.enable_device(config.serial);
    }

    config_rs.enable_stream(
        RS2_STREAM_COLOR,
        config.width,
        config.height,
        RS2_FORMAT_YUYV,
        config.fps);

    impl.pipeline = std::make_unique<rs2::pipeline>();
    impl.pipeline->start(config_rs);

    setError(impl, {});
    return true;
  } catch (const rs2::error &e) {
    setError(impl, std::string("open pipeline: ") + e.what());
    impl.pipeline.reset();
    return false;
  } catch (const std::exception &e) {
    setError(impl, std::string("open pipeline: ") + e.what());
    impl.pipeline.reset();
    return false;
  }
}

void D435ColorCapture::closePipeline(Impl &impl)
{
  try {
    if (impl.pipeline) {
      impl.pipeline->stop();
    }
  } catch (const rs2::error &) {
    // Best-effort stop; the device may already be gone.
  } catch (const std::exception &) {
    // Best-effort stop; never let teardown propagate.
  }
  impl.pipeline.reset();
}

void D435ColorCapture::captureLoop(std::shared_ptr<Impl> impl)
{
  // Local copies owned by this thread so a detached thread never touches the
  // (potentially destroyed) class members.
  const Config config = config_;

  while (impl->running.load()) {
    if (!impl->pipeline) {
      ++impl->reconnect_count;

      std::this_thread::sleep_for(
          std::chrono::milliseconds(config.reconnect_delay_ms));

      if (!impl->running.load()) {
        break;
      }

      if (!openPipeline(*impl, config)) {
        continue;
      }
    }

    // try_wait_for_frames distinguishes a benign timeout (returns false, no
    // exception) from a device error (throws), unlike wait_for_frames whose
    // timeout message carries no stable marker.
    rs2::frameset frames;
    bool got_frame = false;

    try {
      got_frame = impl->pipeline->try_wait_for_frames(
          &frames,
          static_cast<unsigned int>(config.wait_timeout_ms));
    } catch (const rs2::error &e) {
      setError(*impl, std::string("wait_for_frames: ") + e.what());
      closePipeline(*impl);
      continue;
    } catch (const std::exception &e) {
      setError(*impl, std::string("wait_for_frames: ") + e.what());
      closePipeline(*impl);
      continue;
    }

    if (!got_frame) {
      ++impl->timeout_count;
      continue;
    }

    if (!frames) {
      continue;
    }

    rs2::video_frame color = frames.first_or_default(RS2_STREAM_COLOR);

    if (!color) {
      continue;
    }

    const int width = color.get_width();
    const int height = color.get_height();
    const int stride = color.get_stride_in_bytes();
    const int row_bytes = std::min<int>(stride, width * 2);

    if (row_bytes <= 0 || height <= 0) {
      continue;
    }

    const std::uint8_t *data =
        static_cast<const std::uint8_t *>(color.get_data());

    {
      std::lock_guard<std::mutex> lock(impl->mutex);

      if (impl->frame_pending) {
        ++impl->dropped_count;
      }

      D435ColorFrame frame;
      frame.sequence = impl->next_sequence++;
      frame.device_timestamp_ms = color.get_timestamp();
      frame.received_at = std::chrono::steady_clock::now();
      frame.width = width;
      frame.height = height;
      frame.stride_bytes = row_bytes;
      frame.yuyv.resize(
          static_cast<std::size_t>(row_bytes) *
          static_cast<std::size_t>(height));

      for (int row = 0; row < height; ++row) {
        std::memcpy(
            frame.yuyv.data() +
                static_cast<std::size_t>(row) * static_cast<std::size_t>(row_bytes),
            data +
                static_cast<std::size_t>(row) * static_cast<std::size_t>(stride),
            static_cast<std::size_t>(row_bytes));
      }

      impl->latest_frame = std::move(frame);
      impl->frame_pending = true;
      ++impl->captured_count;
    }

    impl->frame_cv.notify_all();
  }

  closePipeline(*impl);
  impl->exited = true;
}

bool D435ColorCapture::waitForLatest(
    D435ColorFrame &frame,
    std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(impl_->mutex);
  const bool signaled = impl_->frame_cv.wait_for(lock, timeout, [this]() {
    return impl_->frame_pending || !impl_->running.load();
  });

  if (!signaled || !impl_->frame_pending) {
    return false;
  }

  frame = impl_->latest_frame;
  impl_->frame_pending = false;
  return true;
}

}  // namespace drone_perception
