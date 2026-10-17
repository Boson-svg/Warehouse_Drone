#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <GC820/nano2D.h>

namespace drone_perception
{

// Read-only view of a contiguous NV12 buffer ready for BPU consumption.
// The backing storage is owned by the preprocessor and is only valid until the
// next convertYuyvToLetterboxNv12() call.
struct Nv12FrameView
{
  const std::uint8_t *data{nullptr};
  std::size_t size{0};
  int width{0};
  int height{0};
  int stride{0};
};

// Letterbox geometry applied when mapping model (letterbox) coordinates back
// to the original image coordinates.
struct Nano2DLetterboxState
{
  float scale{1.0F};
  int pad_x{0};
  int pad_y{0};
};

// Hardware (GC820 Nano2D) YUYV -> NV12 letterbox preprocessor. Converts a
// YUYV source into a scale-1.0 letterboxed NV12 image sized to the BPU model
// input (e.g. 640x480 -> 640x640 with pad_y=80) and returns a contiguous
// buffer ready for BpuYoloDetector::inferNv12().
//
// Threading: initialize() runs on the node constructor; convert*() must be
// called from the vision worker thread only. No internal locking is performed.
class Nano2DPreprocessor
{
public:
  Nano2DPreprocessor();
  ~Nano2DPreprocessor();

  Nano2DPreprocessor(const Nano2DPreprocessor &) = delete;
  Nano2DPreprocessor &operator=(const Nano2DPreprocessor &) = delete;

  bool initialize(int source_width, int source_height,
                  int model_width, int model_height);
  bool convertYuyvToLetterboxNv12(
      const std::uint8_t *yuyv,
      int source_stride,
      Nv12FrameView &output);
  void shutdown();
  std::string lastError() const;
  Nano2DLetterboxState letterboxState() const;

private:
  bool allocateBuffers();
  bool fillLetterboxBackground();
  void setError(const std::string &message);

  int source_width_{0};
  int source_height_{0};
  int model_width_{0};
  int model_height_{0};
  int pad_x_{0};
  int pad_y_{0};
  float scale_{1.0F};

  n2d_buffer_t yuyv_buffer_{};
  n2d_buffer_t nv12_buffer_{};
  bool n2d_opened_{false};
  bool initialized_{false};

  std::vector<std::uint8_t> contiguous_nv12_;
  std::string last_error_;
};

}  // namespace drone_perception
