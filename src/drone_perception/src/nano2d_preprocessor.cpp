#include "drone_perception/nano2d_preprocessor.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include <GC820/nano2D_util.h>

namespace drone_perception
{

namespace
{
constexpr std::uint8_t kLetterboxY = 114;
constexpr std::uint8_t kLetterboxUv = 128;
}  // namespace

Nano2DPreprocessor::Nano2DPreprocessor() = default;

Nano2DPreprocessor::~Nano2DPreprocessor()
{
  shutdown();
}

bool Nano2DPreprocessor::initialize(
    int source_width,
    int source_height,
    int model_width,
    int model_height)
{
  shutdown();

  if (source_width <= 0 || source_height <= 0 ||
      model_width <= 0 || model_height <= 0) {
    setError("invalid dimensions");
    return false;
  }

  // Stage B is fixed to a scale-1.0 letterbox: content fills the model width
  // and is centered vertically. pad_y must be even so NV12 chroma rows align.
  const int pad_x = (model_width - source_width) / 2;
  const int pad_y = (model_height - source_height) / 2;

  if (source_width != model_width ||
      source_height > model_height ||
      (model_height - source_height) % 2 != 0) {
    setError(
        "stage B only supports scale=1.0 letterbox "
        "(e.g. source 640x480 -> model 640x640)");
    return false;
  }

  source_width_ = source_width;
  source_height_ = source_height;
  model_width_ = model_width;
  model_height_ = model_height;
  pad_x_ = pad_x;
  pad_y_ = pad_y;
  scale_ = 1.0F;

  if (!allocateBuffers()) {
    shutdown();
    return false;
  }

  if (!fillLetterboxBackground()) {
    shutdown();
    return false;
  }

  contiguous_nv12_.resize(
      static_cast<std::size_t>(model_width) * model_height * 3 / 2);

  if (contiguous_nv12_.empty()) {
    setError("failed to allocate contiguous NV12 output");
    shutdown();
    return false;
  }

  initialized_ = true;
  return true;
}

bool Nano2DPreprocessor::allocateBuffers()
{
  if (n2d_open() != N2D_SUCCESS) {
    setError("n2d_open failed");
    return false;
  }
  n2d_opened_ = true;

  if (n2d_switch_device(N2D_DEVICE_0) != N2D_SUCCESS) {
    setError("n2d_switch_device(N2D_DEVICE_0) failed");
    return false;
  }

  if (n2d_switch_core(N2D_CORE_0) != N2D_SUCCESS) {
    setError("n2d_switch_core(N2D_CORE_0) failed");
    return false;
  }

  if (n2d_util_allocate_buffer(
          static_cast<n2d_uint32_t>(source_width_),
          static_cast<n2d_uint32_t>(source_height_),
          N2D_YUYV,
          N2D_0,
          N2D_LINEAR,
          N2D_TSC_DISABLE,
          &yuyv_buffer_) != N2D_SUCCESS) {
    setError("allocate YUYV buffer failed");
    return false;
  }

  if (n2d_util_allocate_buffer(
          static_cast<n2d_uint32_t>(model_width_),
          static_cast<n2d_uint32_t>(model_height_),
          N2D_NV12,
          N2D_0,
          N2D_LINEAR,
          N2D_TSC_DISABLE,
          &nv12_buffer_) != N2D_SUCCESS) {
    setError("allocate NV12 buffer failed");
    return false;
  }

  return true;
}

bool Nano2DPreprocessor::fillLetterboxBackground()
{
  uint8_t *y = static_cast<uint8_t *>(nv12_buffer_.memory);
  uint8_t *uv = static_cast<uint8_t *>(nv12_buffer_.uv_memory[0]);

  if (y == nullptr || uv == nullptr) {
    setError("NV12 buffer memory not mapped");
    return false;
  }

  const int y_stride = nv12_buffer_.stride;
  const int uv_stride = nv12_buffer_.uvstride[0];

  // Y plane padding rows outside [pad_y, pad_y + source_height).
  for (int r = 0; r < pad_y_; ++r) {
    std::memset(
        y + static_cast<std::size_t>(r) * y_stride,
        kLetterboxY,
        static_cast<std::size_t>(model_width_));
  }
  for (int r = pad_y_ + source_height_; r < model_height_; ++r) {
    std::memset(
        y + static_cast<std::size_t>(r) * y_stride,
        kLetterboxY,
        static_cast<std::size_t>(model_width_));
  }

  // UV plane has model_height/2 rows; content occupies rows
  // [pad_y/2, pad_y/2 + source_height/2).
  const int uv_top_pad = pad_y_ / 2;
  const int uv_bottom_start = uv_top_pad + source_height_ / 2;

  for (int r = 0; r < uv_top_pad; ++r) {
    std::memset(
        uv + static_cast<std::size_t>(r) * uv_stride,
        kLetterboxUv,
        static_cast<std::size_t>(model_width_));
  }
  for (int r = uv_bottom_start; r < model_height_ / 2; ++r) {
    std::memset(
        uv + static_cast<std::size_t>(r) * uv_stride,
        kLetterboxUv,
        static_cast<std::size_t>(model_width_));
  }

  return true;
}

bool Nano2DPreprocessor::convertYuyvToLetterboxNv12(
    const std::uint8_t *yuyv,
    int source_stride,
    Nv12FrameView &output)
{
  output = {};

  if (!initialized_ || yuyv == nullptr) {
    setError("Nano2D preprocessor not initialized");
    return false;
  }

  uint8_t *input = static_cast<uint8_t *>(yuyv_buffer_.memory);
  if (input == nullptr) {
    setError("YUYV buffer memory not mapped");
    return false;
  }

  const int input_stride = yuyv_buffer_.stride;
  const int row_bytes = std::min<int>(source_stride, source_width_ * 2);

  if (row_bytes <= 0) {
    setError("invalid YUYV source stride");
    return false;
  }

  for (int r = 0; r < source_height_; ++r) {
    std::memcpy(
        input + static_cast<std::size_t>(r) * input_stride,
        yuyv + static_cast<std::size_t>(r) * source_stride,
        static_cast<std::size_t>(row_bytes));
  }

  // Hardware color-space conversion + letterbox blit (synchronous commit so
  // the BPU never reads a partially converted frame).
  n2d_rectangle_t src_rect{0, 0, source_width_, source_height_};
  n2d_rectangle_t dst_rect{pad_x_, pad_y_, source_width_, source_height_};

  if (n2d_blit(&nv12_buffer_, &dst_rect, &yuyv_buffer_, &src_rect, N2D_BLEND_NONE) !=
      N2D_SUCCESS) {
    setError("n2d_blit failed");
    return false;
  }

  if (n2d_commit_ex(N2D_TRUE) != N2D_SUCCESS) {
    setError("n2d_commit_ex(N2D_TRUE) failed");
    return false;
  }

  // Gather the two NV12 planes into one contiguous buffer for BPU. Nano2D may
  // lay out the Y and UV planes at separate addresses with their own strides,
  // while inferNv12() requires a single 614400-byte (W*H*3/2) buffer.
  const uint8_t *y_plane = static_cast<const uint8_t *>(nv12_buffer_.memory);
  const uint8_t *uv_plane = static_cast<const uint8_t *>(nv12_buffer_.uv_memory[0]);
  const int y_stride = nv12_buffer_.stride;
  const int uv_stride = nv12_buffer_.uvstride[0];

  if (y_plane == nullptr || uv_plane == nullptr) {
    setError("NV12 output buffer memory not mapped");
    return false;
  }

  uint8_t *out = contiguous_nv12_.data();

  for (int r = 0; r < model_height_; ++r) {
    std::memcpy(
        out + static_cast<std::size_t>(r) * model_width_,
        y_plane + static_cast<std::size_t>(r) * y_stride,
        static_cast<std::size_t>(model_width_));
  }

  uint8_t *uv_out = out + static_cast<std::size_t>(model_width_) * model_height_;
  for (int r = 0; r < model_height_ / 2; ++r) {
    std::memcpy(
        uv_out + static_cast<std::size_t>(r) * model_width_,
        uv_plane + static_cast<std::size_t>(r) * uv_stride,
        static_cast<std::size_t>(model_width_));
  }

  output.data = contiguous_nv12_.data();
  output.size = contiguous_nv12_.size();
  output.width = model_width_;
  output.height = model_height_;
  output.stride = model_width_;
  return true;
}

void Nano2DPreprocessor::shutdown()
{
  initialized_ = false;
  contiguous_nv12_.clear();

  // Free in reverse allocation order (NV12 then YUYV), guarding partial
  // allocation failures. last_error_ is preserved for the caller.
  if (nv12_buffer_.handle != 0U) {
    n2d_free(&nv12_buffer_);
  }
  nv12_buffer_ = {};

  if (yuyv_buffer_.handle != 0U) {
    n2d_free(&yuyv_buffer_);
  }
  yuyv_buffer_ = {};

  if (n2d_opened_) {
    n2d_close();
    n2d_opened_ = false;
  }
}

std::string Nano2DPreprocessor::lastError() const
{
  return last_error_;
}

Nano2DLetterboxState Nano2DPreprocessor::letterboxState() const
{
  return {scale_, pad_x_, pad_y_};
}

void Nano2DPreprocessor::setError(const std::string &message)
{
  last_error_ = message;
}

}  // namespace drone_perception
