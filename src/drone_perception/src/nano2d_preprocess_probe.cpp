// nano2d_preprocess_probe: standalone validation of the Nano2D YUYV -> NV12
// letterbox preprocessor on the RDK X5.
//
// Usage:
//   nano2d_preprocess_probe [input.yuv] [output.nv12]
//
// input.yuv  : raw YUYV 640x480 (614400 bytes). If omitted a solid-color
//              pattern (Y=200 U=100 V=150) is generated.
// output.nv12: optional path to dump the contiguous 640x640 NV12 result.
//
// Prints the letterbox geometry and samples pixels in the content region and
// both padding bands, so cache/stride/layout problems (purple bands, wrong
// colors, shifted padding) surface immediately.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "drone_perception/nano2d_preprocessor.hpp"

namespace
{
constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr int kModelSize = 640;
constexpr int kYuyvStride = kWidth * 2;
}  // namespace

int main(int argc, char **argv)
{
  const char *input_path = argc > 1 ? argv[1] : nullptr;
  const char *output_path = argc > 2 ? argv[2] : nullptr;

  std::vector<std::uint8_t> yuyv(
      static_cast<std::size_t>(kWidth) * kHeight * 2);

  if (input_path != nullptr) {
    FILE *file = std::fopen(input_path, "rb");
    if (file == nullptr) {
      std::fprintf(stderr, "cannot open %s\n", input_path);
      return 1;
    }
    const std::size_t read_count =
        std::fread(yuyv.data(), 1, yuyv.size(), file);
    std::fclose(file);
    if (read_count != yuyv.size()) {
      std::fprintf(stderr, "short read: %zu / %zu bytes\n", read_count, yuyv.size());
      return 1;
    }
    std::printf("loaded %s (%zu bytes)\n", input_path, read_count);
  } else {
    for (int row = 0; row < kHeight; ++row) {
      std::uint8_t *line = yuyv.data() + row * kYuyvStride;
      for (int x = 0; x < kWidth; x += 2) {
        line[x * 2 + 0] = 200;  // Y0
        line[x * 2 + 1] = 100;  // U
        line[x * 2 + 2] = 200;  // Y1
        line[x * 2 + 3] = 150;  // V
      }
    }
    std::printf("using generated solid-color YUYV (Y=200 U=100 V=150)\n");
  }

  drone_perception::Nano2DPreprocessor preprocessor;
  if (!preprocessor.initialize(kWidth, kHeight, kModelSize, kModelSize)) {
    std::fprintf(
        stderr,
        "initialize failed: %s\n",
        preprocessor.lastError().c_str());
    return 1;
  }

  const drone_perception::Nano2DLetterboxState letterbox =
      preprocessor.letterboxState();
  std::printf(
      "letterbox: scale=%.1f pad_x=%d pad_y=%d\n",
      letterbox.scale,
      letterbox.pad_x,
      letterbox.pad_y);

  drone_perception::Nv12FrameView view;
  if (!preprocessor.convertYuyvToLetterboxNv12(yuyv.data(), kYuyvStride, view)) {
    std::fprintf(
        stderr,
        "convert failed: %s\n",
        preprocessor.lastError().c_str());
    return 1;
  }

  std::printf(
      "output: %dx%d stride=%d size=%zu\n",
      view.width,
      view.height,
      view.stride,
      view.size);

  const auto sample = [&view](int x, int y) {
    const std::size_t y_offset =
        static_cast<std::size_t>(y) * view.stride + x;
    const std::size_t uv_offset =
        static_cast<std::size_t>(view.height) * view.stride +
        static_cast<std::size_t>(y / 2) * view.stride +
        static_cast<std::size_t>(x & ~1);
    std::string text;
    char buffer[64];
    std::snprintf(
        buffer,
        sizeof(buffer),
        "Y=%d U=%d V=%d",
        static_cast<int>(view.data[y_offset]),
        static_cast<int>(view.data[uv_offset]),
        static_cast<int>(view.data[uv_offset + 1]));
    text = buffer;
    return text;
  };

  std::printf("content(320,240):   %s (expect Y=200 U=100 V=150)\n",
              sample(320, 240).c_str());
  std::printf("content(320,560):   %s (expect Y=200 U=100 V=150)\n",
              sample(320, 560).c_str());
  std::printf("padding-top(320,40):   %s (expect Y=114 U=128 V=128)\n",
              sample(320, 40).c_str());
  std::printf("padding-bottom(320,620): %s (expect Y=114 U=128 V=128)\n",
              sample(320, 620).c_str());

  if (output_path != nullptr) {
    FILE *file = std::fopen(output_path, "wb");
    if (file == nullptr) {
      std::fprintf(stderr, "cannot write %s\n", output_path);
      return 1;
    }
    std::fwrite(view.data, 1, view.size, file);
    std::fclose(file);
    std::printf("saved %zu bytes to %s\n", view.size, output_path);
  }

  preprocessor.shutdown();
  return 0;
}
