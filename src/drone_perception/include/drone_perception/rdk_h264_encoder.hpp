#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace drone_perception
{

// RDK X5 hardware H.264 encoder (hb_media_codec) fed from caller-supplied YUYV
// pixels, not from a camera it opens itself. This is the encoding module reused
// by the qr_vision_node video-stream worker so that detection and encoding
// consume the same D435ColorFrame (single camera opener).
//
// Input:  YUYV frame (stride_bytes per row, width*2 bytes per pixel).
// Output: Annex-B H.264 appended to the caller's vector. The caller owns the
// stream (e.g. writes it to an FFmpeg -c:v copy pipe toward MediaMTX).
//
// The class is pimpl-based so this header stays portable; hb_media_codec is
// only referenced from the .cpp which is built on RDK targets.
class RdkH264Encoder
{
public:
  struct Config
  {
    int width{640};
    int height{480};
    int fps{30};
    int bitrate_kbps{1500};
    int gop{15};
  };

  RdkH264Encoder();
  ~RdkH264Encoder();

  RdkH264Encoder(const RdkH264Encoder &) = delete;
  RdkH264Encoder & operator=(const RdkH264Encoder &) = delete;

  // Initializes the hb_media_codec context and the YUYV->NV12 swscale context.
  // Returns false and fills error on failure.
  bool initialize(const Config & config, std::string & error);

  // Encodes one YUYV frame and appends the resulting H.264 NAL data to out.
  // pts is carried into the encoder's user pts. Returns false on error.
  bool encodeFrame(
    const std::uint8_t * yuyv,
    int stride_bytes,
    std::uint64_t pts,
    std::vector<std::uint8_t> & out,
    std::string & error);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace drone_perception
