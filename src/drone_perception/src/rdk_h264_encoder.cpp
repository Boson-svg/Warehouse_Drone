#include "drone_perception/rdk_h264_encoder.hpp"

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

extern "C" {
#include <hb_media_codec.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>
}

namespace drone_perception
{

struct RdkH264Encoder::Impl
{
  Config config;
  media_codec_context_t context{};
  SwsContext * converter{nullptr};
  bool initialized{false};
  bool started{false};
};

RdkH264Encoder::RdkH264Encoder() : impl_(std::make_unique<Impl>())
{
}

RdkH264Encoder::~RdkH264Encoder()
{
  if (impl_->started) {
    hb_mm_mc_pause(&impl_->context);
  }
  if (impl_->initialized) {
    hb_mm_mc_release(&impl_->context);
  }
  if (impl_->converter != nullptr) {
    sws_freeContext(impl_->converter);
    impl_->converter = nullptr;
  }
}

bool RdkH264Encoder::initialize(const Config & config, std::string & error)
{
  impl_->config = config;

  std::memset(&impl_->context, 0, sizeof(impl_->context));
  impl_->context.encoder = true;
  impl_->context.codec_id = MEDIA_CODEC_ID_H264;

  mc_video_codec_enc_params_t & parameters = impl_->context.video_enc_params;
  parameters.width = static_cast<uint32_t>(config.width);
  parameters.height = static_cast<uint32_t>(config.height);
  parameters.pix_fmt = MC_PIXEL_FORMAT_NV12;
  parameters.frame_buf_count = 3;
  parameters.external_frame_buf = false;
  parameters.bitstream_buf_count = 3;
  parameters.bitstream_buf_size = static_cast<uint32_t>(
      (config.width * config.height * 3 / 2 + 0x3ff) & ~0x3ff);
  parameters.gop_params.gop_preset_idx = 9;
  parameters.rot_degree = MC_CCW_0;
  parameters.mir_direction = MC_DIRECTION_NONE;
  parameters.frame_cropping_flag = false;
  parameters.enable_user_pts = 1;
  parameters.rc_params.mode = MC_AV_RC_MODE_H264CBR;

  int result = hb_mm_mc_get_rate_control_config(&impl_->context, &parameters.rc_params);
  if (result != 0) {
    error = "hb_mm_mc_get_rate_control_config failed, result=" +
        std::to_string(result);
    return false;
  }

  mc_h264_cbr_params_t & rate = parameters.rc_params.h264_cbr_params;
  rate.intra_period = static_cast<uint32_t>(config.gop);
  rate.intra_qp = 30;
  rate.bit_rate = static_cast<uint32_t>(config.bitrate_kbps);
  rate.frame_rate = static_cast<uint32_t>(config.fps);
  rate.initial_rc_qp = 20;
  rate.vbv_buffer_size = 20;
  rate.mb_level_rc_enalbe = 1;
  rate.min_qp_I = 8;
  rate.max_qp_I = 50;
  rate.min_qp_P = 8;
  rate.max_qp_P = 50;
  rate.min_qp_B = 8;
  rate.max_qp_B = 50;
  rate.hvs_qp_enable = 1;
  rate.hvs_qp_scale = 2;
  rate.max_delta_qp = 10;
  rate.qp_map_enable = 0;
  parameters.h264_enc_config.h264_profile = MC_H264_PROFILE_MP;
  parameters.h264_enc_config.h264_level = MC_H264_LEVEL4;

  result = hb_mm_mc_initialize(&impl_->context);
  if (result != 0) {
    error = "hb_mm_mc_initialize failed, result=" + std::to_string(result);
    return false;
  }
  impl_->initialized = true;

  result = hb_mm_mc_configure(&impl_->context);
  if (result != 0) {
    error = "hb_mm_mc_configure failed, result=" + std::to_string(result);
    return false;
  }

  mc_av_codec_startup_params_t startup_parameters{};
  result = hb_mm_mc_start(&impl_->context, &startup_parameters);
  if (result != 0) {
    error = "hb_mm_mc_start failed, result=" + std::to_string(result);
    return false;
  }
  impl_->started = true;

  impl_->converter = sws_getContext(
      config.width, config.height, AV_PIX_FMT_YUYV422,
      config.width, config.height, AV_PIX_FMT_NV12,
      SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
  if (impl_->converter == nullptr) {
    error = "sws_getContext failed";
    return false;
  }

  return true;
}

bool RdkH264Encoder::encodeFrame(
    const std::uint8_t * yuyv,
    int stride_bytes,
    std::uint64_t pts,
    std::vector<std::uint8_t> & out,
    std::string & error)
{
  if (yuyv == nullptr || !impl_->initialized || !impl_->started) {
    error = "encoder not initialized or null input";
    return false;
  }

  const int width = impl_->config.width;
  const int height = impl_->config.height;

  media_codec_buffer_t input{};
  int result = hb_mm_mc_dequeue_input_buffer(&impl_->context, &input, 2000);
  if (result != 0) {
    error = "hb_mm_mc_dequeue_input_buffer failed, result=" +
        std::to_string(result);
    return false;
  }
  input.type = MC_VIDEO_FRAME_BUFFER;
  input.vframe_buf.width = static_cast<uint32_t>(width);
  input.vframe_buf.height = static_cast<uint32_t>(height);
  input.vframe_buf.pix_fmt = MC_PIXEL_FORMAT_NV12;
  input.vframe_buf.size = static_cast<uint32_t>(width * height * 3 / 2);

  const uint8_t * source_data[4] = {yuyv, nullptr, nullptr, nullptr};
  const int source_stride[4] = {stride_bytes, 0, 0, 0};
  uint8_t * destination_data[4] = {
      input.vframe_buf.vir_ptr[0],
      input.vframe_buf.vir_ptr[1],
      nullptr,
      nullptr};
  const int destination_stride[4] = {width, width, 0, 0};
  const int converted = sws_scale(
      impl_->converter, source_data, source_stride, 0, height,
      destination_data, destination_stride);
  if (converted != height) {
    error = "sws_scale returned an incomplete frame";
    return false;
  }

  input.vframe_buf.pts = pts;
  result = hb_mm_mc_queue_input_buffer(&impl_->context, &input, 2000);
  if (result != 0) {
    error = "hb_mm_mc_queue_input_buffer failed, result=" +
        std::to_string(result);
    return false;
  }

  media_codec_buffer_t output{};
  media_codec_output_buffer_info_t information{};
  result = hb_mm_mc_dequeue_output_buffer(
      &impl_->context, &output, &information, 2000);
  if (result != 0) {
    error = "hb_mm_mc_dequeue_output_buffer failed, result=" +
        std::to_string(result);
    return false;
  }

  const uint8_t * encoded =
      static_cast<const uint8_t *>(output.vstream_buf.vir_ptr);
  out.insert(out.end(), encoded, encoded + output.vstream_buf.size);

  result = hb_mm_mc_queue_output_buffer(&impl_->context, &output, 2000);
  if (result != 0) {
    error = "hb_mm_mc_queue_output_buffer failed, result=" +
        std::to_string(result);
    return false;
  }

  return true;
}

}  // namespace drone_perception
