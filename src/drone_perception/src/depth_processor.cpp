#include "drone_perception/depth_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

float DepthProcessor::sampleDepthMeters(
  const cv::Mat & depth_image,
  int u,
  int v,
  int sample_radius_px)
{
  if (depth_image.empty() || u < 0 || v < 0 ||
    u >= depth_image.cols || v >= depth_image.rows)
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  std::vector<float> valid_depths;
  const int radius = std::max(0, sample_radius_px);

  for (int dv = -radius; dv <= radius; ++dv) {
    for (int du = -radius; du <= radius; ++du) {
      const int sample_u = u + du;
      const int sample_v = v + dv;

      if (sample_u < 0 || sample_v < 0 ||
        sample_u >= depth_image.cols || sample_v >= depth_image.rows)
      {
        continue;
      }

      float depth_m = std::numeric_limits<float>::quiet_NaN();

      if (depth_image.type() == CV_16UC1) {
        const uint16_t depth_raw = depth_image.at<uint16_t>(sample_v, sample_u);
        if (depth_raw != 0U) {
          depth_m = static_cast<float>(depth_raw) * 0.001F;
        }
      } else if (depth_image.type() == CV_32FC1) {
        const float depth_raw = depth_image.at<float>(sample_v, sample_u);
        if (std::isfinite(depth_raw) && depth_raw > 0.0F) {
          depth_m = depth_raw;
        }
      }

      if (std::isfinite(depth_m)) {
        valid_depths.push_back(depth_m);
      }
    }
  }

  if (valid_depths.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  std::sort(valid_depths.begin(), valid_depths.end());
  return valid_depths[valid_depths.size() / 2];
}

DepthSampleResult DepthProcessor::sampleAt(
  const cv::Mat & depth_image,
  int u,
  int v,
  int sample_radius_px) const
{
  DepthSampleResult result;
  result.depth_m = sampleDepthMeters(depth_image, u, v, sample_radius_px);
  result.has_valid_depth = std::isfinite(result.depth_m);
  return result;
}

cv::Point3d DepthProcessor::projectTo3D(
  int u,
  int v,
  float depth_m,
  const sensor_msgs::msg::CameraInfo & camera_info)
{
  camera_model_.fromCameraInfo(camera_info);
  const cv::Point3d ray = camera_model_.projectPixelTo3dRay(cv::Point2d(u, v));
  return ray * static_cast<double>(depth_m);
}
