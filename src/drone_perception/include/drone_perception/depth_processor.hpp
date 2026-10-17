#pragma once

#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

struct DepthSampleResult
{
  bool has_valid_depth = false;
  float depth_m = 0.0F;
  cv::Point3d point_3d;
};

class DepthProcessor
{
public:
  DepthSampleResult sampleAt(
      const cv::Mat &depth_image,
      int u,
      int v,
      int sample_radius_px = 10) const;

  cv::Point3d projectTo3D(
      int u,
      int v,
      float depth_m,
      const sensor_msgs::msg::CameraInfo &camera_info);

private:
  static float sampleDepthMeters(
      const cv::Mat &depth_image,
      int u,
      int v,
      int sample_radius_px);

  image_geometry::PinholeCameraModel camera_model_;
};
