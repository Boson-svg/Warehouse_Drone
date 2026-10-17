#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

struct Mat3 {
  std::array<std::array<double, 3>, 3> m{{
      {{1.0, 0.0, 0.0}},
      {{0.0, 1.0, 0.0}},
      {{0.0, 0.0, 1.0}},
  }};
};

class SlidingWindowAverage {
 public:
  SlidingWindowAverage(size_t window_size, double jump_reset_rad)
      : window_size_(window_size), jump_reset_rad_(jump_reset_rad) {}

  void reset(size_t window_size, double jump_reset_rad) {
    window_size_ = window_size;
    jump_reset_rad_ = jump_reset_rad;
    clear();
  }

  void clear() {
    data_queue_.clear();
    sum_sin_ = 0.0;
    sum_cos_ = 0.0;
    has_last_ = false;
    last_value_ = 0.0;
  }

  void add(double new_data) {
    if (has_last_) {
      const double diff = shortestAngularDistance(last_value_, new_data);
      if (std::abs(diff) > jump_reset_rad_) {
        clear();
      }
    }

    data_queue_.push_back(new_data);
    sum_sin_ += std::sin(new_data);
    sum_cos_ += std::cos(new_data);

    if (data_queue_.size() > window_size_) {
      const double old = data_queue_.front();
      data_queue_.pop_front();
      sum_sin_ -= std::sin(old);
      sum_cos_ -= std::cos(old);
    }

    last_value_ = new_data;
    has_last_ = true;
  }

  size_t size() const { return data_queue_.size(); }

  double average() const {
    if (data_queue_.empty()) {
      return 0.0;
    }
    return std::atan2(sum_sin_, sum_cos_);
  }

  static double normalizeAngle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double shortestAngularDistance(double from, double to) {
    return normalizeAngle(to - from);
  }

 private:
  size_t window_size_;
  double jump_reset_rad_;
  std::deque<double> data_queue_;
  double sum_sin_ = 0.0;
  double sum_cos_ = 0.0;
  bool has_last_ = false;
  double last_value_ = 0.0;
};

class FastLioToMavros : public rclcpp::Node {
 public:
  FastLioToMavros() : Node("fastlio_to_mavros"), yaw_window_(8, 0.15) {
    declareAndLoadParameters();
    setupInterfaces();
    logConfiguration();
  }

 private:
  struct AcceptedState {
    std::array<double, 3> pos_enu{0.0, 0.0, 0.0};
    double yaw_enu = 0.0;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool valid = false;
  };

  void declareAndLoadParameters() {
    pos_sigma_ = declare_parameter<double>("pos_sigma", 0.05);
    att_sigma_ = declare_parameter<double>("att_sigma", 0.03);
    lin_vel_sigma_ = declare_parameter<double>("lin_vel_sigma", 0.10);
    ang_vel_sigma_ = declare_parameter<double>("ang_vel_sigma", 0.05);

    pose_frame_id_ = declare_parameter<std::string>("pose_frame_id", "map");
    child_frame_id_ =
        declare_parameter<std::string>("child_frame_id", "base_link");

    mount_x_ = declare_parameter<double>("mount_x", 0.0);
    mount_y_ = declare_parameter<double>("mount_y", 0.0);
    mount_z_ = declare_parameter<double>("mount_z", 0.05);
    mount_roll_deg_ = declare_parameter<double>("mount_roll_deg", 0.0);
    mount_pitch_deg_ = declare_parameter<double>("mount_pitch_deg", 0.0);
    mount_yaw_deg_ = declare_parameter<double>("mount_yaw_deg", 0.0);

    fastlio_topic_ =
        declare_parameter<std::string>("fastlio_odom_topic", "/Odometry");
    px4_odom_topic_ = declare_parameter<std::string>(
        "px4_odom_topic", "/mavros/local_position/odom");
    vision_pose_topic_ = declare_parameter<std::string>(
        "vision_pose_topic", "/mavros/vision_pose/pose");
    odom_out_topic_ = declare_parameter<std::string>(
        "odom_out_topic", "/mavros/odometry/out");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz",20.0);
    max_input_age_sec_ = declare_parameter<double>("max_input_age_sec", 0.2);
    use_input_stamp_ = declare_parameter<bool>("use_input_stamp", false);

    fallback_zero_yaw_on_no_px4_ =
        declare_parameter<bool>("fallback_zero_yaw_on_no_px4", true);

    yaw_window_size_ =
        static_cast<size_t>(declare_parameter<int>("yaw_window_size", 8));
    yaw_jump_reset_rad_ =
        declare_parameter<double>("yaw_jump_reset_rad", 0.15);
    fallback_init_delay_sec_ =
        declare_parameter<double>("fallback_init_delay_sec", 2.0);

    min_valid_dt_sec_ = declare_parameter<double>("min_valid_dt_sec", 1e-4);
    max_valid_dt_sec_ = declare_parameter<double>("max_valid_dt_sec", 0.5);
    max_position_jump_m_ =
        declare_parameter<double>("max_position_jump_m", 2.0);
    max_implied_speed_mps_ =
        declare_parameter<double>("max_implied_speed_mps", 20.0);
    max_yaw_rate_rps_ = declare_parameter<double>("max_yaw_rate_rps", 3.0);
    max_consecutive_bad_frames_ =
        declare_parameter<int>("max_consecutive_bad_frames", 5);

    disabled_twist_covariance_ =
        declare_parameter<double>("disabled_twist_covariance", 1e6);

    yaw_window_.reset(yaw_window_size_, yaw_jump_reset_rad_);
    updateMountTransform();

    pos_var_ = pos_sigma_ * pos_sigma_;
    att_var_ = att_sigma_ * att_sigma_;
    lin_vel_var_ = lin_vel_sigma_ * lin_vel_sigma_;
    ang_vel_var_ = ang_vel_sigma_ * ang_vel_sigma_;
  }

  void setupInterfaces() {
    rclcpp::QoS fastlio_qos(rclcpp::KeepLast(10));
    fastlio_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    fastlio_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    rclcpp::QoS px4_odom_qos(rclcpp::KeepLast(10));
    px4_odom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    px4_odom_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    fastlio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        fastlio_topic_, fastlio_qos,
        std::bind(&FastLioToMavros::fastlioCallback, this,
                  std::placeholders::_1));

    px4_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        px4_odom_topic_, px4_odom_qos,
        std::bind(&FastLioToMavros::px4OdomCallback, this,
                  std::placeholders::_1));

    vision_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        vision_pose_topic_, 10);

    reset_service_ = create_service<std_srvs::srv::Empty>(
        "reset_initialization",
        std::bind(&FastLioToMavros::resetCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    init_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&FastLioToMavros::fallbackInitCheck, this));

    const auto publish_period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_)));
    publish_timer_ = create_wall_timer(
        std::max(std::chrono::milliseconds(1), publish_period),
        std::bind(&FastLioToMavros::publishLatestPose, this));
  }

  void logConfiguration() const {
    std::ostringstream oss;
    oss << "FastLioToMavros config"
        << " | fastlio_topic=" << fastlio_topic_
        << " | px4_odom_topic=" << px4_odom_topic_
        << " | vision_pose_topic=" << vision_pose_topic_
        << " | odom_out_topic(declared_only)=" << odom_out_topic_
        << " | pose_frame_id=" << pose_frame_id_
        << " | child_frame_id(declared_only)=" << child_frame_id_
        << " | publish_rate_hz=" << publish_rate_hz_
        << " | max_input_age_sec=" << max_input_age_sec_
        << " | use_input_stamp=" << (use_input_stamp_ ? "true" : "false")
        << " | mount_xyz=(" << mount_x_ << "," << mount_y_ << ","
        << mount_z_ << ")"
        << " | mount_rpy_deg=(" << mount_roll_deg_ << ","
        << mount_pitch_deg_ << "," << mount_yaw_deg_ << ")"
        << " | max_position_jump_m=" << max_position_jump_m_
        << " | max_implied_speed_mps=" << max_implied_speed_mps_
        << " | max_yaw_rate_rps=" << max_yaw_rate_rps_;
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

  void px4OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    has_px4_odom_ = true;

    tf2::Quaternion q_px4;
    tf2::fromMsg(msg->pose.pose.orientation, q_px4);
    if (!isFiniteQuaternion(q_px4)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "PX4 odom quaternion invalid, skip yaw init sample.");
      return;
    }
    q_px4.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q_px4).getRPY(roll, pitch, yaw);

    if (!initialized_) {
      yaw_window_.add(yaw);
      if (yaw_window_.size() >= yaw_window_size_) {
        initializeWithYaw(yaw_window_.average(), "px4_yaw_window");
      }
    }
  }

  void fastlioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    has_fastlio_odom_ = true;

    if (!first_fastlio_stamp_received_) {
      first_fastlio_received_wall_time_ = now();
      first_fastlio_stamp_received_ = true;
    }

    if (!initialized_) {
      return;
    }

    std::string reject_reason;
    geometry_msgs::msg::PoseStamped vision_pose;
    if (!buildVisionPose(*msg, vision_pose, reject_reason)) {
      handleBadFrame(reject_reason);
      return;
    }

    consecutive_bad_frames_ = 0;
    latest_pose_ = vision_pose;
    latest_pose_valid_ = true;
    latest_pose_received_wall_time_ = now();

    const auto& p = vision_pose.pose.position;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Bridge OK | init=%s | bad=%d | cached ENU pos=(%+.3f,%+.3f,%+.3f)",
        initialized_ ? "true" : "false", consecutive_bad_frames_, p.x, p.y,
        p.z);
  }

  bool buildVisionPose(const nav_msgs::msg::Odometry& msg,
                       geometry_msgs::msg::PoseStamped& vision_pose,
                       std::string& reject_reason) {
    const rclcpp::Time stamp(msg.header.stamp);

    if (last_input_stamp_valid_) {
      const double dt_from_input = (stamp - last_input_stamp_).seconds();
      if (dt_from_input <= 0.0) {
        reject_reason = "input timestamp went backwards or duplicated";
        return false;
      }
    }

    last_input_stamp_ = stamp;
    last_input_stamp_valid_ = true;

    if (!isFinitePose(msg.pose.pose)) {
      reject_reason = "pose contains NaN/Inf";
      return false;
    }

    tf2::Quaternion q_body_in_init;
    tf2::fromMsg(msg.pose.pose.orientation, q_body_in_init);
    if (!isFiniteQuaternion(q_body_in_init)) {
      reject_reason = "fastlio quaternion contains NaN/Inf";
      return false;
    }

    const double q_norm = q_body_in_init.length();
    if (q_norm < 1e-6) {
      reject_reason = "fastlio quaternion norm too small";
      return false;
    }
    q_body_in_init.normalize();

    const std::array<double, 3> p_sensor_in_world{
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
    };

    const std::array<double, 3> p_base_offset_in_world =
        matVecMul(quatToMat3(q_body_in_init), t_sensor_base_);
    const std::array<double, 3> p_base_in_world{
        p_sensor_in_world[0] + p_base_offset_in_world[0],
        p_sensor_in_world[1] + p_base_offset_in_world[1],
        p_sensor_in_world[2] + p_base_offset_in_world[2],
    };

    std::array<double, 3> p_enu{
        init_cos_yaw_ * p_base_in_world[0] - init_sin_yaw_ * p_base_in_world[1],
        init_sin_yaw_ * p_base_in_world[0] + init_cos_yaw_ * p_base_in_world[1],
        p_base_in_world[2],
    };

    tf2::Quaternion q_base_in_world = q_body_in_init * q_sensor_base_;
    q_base_in_world.normalize();

    tf2::Quaternion q_enu = init_q_ * q_base_in_world;
    if (!isFiniteQuaternion(q_enu)) {
      reject_reason = "output quaternion became invalid";
      return false;
    }
    q_enu.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw_enu = 0.0;
    tf2::Matrix3x3(q_enu).getRPY(roll, pitch, yaw_enu);

    if (accepted_state_.valid) {
      const double dt = (stamp - accepted_state_.stamp).seconds();
      if (dt < min_valid_dt_sec_) {
        std::ostringstream oss;
        oss << "dt too small: " << dt << " s";
        reject_reason = oss.str();
        return false;
      }
      if (dt > max_valid_dt_sec_) {
        reject_reason = "dt too large, waiting for stream to stabilize";
        return false;
      }

      const double dx = p_enu[0] - accepted_state_.pos_enu[0];
      const double dy = p_enu[1] - accepted_state_.pos_enu[1];
      const double dz = p_enu[2] - accepted_state_.pos_enu[2];
      const double delta_pos = norm3(dx, dy, dz);
      if (delta_pos > max_position_jump_m_) {
        std::ostringstream oss;
        oss << "position jump too large: " << delta_pos << " m";
        reject_reason = oss.str();
        return false;
      }

      const double implied_speed = delta_pos / dt;
      if (implied_speed > max_implied_speed_mps_) {
        std::ostringstream oss;
        oss << "implied speed too large: " << implied_speed << " m/s";
        reject_reason = oss.str();
        return false;
      }

      const double yaw_rate =
          std::abs(SlidingWindowAverage::shortestAngularDistance(
                       accepted_state_.yaw_enu, yaw_enu)) /
          dt;
      if (yaw_rate > max_yaw_rate_rps_) {
        std::ostringstream oss;
        oss << "yaw rate too large: " << yaw_rate << " rad/s";
        reject_reason = oss.str();
        return false;
      }
    }

    vision_pose.header.stamp = msg.header.stamp;
    vision_pose.header.frame_id = pose_frame_id_;
    vision_pose.pose.position.x = p_enu[0];
    vision_pose.pose.position.y = p_enu[1];
    vision_pose.pose.position.z = p_enu[2];
    vision_pose.pose.orientation = tf2::toMsg(q_enu);

    accepted_state_.pos_enu = p_enu;
    accepted_state_.yaw_enu = yaw_enu;
    accepted_state_.stamp = stamp;
    accepted_state_.valid = true;
    return true;
  }

  void publishLatestPose() {
    if (!initialized_ || !latest_pose_valid_) {
      return;
    }

    const double age = (now() - latest_pose_received_wall_time_).seconds();
    if (age > max_input_age_sec_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Latest FAST_LIO pose too old for publish: age=%.3f s", age);
      return;
    }

    auto out = latest_pose_;
    if (!use_input_stamp_) {
      out.header.stamp = now();
    }
    vision_pub_->publish(out);
  }

  void handleBadFrame(const std::string& reason) {
    ++consecutive_bad_frames_;
    RCLCPP_WARN(get_logger(), "Drop bad frame (%d/%d): %s",
                consecutive_bad_frames_, max_consecutive_bad_frames_,
                reason.c_str());

    if (consecutive_bad_frames_ >= max_consecutive_bad_frames_) {
      enterReinitialization(reason);
    }
  }

  void enterReinitialization(const std::string& reason) {
    initialized_ = false;
    accepted_state_ = AcceptedState();
    latest_pose_valid_ = false;
    yaw_window_.clear();
    consecutive_bad_frames_ = 0;
    last_input_stamp_valid_ = false;

    RCLCPP_ERROR(
        get_logger(),
        "Bridge entered reinitialization because: %s | waiting px4 yaw window again.",
        reason.c_str());
  }

  void initializeWithYaw(double init_yaw, const std::string& source) {
    init_yaw_ = SlidingWindowAverage::normalizeAngle(init_yaw);
    init_cos_yaw_ = std::cos(init_yaw_);
    init_sin_yaw_ = std::sin(init_yaw_);
    init_q_.setRPY(0.0, 0.0, init_yaw_);
    init_q_.normalize();
    initialized_ = true;
    accepted_state_ = AcceptedState();
    latest_pose_valid_ = false;
    consecutive_bad_frames_ = 0;
    last_input_stamp_valid_ = false;

    RCLCPP_INFO(get_logger(),
                "Initialization complete from %s: init_yaw = %.2f deg",
                source.c_str(), init_yaw_ * 180.0 / M_PI);
  }

  void fallbackInitCheck() {
    if (initialized_ || !fallback_zero_yaw_on_no_px4_ || !has_fastlio_odom_ ||
        has_px4_odom_) {
      return;
    }

    if (!first_fastlio_stamp_received_) {
      return;
    }

    const double elapsed =
        (now() - first_fastlio_received_wall_time_).seconds();
    if (elapsed >= fallback_init_delay_sec_) {
      initializeWithYaw(0.0, "fallback_zero_yaw");
      RCLCPP_WARN(
          get_logger(),
          "No PX4 odom received within %.2f s. Fallback to zero yaw. If heading is not aligned, fixed yaw bias will remain.",
          fallback_init_delay_sec_);
    }
  }

  void resetCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                     std::shared_ptr<std_srvs::srv::Empty::Response>) {
    initialized_ = false;
    has_fastlio_odom_ = false;
    has_px4_odom_ = false;
    first_fastlio_stamp_received_ = false;
    accepted_state_ = AcceptedState();
    latest_pose_valid_ = false;
    yaw_window_.clear();
    consecutive_bad_frames_ = 0;
    last_input_stamp_valid_ = false;
    RCLCPP_WARN(get_logger(),
                "Manual reset requested. Waiting for reinitialization.");
  }

  void updateMountTransform() {
    const double roll = mount_roll_deg_ * M_PI / 180.0;
    const double pitch = mount_pitch_deg_ * M_PI / 180.0;
    const double yaw = mount_yaw_deg_ * M_PI / 180.0;

    q_base_sensor_.setRPY(roll, pitch, yaw);
    q_base_sensor_.normalize();
    q_sensor_base_ = q_base_sensor_.inverse();
    q_sensor_base_.normalize();

    r_base_sensor_ = quatToMat3(q_base_sensor_);
    r_sensor_base_ = quatToMat3(q_sensor_base_);

    t_base_sensor_ = {mount_x_, mount_y_, mount_z_};
    const std::array<double, 3> neg_t_base_sensor{-mount_x_, -mount_y_, -mount_z_};
    t_sensor_base_ = matVecMul(r_sensor_base_, neg_t_base_sensor);
  }

  static Mat3 quatToMat3(const tf2::Quaternion& q) {
    tf2::Matrix3x3 basis(q);
    Mat3 out;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        out.m[i][j] = basis[i][j];
      }
    }
    return out;
  }

  static std::array<double, 3> matVecMul(const Mat3& mat,
                                         const std::array<double, 3>& vec) {
    return {
        mat.m[0][0] * vec[0] + mat.m[0][1] * vec[1] + mat.m[0][2] * vec[2],
        mat.m[1][0] * vec[0] + mat.m[1][1] * vec[1] + mat.m[1][2] * vec[2],
        mat.m[2][0] * vec[0] + mat.m[2][1] * vec[1] + mat.m[2][2] * vec[2],
    };
  }

  static bool isFiniteDouble(double value) { return std::isfinite(value); }

  static bool isFiniteQuaternion(const tf2::Quaternion& q) {
    return isFiniteDouble(q.x()) && isFiniteDouble(q.y()) &&
           isFiniteDouble(q.z()) && isFiniteDouble(q.w());
  }

  static bool isFinitePose(const geometry_msgs::msg::Pose& pose) {
    return isFiniteDouble(pose.position.x) &&
           isFiniteDouble(pose.position.y) &&
           isFiniteDouble(pose.position.z) &&
           isFiniteDouble(pose.orientation.x) &&
           isFiniteDouble(pose.orientation.y) &&
           isFiniteDouble(pose.orientation.z) &&
           isFiniteDouble(pose.orientation.w);
  }

  static double norm3(double x, double y, double z) {
    return std::sqrt(x * x + y * y + z * z);
  }

 private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr px4_odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  double pos_sigma_ = 0.05;
  double att_sigma_ = 0.03;
  double lin_vel_sigma_ = 0.10;
  double ang_vel_sigma_ = 0.05;

  double pos_var_ = 0.0;
  double att_var_ = 0.0;
  double lin_vel_var_ = 0.0;
  double ang_vel_var_ = 0.0;

  std::string pose_frame_id_ = "map";
  std::string child_frame_id_ = "base_link";
  double mount_x_ = 0.0;
  double mount_y_ = -0.09;
  double mount_z_ = 0.0;
  double mount_roll_deg_ = 0.0;
  double mount_pitch_deg_ = 0.0;
  double mount_yaw_deg_ = 0.0;
  std::string fastlio_topic_ = "/Odometry";
  std::string px4_odom_topic_ = "/mavros/local_position/odom";
  std::string vision_pose_topic_ = "/mavros/vision_pose/pose";
  std::string odom_out_topic_ = "/mavros/odometry/out";

  double publish_rate_hz_ = 30.0;
  double max_input_age_sec_ = 0.2;
  bool use_input_stamp_ = false;

  bool fallback_zero_yaw_on_no_px4_ = true;

  size_t yaw_window_size_ = 8;
  double yaw_jump_reset_rad_ = 0.15;
  double fallback_init_delay_sec_ = 2.0;

  double min_valid_dt_sec_ = 1e-4;
  double max_valid_dt_sec_ = 0.5;
  double max_position_jump_m_ = 2.0;
  double max_implied_speed_mps_ = 20.0;
  double max_yaw_rate_rps_ = 3.0;
  int max_consecutive_bad_frames_ = 5;
  double disabled_twist_covariance_ = 1e6;

  SlidingWindowAverage yaw_window_;

  bool initialized_ = false;
  bool has_fastlio_odom_ = false;
  bool has_px4_odom_ = false;
  bool first_fastlio_stamp_received_ = false;
  bool latest_pose_valid_ = false;

  rclcpp::Time first_fastlio_received_wall_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_pose_received_wall_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_input_stamp_{0, 0, RCL_ROS_TIME};
  bool last_input_stamp_valid_ = false;

  double init_yaw_ = 0.0;
  double init_cos_yaw_ = 1.0;
  double init_sin_yaw_ = 0.0;
  tf2::Quaternion init_q_{0.0, 0.0, 0.0, 1.0};
  tf2::Quaternion q_base_sensor_{0.0, 0.0, 0.0, 1.0};
  tf2::Quaternion q_sensor_base_{0.0, 0.0, 0.0, 1.0};
  Mat3 r_base_sensor_{};
  Mat3 r_sensor_base_{};
  std::array<double, 3> t_base_sensor_{0.0, 0.0, 0.0};
  std::array<double, 3> t_sensor_base_{0.0, 0.0, 0.0};

  int consecutive_bad_frames_ = 0;
  AcceptedState accepted_state_;
  geometry_msgs::msg::PoseStamped latest_pose_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastLioToMavros>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
