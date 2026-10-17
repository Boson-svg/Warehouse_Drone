#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>
#include <queue>

namespace offboard_run {

// 位姿滑动窗口平均器。
//
// tf_bridge_node 在刚启动时会连续接收若干帧 MAVROS 本地位姿，
// 用这些位姿估计无人机起始位置和起始朝向，然后发布静态 TF：
//
//   world_enu -> world_body
//
// 这个类的职责就是维护一个固定长度窗口，并输出窗口内位姿的平均值。
// 位置部分使用普通算术平均；姿态部分使用一个轻量的四元数归一化平均。
class SlidingWindowPoseAverage {
 public:
  // window_size 表示参与平均的最大位姿帧数。
  // 例如 tf_bridge_node 当前使用 10 帧，能过滤启动瞬间的小抖动。
  explicit SlidingWindowPoseAverage(int window_size) : window_size_(window_size) {}

  // 清空窗口和累计量。
  //
  // /reset_world_tf 服务会调用它，让 TF 初始化重新开始采样。
  void clear() {
    std::queue<geometry_msgs::msg::PoseStamped> empty;
    std::swap(pose_queue_, empty);
    sum_position_x_ = 0.0;
    sum_position_y_ = 0.0;
    sum_position_z_ = 0.0;
    sum_quaternion_ = Eigen::Quaterniond::Identity();
    quaternion_count_ = 0;
  }

  // 向窗口中加入一帧新位姿。
  //
  // 如果新位姿和上一帧距离突变超过阈值，认为定位发生跳变，
  // 直接重置窗口，避免旧数据污染新的 TF 初始化结果。
  void addPose(const geometry_msgs::msg::PoseStamped &new_pose) {
    if (!pose_queue_.empty() && exceedsResetThreshold(new_pose, pose_queue_.back())) {
      reset(new_pose);
    } else {
      pose_queue_.push(new_pose);
      sum_position_x_ += new_pose.pose.position.x;
      sum_position_y_ += new_pose.pose.position.y;
      sum_position_z_ += new_pose.pose.position.z;
      addQuaternion(toEigenQuaternion(new_pose.pose.orientation));
    }

    // 固定窗口长度：超过 window_size_ 时移除最老的一帧，
    // 同时从位置累计量和姿态累计量中减去它的贡献。
    if (static_cast<int>(pose_queue_.size()) > window_size_) {
      const auto &old_pose = pose_queue_.front();
      sum_position_x_ -= old_pose.pose.position.x;
      sum_position_y_ -= old_pose.pose.position.y;
      sum_position_z_ -= old_pose.pose.position.z;
      removeQuaternion(toEigenQuaternion(old_pose.pose.orientation));
      pose_queue_.pop();
    }
  }

  // 计算当前窗口的平均位姿。
  //
  // 当窗口为空时返回原点 + 单位四元数，避免出现非法姿态。
  geometry_msgs::msg::PoseStamped computeAveragePose() const {
    geometry_msgs::msg::PoseStamped avg_pose;
    const int n = static_cast<int>(pose_queue_.size());
    if (n == 0) {
      avg_pose.pose.orientation.w = 1.0;
      return avg_pose;
    }

    avg_pose.pose.position.x = sum_position_x_ / n;
    avg_pose.pose.position.y = sum_position_y_ / n;
    avg_pose.pose.position.z = sum_position_z_ / n;
    avg_pose.pose.orientation = toRosQuaternion(sum_quaternion_);
    return avg_pose;
  }

  // 当前窗口中已有的样本数量。
  // tf_bridge_node 会等待 getSize() 达到窗口大小后再初始化 TF。
  int getSize() const { return static_cast<int>(pose_queue_.size()); }

 private:
  // 用一帧新位姿重新开始窗口统计。
  //
  // 常见触发原因是定位跳变或手动 reset。
  void reset(const geometry_msgs::msg::PoseStamped &new_pose) {
    clear();
    pose_queue_.push(new_pose);
    sum_position_x_ = new_pose.pose.position.x;
    sum_position_y_ = new_pose.pose.position.y;
    sum_position_z_ = new_pose.pose.position.z;
    sum_quaternion_ = toEigenQuaternion(new_pose.pose.orientation);
    quaternion_count_ = 1;
  }

  // 把一个四元数加入平均值。
  //
  // 这里采用“当前平均 * 样本数 + 新四元数”再归一化的轻量写法，
  // 对启动阶段小范围姿态抖动足够使用。
  //
  // 注意：这不是严格的球面平均。如果后续遇到大角度跨越或姿态反号问题，
  // 可以替换为更严谨的四元数平均算法。
  void addQuaternion(const Eigen::Quaterniond &q) {
    if (quaternion_count_ == 0) {
      sum_quaternion_ = q;
    } else {
      sum_quaternion_ =
          Eigen::Quaterniond(sum_quaternion_.w() * quaternion_count_ + q.w(),
                             sum_quaternion_.x() * quaternion_count_ + q.x(),
                             sum_quaternion_.y() * quaternion_count_ + q.y(),
                             sum_quaternion_.z() * quaternion_count_ + q.z())
              .normalized();
    }
    quaternion_count_++;
  }

  // 从当前四元数平均中移除最老一帧。
  //
  // 这是为了支持固定长度滑动窗口。当窗口超过上限时，
  // addPose() 会调用它去掉队首位姿的姿态贡献。
  void removeQuaternion(const Eigen::Quaterniond &q) {
    if (quaternion_count_ <= 1) {
      sum_quaternion_ = Eigen::Quaterniond::Identity();
      quaternion_count_ = 0;
    } else {
      sum_quaternion_ =
          Eigen::Quaterniond(sum_quaternion_.w() * quaternion_count_ - q.w(),
                             sum_quaternion_.x() * quaternion_count_ - q.x(),
                             sum_quaternion_.y() * quaternion_count_ - q.y(),
                             sum_quaternion_.z() * quaternion_count_ - q.z())
              .normalized();
      quaternion_count_--;
    }
  }

  // 判断相邻两帧位置是否发生跳变。
  //
  // 当前阈值是 1cm。如果新位姿相对上一帧超过这个距离，
  // 就认为窗口里的旧数据不再可信，需要重置。
  bool exceedsResetThreshold(const geometry_msgs::msg::PoseStamped &a,
                             const geometry_msgs::msg::PoseStamped &b) const {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    const double dz = a.pose.position.z - b.pose.position.z;
    return (dx * dx + dy * dy + dz * dz) > reset_threshold_squared_;
  }

  // ROS2 geometry_msgs 四元数 -> Eigen 四元数。
  //
  // 两者字段顺序不同：Eigen 构造函数是 (w, x, y, z)。
  Eigen::Quaterniond toEigenQuaternion(
      const geometry_msgs::msg::Quaternion &q) const {
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  // Eigen 四元数 -> ROS2 geometry_msgs 四元数。
  geometry_msgs::msg::Quaternion toRosQuaternion(
      const Eigen::Quaterniond &q) const {
    geometry_msgs::msg::Quaternion ros_q;
    ros_q.w = q.w();
    ros_q.x = q.x();
    ros_q.y = q.y();
    ros_q.z = q.z();
    return ros_q;
  }

  // 窗口最大长度。
  int window_size_;

  // 位置累计和。使用累计和可以让平均计算保持 O(1)。
  double sum_position_x_ = 0.0;
  double sum_position_y_ = 0.0;
  double sum_position_z_ = 0.0;

  // 姿态平均值和参与姿态平均的样本数量。
  Eigen::Quaterniond sum_quaternion_ = Eigen::Quaterniond::Identity();
  int quaternion_count_ = 0;

  // 固定长度 FIFO 队列，保存窗口内的原始位姿。
  std::queue<geometry_msgs::msg::PoseStamped> pose_queue_;

  // 位置跳变重置阈值的平方，单位 m^2。
  // 用平方距离比较可以避免每帧调用 sqrt。
  const double reset_threshold_squared_ = 0.01 * 0.01;
};

}  // namespace offboard_run
