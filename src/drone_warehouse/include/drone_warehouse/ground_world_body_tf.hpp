#pragma once

#include <deque>

// 地面站内部使用的轻量位姿，不依赖 TF2 节点或额外 ROS 话题。
// 输入单位与 geometry_msgs::msg::PoseStamped 相同：位置为米，姿态为四元数。
struct GroundTfPose
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
};

// 在地面站复现控制程序的固定 world_body 坐标系：
//   1. 缓存最开始 10 帧 world_enu/local_position；
//   2. 平均得到起始位置和起始姿态；
//   3. 后续位姿先减去起点，再乘起始姿态的逆旋转。
//
// 转换建立后坐标轴保持固定，不会跟随无人机当前航向继续旋转。
class GroundWorldBodyTf
{
public:
    static constexpr int kInitializationSampleCount = 10;

    // 输入一帧原始 world_enu 位姿。
    // TF 未完成 10 帧初始化时返回 false；完成后通过 world_body_pose 返回转换结果。
    bool update(
        const GroundTfPose &world_enu_pose,
        GroundTfPose &world_body_pose);

    // 开始新一轮任务或重新连接时调用，使下一批 10 帧成为新的固定参考位姿。
    void reset();

    bool isReady() const;
    int sampleCount() const;

private:
    struct Quaternion
    {
        double w = 1.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    static bool isFinitePose(const GroundTfPose &pose);
    static Quaternion poseQuaternion(const GroundTfPose &pose);
    static Quaternion normalized(const Quaternion &q);
    static Quaternion conjugate(const Quaternion &q);
    static Quaternion multiply(
        const Quaternion &left,
        const Quaternion &right);
    static void rotateVector(
        const Quaternion &rotation,
        double x,
        double y,
        double z,
        double &rotated_x,
        double &rotated_y,
        double &rotated_z);

    bool exceedsResetThreshold(
        const GroundTfPose &current,
        const GroundTfPose &previous) const;
    void addInitializationPose(const GroundTfPose &pose);
    void addQuaternion(const Quaternion &q);
    void initializeReferencePose();
    GroundTfPose transformPose(const GroundTfPose &world_enu_pose) const;

    std::deque<GroundTfPose> initialization_poses_;
    GroundTfPose last_initialization_pose_;
    bool has_last_initialization_pose_ = false;

    double sum_position_x_ = 0.0;
    double sum_position_y_ = 0.0;
    double sum_position_z_ = 0.0;
    Quaternion average_quaternion_;
    int quaternion_count_ = 0;

    GroundTfPose reference_pose_;
    bool ready_ = false;

    // 与控制程序 SlidingWindowPoseAverage 相同：相邻初始化帧跳变超过 1 cm 时重采。
    static constexpr double kResetDistanceSquared = 0.01 * 0.01;
};
