#include "drone_warehouse/ground_world_body_tf.hpp"

#include <cmath>

bool GroundWorldBodyTf::update(
    const GroundTfPose &world_enu_pose,
    GroundTfPose &world_body_pose)
{
    if (!isFinitePose(world_enu_pose))
    {
        return false;
    }

    if (!ready_)
    {
        addInitializationPose(world_enu_pose);
        if (!ready_)
        {
            return false;
        }
    }

    world_body_pose = transformPose(world_enu_pose);
    return true;
}

void GroundWorldBodyTf::reset()
{
    initialization_poses_.clear();
    last_initialization_pose_ = GroundTfPose{};
    has_last_initialization_pose_ = false;

    sum_position_x_ = 0.0;
    sum_position_y_ = 0.0;
    sum_position_z_ = 0.0;
    average_quaternion_ = Quaternion{};
    quaternion_count_ = 0;

    reference_pose_ = GroundTfPose{};
    ready_ = false;
}

bool GroundWorldBodyTf::isReady() const
{
    return ready_;
}

int GroundWorldBodyTf::sampleCount() const
{
    return static_cast<int>(initialization_poses_.size());
}

bool GroundWorldBodyTf::isFinitePose(const GroundTfPose &pose)
{
    return std::isfinite(pose.x) &&
           std::isfinite(pose.y) &&
           std::isfinite(pose.z) &&
           std::isfinite(pose.qx) &&
           std::isfinite(pose.qy) &&
           std::isfinite(pose.qz) &&
           std::isfinite(pose.qw);
}

GroundWorldBodyTf::Quaternion
GroundWorldBodyTf::poseQuaternion(const GroundTfPose &pose)
{
    return normalized(
        Quaternion{pose.qw, pose.qx, pose.qy, pose.qz});
}

GroundWorldBodyTf::Quaternion
GroundWorldBodyTf::normalized(const Quaternion &q)
{
    const double norm = std::sqrt(
        q.w * q.w + q.x * q.x +
        q.y * q.y + q.z * q.z);
    if (norm <= 1e-12)
    {
        return Quaternion{};
    }

    return Quaternion{
        q.w / norm,
        q.x / norm,
        q.y / norm,
        q.z / norm};
}

GroundWorldBodyTf::Quaternion
GroundWorldBodyTf::conjugate(const Quaternion &q)
{
    return Quaternion{q.w, -q.x, -q.y, -q.z};
}

GroundWorldBodyTf::Quaternion
GroundWorldBodyTf::multiply(
    const Quaternion &left,
    const Quaternion &right)
{
    return Quaternion{
        left.w * right.w - left.x * right.x -
            left.y * right.y - left.z * right.z,
        left.w * right.x + left.x * right.w +
            left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z +
            left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y -
            left.y * right.x + left.z * right.w};
}

void GroundWorldBodyTf::rotateVector(
    const Quaternion &rotation,
    double x,
    double y,
    double z,
    double &rotated_x,
    double &rotated_y,
    double &rotated_z)
{
    const Quaternion q = normalized(rotation);
    const Quaternion vector{0.0, x, y, z};
    const Quaternion result =
        multiply(multiply(q, vector), conjugate(q));
    rotated_x = result.x;
    rotated_y = result.y;
    rotated_z = result.z;
}

bool GroundWorldBodyTf::exceedsResetThreshold(
    const GroundTfPose &current,
    const GroundTfPose &previous) const
{
    const double dx = current.x - previous.x;
    const double dy = current.y - previous.y;
    const double dz = current.z - previous.z;
    return dx * dx + dy * dy + dz * dz >
           kResetDistanceSquared;
}

void GroundWorldBodyTf::addInitializationPose(
    const GroundTfPose &pose)
{
    // 与控制端相同：初始化期间发生超过 1 cm 的位置跳变时，丢弃旧窗口重新采样。
    if (has_last_initialization_pose_ &&
        exceedsResetThreshold(pose, last_initialization_pose_))
    {
        reset();
    }

    initialization_poses_.push_back(pose);
    last_initialization_pose_ = pose;
    has_last_initialization_pose_ = true;
    sum_position_x_ += pose.x;
    sum_position_y_ += pose.y;
    sum_position_z_ += pose.z;
    addQuaternion(poseQuaternion(pose));

    if (sampleCount() >= kInitializationSampleCount)
    {
        initializeReferencePose();
    }
}

void GroundWorldBodyTf::addQuaternion(const Quaternion &q)
{
    // 保持与控制程序相同的轻量归一化平均方式。
    if (quaternion_count_ == 0)
    {
        average_quaternion_ = q;
    }
    else
    {
        average_quaternion_ = normalized(
            Quaternion{
                average_quaternion_.w * quaternion_count_ + q.w,
                average_quaternion_.x * quaternion_count_ + q.x,
                average_quaternion_.y * quaternion_count_ + q.y,
                average_quaternion_.z * quaternion_count_ + q.z});
    }
    ++quaternion_count_;
}

void GroundWorldBodyTf::initializeReferencePose()
{
    const double count =
        static_cast<double>(initialization_poses_.size());
    reference_pose_.x = sum_position_x_ / count;
    reference_pose_.y = sum_position_y_ / count;
    reference_pose_.z = sum_position_z_ / count;

    const Quaternion reference_orientation =
        normalized(average_quaternion_);
    reference_pose_.qw = reference_orientation.w;
    reference_pose_.qx = reference_orientation.x;
    reference_pose_.qy = reference_orientation.y;
    reference_pose_.qz = reference_orientation.z;
    ready_ = true;
}

GroundTfPose GroundWorldBodyTf::transformPose(
    const GroundTfPose &world_enu_pose) const
{
    const Quaternion reference_inverse =
        conjugate(poseQuaternion(reference_pose_));

    GroundTfPose world_body_pose;
    rotateVector(
        reference_inverse,
        world_enu_pose.x - reference_pose_.x,
        world_enu_pose.y - reference_pose_.y,
        world_enu_pose.z - reference_pose_.z,
        world_body_pose.x,
        world_body_pose.y,
        world_body_pose.z);

    const Quaternion world_body_orientation = normalized(
        multiply(
            reference_inverse,
            poseQuaternion(world_enu_pose)));
    world_body_pose.qw = world_body_orientation.w;
    world_body_pose.qx = world_body_orientation.x;
    world_body_pose.qy = world_body_orientation.y;
    world_body_pose.qz = world_body_orientation.z;
    return world_body_pose;
}
