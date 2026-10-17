#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include "drone_msgs/msg/k230_animal_targets.hpp"
#include "drone_msgs/msg/k230_animal_target.hpp"
#include "drone_msgs/msg/k230_capture_ready.hpp"
#include "drone_msgs/msg/k230_record_result.hpp"
#include "drone_msgs/msg/k230_scan_point_done.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map> // std::unordered_map
#include <deque>         // std::deque
#include <vector>

#include "drone_action.hpp"

using offboard_run::ActionStatus;
using offboard_run::ActionType;
using offboard_run::DroneAction;
using offboard_run::HoldAxis;
using offboard_run::SpatialPoint;

class Vec3dPID
{
public:
    Vec3dPID(double p, double i, double d)
        : p_gain_(p),
          i_gain_(i),
          d_gain_(d),
          integral_(0.0, 0.0, 0.0),
          last_error_(0.0, 0.0, 0.0) {}

    Eigen::Vector3d update(const Eigen::Vector3d &error, double dt)
    {
        integral_ += error * dt;
        const Eigen::Vector3d derivative = (error - last_error_) / dt;
        last_error_ = error;
        return p_gain_ * error + i_gain_ * integral_ + d_gain_ * derivative;
    }

    Eigen::Vector3d update(const Eigen::Vector3d &error,
                           const rclcpp::Time &time_stamp)
    {
        if (last_time_.nanoseconds() == 0)
        {
            last_time_ = time_stamp;
            return Eigen::Vector3d::Zero();
        }

        const double dt = (time_stamp - last_time_).seconds();
        last_time_ = time_stamp;
        if (dt <= 0.0)
        {
            return Eigen::Vector3d::Zero();
        }
        return update(error, dt);
    }

    void reset()
    {
        last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        integral_.setZero();
        last_error_.setZero();
    }

private:
    double p_gain_;
    double i_gain_;
    double d_gain_;

    rclcpp::Time last_time_;
    Eigen::Vector3d integral_;
    Eigen::Vector3d last_error_;
};

struct TaskRuntimeStatus
{
    bool task_running;
    std::string action_name;
    int32_t action_step;
};

enum class VisionTargetState
{
    PENDING = 0,
    TRACKING,
    CAPTURE_REQUESTED,
    CAPTURED,
    FAILED,
    SKIPPED,
};

inline std::string makeVisionTargetKey(
    int32_t scan_point_index,
    const std::string &label,
    uint32_t label_instance_id)
{
    std::ostringstream oss;
    oss << scan_point_index << "|"
        << label << "|"
        << label_instance_id;
    return oss.str();
}

struct VisionTargetEntry
{
    std::string key;
    int32_t scan_point_index = -1;
    uint32_t frame_seq = 0;
    std::string label;
    uint32_t label_instance_id = 0;

    drone_msgs::msg::K230AnimalTarget target_msg;
    VisionTargetState state = VisionTargetState::PENDING;

    bool capture_ready_sent = false;
    rclcpp::Time first_seen_time;
    rclcpp::Time last_seen_time;

    uint32_t latest_frame_seq = 0; // 记录这个目标最近一次出现在哪一帧
    std::string image_name;        // 保存视觉端回传结果

    rclcpp::Time capture_requested_time;
};

struct ActiveScanPointContext
{
    int32_t scan_point_index = -1;
    uint32_t latest_frame_seq = 0;
    double scan_point_x = 0.0;
    double scan_point_y = 0.0;

    std::unordered_map<std::string, VisionTargetEntry> targets_by_key;
    std::deque<std::string> target_order;

    std::string current_target_key;
    rclcpp::Time last_targets_msg_time;
    bool has_active_context = false;
    bool scan_point_done_sent = false;       // 防止重复发布
    rclcpp::Time scan_point_start_time;      // 单网格总超时 30s
    rclcpp::Time empty_targets_since;        // 连续空目标确认窗口
    bool has_received_targets_frame = false; // 区分“空网格”与“根本没收到数据”
};

struct MoveRuntimeState
{
    bool initialized = false;

    geometry_msgs::msg::PoseStamped start_pose;
    // 进入动作时一次性冻结后的world_enu终点
    geometry_msgs::msg::PoseStamped resolved_target_pose;
    // 上一次真正发出去的平滑 setpoint
    geometry_msgs::msg::PoseStamped last_command_pose;

    rclcpp::Time last_update_time;

    double start_yaw_rad = 0.0;
    // 用于 yaw 平滑推进
    double target_yaw_rad = 0.0;
    double last_command_yaw_rad = 0.0;

    double total_distance_m = 0.0;
    int stable_count = 0;
};

class ActionExecutor
{
public:
    ActionExecutor(const rclcpp::Node::SharedPtr &node,
                   tf2_ros::Buffer &tf_buffer)
        : node_(node),
          tf_buffer_(tf_buffer),
          camera_aim_pid_p_(node_->declare_parameter<double>("camera_aim_pid_p", 0.001)),
          camera_aim_pid_i_(node_->declare_parameter<double>("camera_aim_pid_i", 0.0)),
          camera_aim_pid_d_(node_->declare_parameter<double>("camera_aim_pid_d", 0.001)),
          pid_cam_aim_(camera_aim_pid_p_, camera_aim_pid_i_, camera_aim_pid_d_)
    {
        camera_aim_target_timeout_s_ =
            node_->declare_parameter<double>("camera_aim_target_timeout_s", 0.5);
        camera_aim_stable_cycles_ =
            node_->declare_parameter<int>("camera_aim_stable_cycles", 20);
        camera_aim_max_step_ =
            node_->declare_parameter<double>("camera_aim_max_step", 0.05);

        camera_aim_wait_first_targets_timeout_s_ =
            node_->declare_parameter<double>("camera_aim_wait_first_targets_timeout_s", 2.0);

        camera_aim_no_target_confirm_s_ =
            node_->declare_parameter<double>("camera_aim_no_target_confirm_s", 2.0);

        camera_aim_record_result_timeout_s_ =
            node_->declare_parameter<double>("camera_aim_record_result_timeout_s", 5.0);

        camera_aim_scan_point_timeout_s_ =
            node_->declare_parameter<double>("camera_aim_scan_point_timeout_s", 30.0);
        land_setpoint_quiet_time_s_ =
            node_->declare_parameter<double>("land_setpoint_quiet_time_s", 0.2);

        setpoint_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", rclcpp::QoS(10).reliable());
        capture_ready_pub_ = node_->create_publisher<drone_msgs::msg::K230CaptureReady>(
            "/k230/animals/capture_ready", rclcpp::QoS(10).reliable());

        step_pub_ = node_->create_publisher<std_msgs::msg::String>("/step", 10);
        mission_status_pub_ =
            node_->create_publisher<std_msgs::msg::String>("/mission_status", 10);

        hover_active_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/mission/hover_active", rclcpp::QoS(10).reliable().transient_local());

        scan_point_done_pub_ = node_->create_publisher<drone_msgs::msg::K230ScanPointDone>(
            "/k230/animals/scan_point_done", rclcpp::QoS(10).reliable());
        
        state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            std::bind(&ActionExecutor::state_callback, this, std::placeholders::_1));

        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
            std::bind(&ActionExecutor::pose_callback, this, std::placeholders::_1));

        camera_aim_sub_ = node_->create_subscription<geometry_msgs::msg::Point>(
            "/camera_aiming_center", rclcpp::SensorDataQoS(),
            std::bind(&ActionExecutor::cameraAim_callback, this, std::placeholders::_1));

        k230_targets_sub_ = node_->create_subscription<drone_msgs::msg::K230AnimalTargets>(
            "/k230/animals/targets",
            rclcpp::SensorDataQoS(),
            std::bind(&ActionExecutor::k230TargetsCallback, this, std::placeholders::_1));

        record_result_sub_ = node_->create_subscription<drone_msgs::msg::K230RecordResult>(
            "/k230/animals/record_result",
            rclcpp::QoS(10).reliable(),
            std::bind(&ActionExecutor::recordResultCallback, this, std::placeholders::_1));

        arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        land_client_ = node_->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

        last_finish_pose_.header.frame_id = "world_enu";
        last_finish_pose_.pose.orientation.w = 1.0;

        last_camera_aim_time_ = node_->now();

        RCLCPP_INFO(node_->get_logger(),
                    "动作执行器初始化完成，已准备执行任务。camera_aim_timeout=%.2f s, stable_cycles=%d, max_step=%.3f m, pid=(%.6f, %.6f, %.6f)",
                    camera_aim_target_timeout_s_,
                    camera_aim_stable_cycles_,
                    camera_aim_max_step_,
                    camera_aim_pid_p_,
                    camera_aim_pid_i_,
                    camera_aim_pid_d_);
    }

    void addAction(const std::shared_ptr<DroneAction> &action)
    {
        action_queue_.push(action);
        RCLCPP_INFO(node_->get_logger(), "已添加动作，当前队列长度：%zu", action_queue_.size());
    }

    void clearAction()
    {
        publishVisionHoverActive(false);
        while (!action_queue_.empty())
        {
            action_queue_.pop();
        }
        action_id_ = 0;
        current_action_.reset();
        resetActionRuntimeState();
    }

    void emergencyStop()
    {
        clearAction();
        if (current_pose_received_)
        {
            sendPositionSetpoint(current_pose_);
        }
    }

    void sendPositionSetpoint(const geometry_msgs::msg::PoseStamped &pose)
    {
        geometry_msgs::msg::PoseStamped target = pose;
        target.header.stamp = node_->now();
        if (target.header.frame_id.empty())
        {
            target.header.frame_id = "world_enu";
        }
        setpoint_pub_->publish(target);
    }

    void sendDummyPose(double altitude = 0.0)
    {
        if (current_pose_received_)
        {
            sendPositionSetpoint(current_pose_);
            return;
        }

        geometry_msgs::msg::PoseStamped dummy_pose;
        dummy_pose.header.frame_id = "world_enu";
        dummy_pose.header.stamp = node_->now();
        dummy_pose.pose.position.z = altitude;
        dummy_pose.pose.orientation.w = 1.0;
        sendPositionSetpoint(dummy_pose);
    }

    bool isIdle() const
    {
        return !current_action_ && action_queue_.empty();
    }

    TaskRuntimeStatus getTaskRuntimeStatus() const
    {
        TaskRuntimeStatus status;
        status.task_running = static_cast<bool>(current_action_);
        status.action_step = current_action_ ? action_id_ : 0;
        status.action_name = current_action_ ? actionTypeToString(current_action_->getType()) : "idle";
        return status;
    }

    void controlLoop()
    {
        if (!current_pose_received_)
        {
            sendDummyPose();
            return;
        }

        if (!current_state_.connected)
        {
            sendPositionSetpoint(current_pose_);
            return;
        }

        const bool executing_land =
            current_action_ &&
            current_action_->getType() == ActionType::LAND;
        if (!current_state_.armed && !executing_land)
        {
            sendPositionSetpoint(current_pose_);
            return;
        }

        std_msgs::msg::String step_msg;
        step_msg.data = std::to_string(action_id_);
        step_pub_->publish(step_msg);

        if (!current_action_ && !action_queue_.empty())
        {
            current_action_ = action_queue_.front();
            action_queue_.pop();
            action_id_++;
            current_action_->setStatus(ActionStatus::EXECUTING);
            current_action_->setStartTime(node_->now());
            force_status_publish_ = true;
            RCLCPP_INFO(node_->get_logger(), "开始执行动作，动作类型编号：%d", static_cast<int>(current_action_->getType()));
        }

        if (current_action_)
        {
            executeAction(current_action_);
        }
    }

private:
    void publishCaptureReady(const VisionTargetEntry &target)
    {
        drone_msgs::msg::K230CaptureReady msg;
        msg.frame_seq = target.latest_frame_seq;
        msg.scan_point_index = target.scan_point_index;
        msg.label = target.label;
        msg.label_instance_id = target.label_instance_id;
        msg.capture_ready = true;
        capture_ready_pub_->publish(msg);

        RCLCPP_INFO(
            node_->get_logger(),
            "已发布 capture_ready：scan_point=%d, frame_seq=%u, label=%s, instance_id=%u。",
            target.scan_point_index,
            target.latest_frame_seq,
            target.label.c_str(),
            target.label_instance_id);
    }

    void skipCurrentTargetAndFinishAction(const std::string &reason)
    {
        VisionTargetEntry *current = getCurrentTarget();
        if (current != nullptr)
        {
            current->state = VisionTargetState::SKIPPED;
            RCLCPP_WARN(
                node_->get_logger(),
                "当前目标已跳过：key=%s, label=%s, instance_id=%u，原因：%s",
                current->key.c_str(),
                current->label.c_str(),
                current->label_instance_id,
                reason.c_str());
        }

        active_scan_point_.current_target_key.clear();
        resetCameraAimTrackingState();
        completeCurrentAction(reason);
        tryFinishActiveScanPoint();
    }

    Eigen::Vector3d bodyVectorToEnu(const Eigen::Vector3d &body_vec) const
    {
        Eigen::Quaterniond q_current(
            current_pose_.pose.orientation.w,
            current_pose_.pose.orientation.x,
            current_pose_.pose.orientation.y,
            current_pose_.pose.orientation.z);
        return q_current * body_vec;
    }

    void resetActionRuntimeState()
    {
        resetCameraAimTrackingState();
        resetMoveRuntimeState();
        land_mode_request_sent_ = false;
        land_setpoint_quiet_started_ = false;
        land_low_altitude_count_ = 0;
        land_setpoint_quiet_start_time_ =
            rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    }

    void resetCameraAimTrackingState()
    {
        aim_close_count_ = 0;
        pid_cam_aim_.reset();
    }

    void completeCurrentAction(const std::string &message)
    {
        if (current_action_ && current_action_->getType() == ActionType::HOVER)
        {
            publishVisionHoverActive(false);
        }
        if (current_action_)
        {
            current_action_->setStatus(ActionStatus::COMPLETED);
        }
        last_finish_pose_ = current_pose_;
        current_action_.reset();
        active_scan_point_.current_target_key.clear();
        resetActionRuntimeState();
        broadcastStatus(message);
    }

    void failCurrentAction(const std::string &message)
    {
        if (current_action_ && current_action_->getType() == ActionType::HOVER)
        {
            publishVisionHoverActive(false);
        }
        if (current_action_)
        {
            current_action_->setStatus(ActionStatus::FAILED);
        }
        current_action_.reset();
        resetActionRuntimeState();
        RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    }

    double getYawDeg(const geometry_msgs::msg::PoseStamped &pose) const
    {
        return getPoseYawRad(pose) * 57.29577951308232;
    }

    double getPoseYawRad(const geometry_msgs::msg::PoseStamped &pose) const
    {
        // move 平滑控制内部统一使用弧度制 yaw，避免角度/弧度混用。
        tf2::Quaternion q;
        tf2::fromMsg(pose.pose.orientation, q);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    geometry_msgs::msg::Quaternion makeQuaternionFromYaw(double yaw_rad) const
    {
        // /mavros/setpoint_position/local 仍然发 PoseStamped，因此需要把目标 yaw 重新封装成四元数。
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_rad);
        q.normalize();
        return tf2::toMsg(q);
    }

    double normalizeAngleRad(double angle_rad) const
    {
        // 把任意角度误差折叠回 [-pi, pi]，保证 yaw 总是走最短角路径。
        return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
    }

    double stepTowardYawRad(
        double current_yaw_rad,
        double target_yaw_rad,
        double max_step_rad) const
    {
        // yaw 不直接跳到终点，而是按本周期允许的最大角步长逼近目标。
        const double yaw_error = normalizeAngleRad(target_yaw_rad - current_yaw_rad);
        if (std::abs(yaw_error) <= max_step_rad)
        {
            return target_yaw_rad;
        }

        return normalizeAngleRad(
            current_yaw_rad + std::copysign(max_step_rad, yaw_error));
    }

    Eigen::Vector3d stepTowardPosition(
        const Eigen::Vector3d &from,
        const Eigen::Vector3d &to,
        double max_xy_step,
        double max_z_step) const
    {
        // 位置平滑推进分成水平和竖直两部分限速：
        // xy 用平面距离限速，z 单独限速，便于室内把升降速度设得更保守。
        Eigen::Vector3d result = from;

        const Eigen::Vector2d xy_delta = to.head<2>() - from.head<2>();
        const double xy_distance = xy_delta.norm();
        if (xy_distance <= max_xy_step || max_xy_step <= 0.0)
        {
            result.x() = to.x();
            result.y() = to.y();
        }
        else
        {
            const Eigen::Vector2d xy_step = xy_delta.normalized() * max_xy_step;
            result.x() += xy_step.x();
            result.y() += xy_step.y();
        }

        const double z_delta = to.z() - from.z();
        if (std::abs(z_delta) <= max_z_step || max_z_step <= 0.0)
        {
            result.z() = to.z();
        }
        else
        {
            result.z() += std::copysign(max_z_step, z_delta);
        }

        return result;
    }

    void resetMoveRuntimeState()
    {
        move_runtime_.initialized = false;
        move_runtime_.start_pose = geometry_msgs::msg::PoseStamped{};
        move_runtime_.resolved_target_pose = geometry_msgs::msg::PoseStamped{};
        move_runtime_.last_command_pose = geometry_msgs::msg::PoseStamped{};
        move_runtime_.last_update_time =
            rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        move_runtime_.start_yaw_rad = 0.0;
        move_runtime_.target_yaw_rad = 0.0;
        move_runtime_.last_command_yaw_rad = 0.0;
        move_runtime_.total_distance_m = 0.0;
        move_runtime_.stable_count = 0;
    }

    bool resolveMoveTargetPoseOnce(
        const std::shared_ptr<DroneAction> &action,
        geometry_msgs::msg::PoseStamped &resolved_target_pose)
    {
        // move 动作一开始就把目标冻结成固定 world_enu 终点。
        // 这样后续即使机体继续转向，BODY / WORLD_BODY 目标也不会在执行过程中漂移。
        resolved_target_pose = action->getTargetPose();

        if (action->getFrame() == DroneAction::Frame::WORLD_BODY)
        {
            try
            {
                resolved_target_pose = tf_buffer_.transform(resolved_target_pose, "world_enu");
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(node_->get_logger(), "坐标变换失败：%s，已保持当前位置。", ex.what());
                return false;
            }
        }
        else if (action->getFrame() == DroneAction::Frame::BODY)
        {
            // BODY 模式下，位置和 yaw 都解释为“相对上一动作结束姿态”的增量。
            const Eigen::Vector3d delta = bodyVectorToEnu(
                Eigen::Vector3d(
                    resolved_target_pose.pose.position.x,
                    resolved_target_pose.pose.position.y,
                    resolved_target_pose.pose.position.z));
            const double target_yaw_delta = getPoseYawRad(resolved_target_pose);

            resolved_target_pose = last_finish_pose_;
            resolved_target_pose.pose.position.x += delta.x();
            resolved_target_pose.pose.position.y += delta.y();
            resolved_target_pose.pose.position.z += delta.z();
            resolved_target_pose.pose.orientation =
                makeQuaternionFromYaw(normalizeAngleRad(getPoseYawRad(last_finish_pose_) + target_yaw_delta));
        }

        if (resolved_target_pose.header.frame_id.empty())
        {
            resolved_target_pose.header.frame_id = "world_enu";
        }

        return true;
    }

    bool initializeMoveRuntime(const std::shared_ptr<DroneAction> &action)
    {
        // 首次进入 move 时初始化运行时状态。
        // 这里缓存的是“平滑轨迹状态”，而不是仅缓存最终目标点。
        geometry_msgs::msg::PoseStamped resolved_target_pose;
        if (!resolveMoveTargetPoseOnce(action, resolved_target_pose))
        {
            return false;
        }

        move_runtime_.initialized = true;
        move_runtime_.start_pose = current_pose_;
        move_runtime_.resolved_target_pose = resolved_target_pose;
        move_runtime_.last_command_pose = current_pose_;
        move_runtime_.last_command_pose.header.frame_id = "world_enu";
        move_runtime_.last_update_time = node_->now();
        move_runtime_.start_yaw_rad = getPoseYawRad(current_pose_);
        move_runtime_.target_yaw_rad = getPoseYawRad(resolved_target_pose);
        move_runtime_.last_command_yaw_rad = move_runtime_.start_yaw_rad;
        move_runtime_.total_distance_m =
            SpatialPoint(current_pose_).distance(SpatialPoint(resolved_target_pose));
        move_runtime_.stable_count = 0;
        return true;
    }

    geometry_msgs::msg::PoseStamped buildNextMoveSetpoint(
        const std::shared_ptr<DroneAction> &action,
        double dt)
    {
        // 参考轨迹生成策略：
        // 从“上一次发出的命令位姿”出发，朝最终目标推进一个受限步长。
        // 这样 setpoint 本身是连续变化的，而不是每周期都跳到最终终点。
        geometry_msgs::msg::PoseStamped next_setpoint = move_runtime_.last_command_pose;

        const Eigen::Vector3d from(
            move_runtime_.last_command_pose.pose.position.x,
            move_runtime_.last_command_pose.pose.position.y,
            move_runtime_.last_command_pose.pose.position.z);
        const Eigen::Vector3d to(
            move_runtime_.resolved_target_pose.pose.position.x,
            move_runtime_.resolved_target_pose.pose.position.y,
            move_runtime_.resolved_target_pose.pose.position.z);

        const Eigen::Vector3d next_position = stepTowardPosition(
            from,
            to,
            action->getMoveMaxXYSpeed() * dt,
            action->getMoveMaxZSpeed() * dt);

        next_setpoint.header.frame_id = "world_enu";
        next_setpoint.pose.position.x = next_position.x();
        next_setpoint.pose.position.y = next_position.y();
        next_setpoint.pose.position.z = next_position.z();

        move_runtime_.last_command_yaw_rad = stepTowardYawRad(
            move_runtime_.last_command_yaw_rad,
            move_runtime_.target_yaw_rad,
            action->getMoveMaxYawRateRadps() * dt);
        next_setpoint.pose.orientation =
            makeQuaternionFromYaw(move_runtime_.last_command_yaw_rad);

        return next_setpoint;
    }

    bool isMoveGoalReached(
        const std::shared_ptr<DroneAction> &action) const
    {
        // move 完成条件同时看位置和 yaw，避免“人到位了但机头还在转”。
        const SpatialPoint current(current_pose_);
        const SpatialPoint target(move_runtime_.resolved_target_pose);
        const double position_error = current.distance(target);
        const double yaw_error = std::abs(normalizeAngleRad(
            move_runtime_.target_yaw_rad - getPoseYawRad(current_pose_)));

        return position_error < action->getPositionTolerance() &&
               yaw_error < action->getYawToleranceRad();
    }

    std::string formatPose(const geometry_msgs::msg::PoseStamped &pose) const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2)
            << "x=" << pose.pose.position.x << " m, "
            << "y=" << pose.pose.position.y << " m, "
            << "z=" << pose.pose.position.z << " m, "
            << "yaw=" << getYawDeg(pose) << " deg";
        return oss.str();
    }

    bool transformToWorldBody(const geometry_msgs::msg::PoseStamped &source,
                              geometry_msgs::msg::PoseStamped &target)
    {
        try
        {
            target = tf_buffer_.transform(source, "world_body");
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "状态播报坐标变换失败：%s，暂时使用 ENU 坐标。",
                                 ex.what());
            target = source;
            return false;
        }
    }

    std::string formatStatusPose(
        const geometry_msgs::msg::PoseStamped &pose,
        const char *fallback_frame_text,
        bool &using_world_body)
    {
        geometry_msgs::msg::PoseStamped world_body_pose;
        if (transformToWorldBody(pose, world_body_pose))
        {
            using_world_body = true;
            return formatPose(world_body_pose);
        }

        using_world_body = false;
        return std::string(fallback_frame_text) + " " + formatPose(world_body_pose);
    }

    std::string formatSeconds(double seconds) const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) << seconds;
        return oss.str();
    }

    std::string formatMeters(double meters) const
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << meters;
        return oss.str();
    }

    void publishStatus(const std::string &message)
    {
        std_msgs::msg::String status_msg;
        status_msg.data = message;
        mission_status_pub_->publish(status_msg);
    }

    void broadcastStatus(const std::string &message)
    {
        publishStatus(message);
        RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
    }

    void broadcastStatusThrottled(const std::string &message)
    {
        const rclcpp::Time now = node_->now();
        const bool should_publish =
            force_status_publish_ ||
            last_status_publish_time_.nanoseconds() == 0 ||
            (now - last_status_publish_time_).seconds() >= status_publish_period_s_;

        if (!should_publish)
        {
            return;
        }

        last_status_publish_time_ = now;
        force_status_publish_ = false;
        broadcastStatus(message);
    }

    void executeAction(const std::shared_ptr<DroneAction> &action)
    {
        switch (action->getType())
        {
        case ActionType::MOVE_TO_POSITION:
            executeMoveToPosition(action);
            break;
        case ActionType::HOVER:
            executeHover(action);
            break;
        case ActionType::CAMERA_AIM:
            executeCameraAim(action);
            break;
        case ActionType::LAND:
            executeLand(action);
            break;
        case ActionType::TAKEOFF:
            executeTakeoff(action);
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "未知动作类型，已保持当前位置。");
            sendPositionSetpoint(current_pose_);
            break;
        }
    }

    void executeMoveToPosition(const std::shared_ptr<DroneAction> &action)
    {
        if (!move_runtime_.initialized)
        {
            if (!initializeMoveRuntime(action))
            {
                sendPositionSetpoint(current_pose_);
                return;
            }
        }

        const rclcpp::Time now = node_->now();
        double dt = (now - move_runtime_.last_update_time).seconds();
        move_runtime_.last_update_time = now;
        if (dt <= 0.0)
        {
            // 首帧或时钟异常时给一个保底周期，避免本周期步长为 0。
            dt = 0.02;
        }

        // 根据当前平滑轨迹状态生成下一帧 setpoint。
        const geometry_msgs::msg::PoseStamped next_setpoint =
            buildNextMoveSetpoint(action, dt);
        move_runtime_.last_command_pose = next_setpoint;
        sendPositionSetpoint(next_setpoint);

        if (isMoveGoalReached(action))
        {
            // 连续满足阈值才完成，降低位置估计轻微抖动带来的误判。
            move_runtime_.stable_count++;
            if (move_runtime_.stable_count > 20)
            {
                completeCurrentAction("已平滑到达目标位置与目标朝向。");
                return;
            }
        }
        else
        {
            move_runtime_.stable_count = 0;
        }

        const SpatialPoint current(current_pose_);
        const SpatialPoint target_point(move_runtime_.resolved_target_pose);
        const double distance_to_target = current.distance(target_point);
        const double yaw_error_deg = std::abs(normalizeAngleRad(
            move_runtime_.target_yaw_rad - getPoseYawRad(current_pose_))) * 57.29577951308232;
        bool current_uses_world_body = false;
        bool target_uses_world_body = false;
        const std::string current_text =
            formatStatusPose(current_pose_, "ENU", current_uses_world_body);
        const std::string target_text =
            formatStatusPose(move_runtime_.resolved_target_pose, "ENU", target_uses_world_body);
        const std::string frame_text =
            current_uses_world_body && target_uses_world_body
                ? "初始点坐标系 world_body"
                : "坐标变换未就绪，混合坐标";

        broadcastStatusThrottled(
            "航点飞行中（" + frame_text + "）：当前 " + current_text +
            "，目标 " + target_text +
            "，剩余距离=" + formatMeters(distance_to_target) +
            " m，剩余偏航=" + formatSeconds(yaw_error_deg) + " deg。");
    }

    void executeHover(const std::shared_ptr<DroneAction> &action)
    {
        publishVisionHoverActive(action->shouldNotifyVisionHover());
        sendPositionSetpoint(last_finish_pose_);
        const double elapsed = (node_->now() - action->getStartTime()).seconds();
        const double remaining = std::max(0.0, action->getHoverTime() - elapsed);
        if (elapsed > action->getHoverTime())
        {
            completeCurrentAction("悬停动作已完成。");
            return;
        }

        bool hold_uses_world_body = false;
        const std::string hold_text =
            formatStatusPose(last_finish_pose_, "ENU", hold_uses_world_body);
        const std::string frame_text =
            hold_uses_world_body ? "初始点坐标系 world_body"
                                 : "坐标变换未就绪，ENU";

        broadcastStatusThrottled(
            "悬停中（" + frame_text + "）：保持位置 " + hold_text +
            "，已悬停=" + formatSeconds(elapsed) + " s，剩余=" +
            formatSeconds(remaining) + " s。");
    }

    void publishVisionHoverActive(bool active)
    {
        if (vision_hover_active_published_ && vision_hover_active_ == active)
        {
            return;
        }

        std_msgs::msg::Bool msg;
        msg.data = active;
        hover_active_pub_->publish(msg);
        vision_hover_active_ = active;
        vision_hover_active_published_ = true;
    }

    void executeCameraAim(const std::shared_ptr<DroneAction> &action)
    {
        const rclcpp::Time now = node_->now();

        if (!active_scan_point_.has_received_targets_frame)
        {
            sendPositionSetpoint(current_pose_);

            const double wait_first_targets = 
                (now - action->getStartTime()).seconds();

            broadcastStatusThrottled(
                "camera_aim 等待 /k230/animals/targets 首帧：已等待=" +
                formatSeconds(wait_first_targets) + " s。");

            if (wait_first_targets > camera_aim_wait_first_targets_timeout_s_)
            {
                completeCurrentAction("等待 /k230/animals/targets 首帧超时，结束当前 camera_aim。");
            }
            return;
        }

        if (active_scan_point_.targets_by_key.empty() &&
            active_scan_point_.empty_targets_since.nanoseconds() != 0)
        {
            sendPositionSetpoint(current_pose_);
            const double empty_duration =
                (now - active_scan_point_.empty_targets_since).seconds();

            broadcastStatusThrottled(
                "camera_aim 空网格确认中：scan_point=" +
                std::to_string(active_scan_point_.scan_point_index) +
                "，连续空帧时长=" + formatSeconds(empty_duration) + " s。");

            if ((now - active_scan_point_.empty_targets_since).seconds() > camera_aim_no_target_confirm_s_)
            {
                publishScanPointDone();
                resetActiveScanPoint();
                completeCurrentAction("当前 scan point 连续空目标，已判定为空网格。");
            }
            return;
        }

        if (active_scan_point_.scan_point_start_time.nanoseconds() != 0 &&
            (now - active_scan_point_.scan_point_start_time).seconds() > camera_aim_scan_point_timeout_s_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "当前 scan point=%d 处理总时长已超过 %.1f s，强制结束剩余目标。",
                active_scan_point_.scan_point_index,
                camera_aim_scan_point_timeout_s_);

            for (auto &[key, target] : active_scan_point_.targets_by_key)
            {
                if (!isTerminalState(target.state))
                {
                    target.state = VisionTargetState::SKIPPED;
                }
            }

            publishScanPointDone();
            resetActiveScanPoint();
            completeCurrentAction("当前 scan point 处理超时，已强制结束。");
            return;
        }

        VisionTargetEntry *current_target = getCurrentTarget();
        if (current_target != nullptr && current_target->state == VisionTargetState::CAPTURE_REQUESTED)
        {
            sendPositionSetpoint(current_pose_);
            const double wait_result =
                (now - current_target->capture_requested_time).seconds();

            broadcastStatusThrottled(
                "camera_aim 等待 record_result：scan_point=" +
                std::to_string(current_target->scan_point_index) +
                "，label=" + current_target->label +
                "，instance_id=" + std::to_string(current_target->label_instance_id) +
                "，已等待=" + formatSeconds(wait_result) + " s。");

            if ((now - current_target->capture_requested_time).seconds() >
                camera_aim_record_result_timeout_s_)
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "目标等待 record_result 超时：key=%s, scan_point=%d, label=%s, instance_id=%u, timeout=%.1f s。已跳过当前目标。",
                    current_target->key.c_str(),
                    current_target->scan_point_index,
                    current_target->label.c_str(),
                current_target->label_instance_id,
                camera_aim_record_result_timeout_s_);

                current_target->state = VisionTargetState::SKIPPED;
                active_scan_point_.current_target_key.clear();
                resetCameraAimTrackingState();
                tryFinishActiveScanPoint();
            }
            return;
        }

        if (current_target == nullptr)
        {
            VisionTargetEntry *next = pickNextPendingTarget();
            if (next == nullptr)
            {
                tryFinishActiveScanPoint();

                if (!active_scan_point_.has_active_context)
                {
                    completeCurrentAction("当前 scan point 全部目标处理完成。");
                }
                else
                {
                    sendPositionSetpoint(current_pose_);
                }
                return;
            }
            active_scan_point_.current_target_key = next->key;
            next->state = VisionTargetState::TRACKING;
            resetCameraAimTrackingState();
            current_target = next;

            RCLCPP_INFO(
                node_->get_logger(),
                "camera_aim 选择新目标进入 TRACKING：scan_point=%d, key=%s, label=%s, instance_id=%u。已重置稳定计数和 PID 状态。",
                current_target->scan_point_index,
                current_target->key.c_str(),
                current_target->label.c_str(),
                current_target->label_instance_id);
        }

        // 当前目标太久没在新帧里出现，视为目标丢失。
        if ((now - current_target->last_seen_time).seconds() > camera_aim_target_timeout_s_)
        {
            current_target->state = VisionTargetState::SKIPPED;
            RCLCPP_WARN(
                node_->get_logger(),
                "当前视觉目标超时丢失：key=%s, scan_point=%d, label=%s, instance_id=%u。已跳过当前目标。",
                current_target->key.c_str(),
                current_target->scan_point_index,
                current_target->label.c_str(),
                current_target->label_instance_id);
            active_scan_point_.current_target_key.clear();
            resetCameraAimTrackingState();
            tryFinishActiveScanPoint();
            return;
        }

        const double err_x = static_cast<double>(current_target->target_msg.err_x);
        const double err_y = static_cast<double>(current_target->target_msg.err_y);

        if (std::abs(err_x) < action->getCameraAimTolerance() &&
            std::abs(err_y) < action->getCameraAimTolerance())
        {
            aim_close_count_++;
            if (aim_close_count_ > camera_aim_stable_cycles_)
            {
                if (!current_target->capture_ready_sent)
                {
                    publishCaptureReady(*current_target);
                    current_target->capture_ready_sent = true;
                }
                current_target->state = VisionTargetState::CAPTURE_REQUESTED;
                current_target->capture_requested_time = now;
                broadcastStatus("当前目标已满足拍照条件，等待视觉端 record_result。");
                return;
            }
        }
        else
        {
            aim_close_count_ = 0;
        }

        const Eigen::Vector3d camera_aim_vector{err_x, err_y, 0.0};

        Eigen::Vector3d body_delta = pid_cam_aim_.update(camera_aim_vector, now);

        for (int i = 0; i < 3; ++i)
        {
            body_delta[i] = std::clamp(body_delta[i], -camera_aim_max_step_, camera_aim_max_step_);
        }

        // MAVROS 外部控制使用 ENU，这里仍然保持“机体系增量 -> ENU 增量”的原有逻辑。
        const Eigen::Vector3d enu_delta = bodyVectorToEnu(body_delta);

        geometry_msgs::msg::PoseStamped target_pose = current_pose_;
        target_pose.pose.position.x += enu_delta.x();
        target_pose.pose.position.y += enu_delta.y();
        target_pose.pose.position.z += enu_delta.z();

        switch (action->getHoldAxis())
        {
        case HoldAxis::X:
            target_pose.pose.position.x = action->getTargetPose().pose.position.x;
            break;

        case HoldAxis::Y:
            target_pose.pose.position.y = action->getTargetPose().pose.position.y;
            break;

        case HoldAxis::Z:
            target_pose.pose.position.z = action->getTargetPose().pose.position.z;
            break;
        }

        broadcastStatusThrottled(
            "camera_aim 对准中：scan_point=" +
            std::to_string(current_target->scan_point_index) +
            "，label=" + current_target->label +
            "，instance_id=" + std::to_string(current_target->label_instance_id) +
            "，err_x=" + std::to_string(current_target->target_msg.err_x) +
            "，err_y=" + std::to_string(current_target->target_msg.err_y) + "。");

        sendPositionSetpoint(target_pose);
    }

    void executeLand(const std::shared_ptr<DroneAction> &action)
    {
        if (!action->isStartPoseInitialized())
        {
            action->setStartPose(current_pose_);
            force_status_publish_ = true;
        }

        if (!land_setpoint_quiet_started_)
        {
            land_setpoint_quiet_started_ = true;
            land_setpoint_quiet_start_time_ = node_->now();
            force_status_publish_ = true;
            broadcastStatus("降落接管开始：已停止发送位置 setpoint，准备切换 AUTO.LAND。");
            return;
        }

        if (!land_mode_request_sent_)
        {
            const double quiet_elapsed =
                (node_->now() - land_setpoint_quiet_start_time_).seconds();
            if (quiet_elapsed < land_setpoint_quiet_time_s_)
            {
                broadcastStatusThrottled(
                    "降落接管静默中：已停止位置 setpoint，等待 " +
                    formatSeconds(land_setpoint_quiet_time_s_ - quiet_elapsed) +
                    " s 后请求 AUTO.LAND。");
                return;
            }

            if (!callSetMode("AUTO.LAND"))
            {
                broadcastStatusThrottled(
                    "AUTO.LAND 切换失败，保持 setpoint 静默并重试：当前高度（ENU z）=" +
                    formatMeters(current_pose_.pose.position.z) + " m，当前模式=" +
                    current_state_.mode + "。");
                return;
            }
            land_mode_request_sent_ = true;
            force_status_publish_ = true;
            RCLCPP_INFO(node_->get_logger(), "降落模式切换请求已发送。");
        }

        const double current_z = current_pose_.pose.position.z;
        const double start_z = action->getStartPose().pose.position.z;
        const double descended = std::max(0.0, start_z - current_z);
        constexpr double kLandCompleteAltitudeM = 0.15;
        constexpr int kLandCompleteStableCycles = 20;

        if (current_z <= kLandCompleteAltitudeM)
        {
            ++land_low_altitude_count_;
        }
        else
        {
            land_low_altitude_count_ = 0;
        }

        if (!current_state_.armed ||
            land_low_altitude_count_ >= kLandCompleteStableCycles)
        {
            const std::string reason =
                !current_state_.armed ? "飞控已上锁" : "高度达到近地阈值";
            completeCurrentAction(
                "降落动作已完成（" + reason + "）：当前高度（ENU z）=" +
                formatMeters(current_z) + " m，相对降落开始已下降=" +
                formatMeters(descended) + " m。");
            return;
        }

        broadcastStatusThrottled(
            "降落中：当前高度（ENU z）=" + formatMeters(current_z) +
            " m，相对降落开始已下降=" + formatMeters(descended) +
            " m，飞控模式=" + current_state_.mode + "。");
    }

    void executeTakeoff(const std::shared_ptr<DroneAction> &action)
    {
        geometry_msgs::msg::PoseStamped takeoff_pose;
        if (action->isStartPoseInitialized())
        {
            takeoff_pose = action->getStartPose();
        }
        else
        {
            action->setStartPose(current_pose_);
            takeoff_pose = current_pose_;
        }

        takeoff_pose.header.frame_id = "world_enu";
        takeoff_pose.header.stamp = node_->now();
        const double target_z =
            action->getStartPose().pose.position.z + action->getTargetAltitude();
        takeoff_pose.pose.position.z = target_z;

        const SpatialPoint current(current_pose_);
        const SpatialPoint target(takeoff_pose);
        if (std::abs(current.z - target.z) < action->getPositionTolerance())
        {
            completeCurrentAction("起飞动作已完成。");
            return;
        }
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "正在执行起飞：当前 ENU z %.2f m，目标 ENU z %.2f m，相对起飞高度 %.2f m。",
                             current.z, target.z, action->getTargetAltitude());
        sendPositionSetpoint(takeoff_pose);
    }

    bool callSetMode(const std::string &mode)
    {
        if (!set_mode_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(node_->get_logger(), "飞行模式切换服务不可用。");
            return false;
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = mode;
        auto future = set_mode_client_->async_send_request(request);
        const auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(2));
        if (result != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_WARN(node_->get_logger(), "飞行模式切换请求超时，目标模式：%s。", mode.c_str());
            return false;
        }
        const bool mode_sent = future.get()->mode_sent;
        if (!mode_sent)
        {
            RCLCPP_WARN(node_->get_logger(), "飞控拒绝切换飞行模式，目标模式：%s。", mode.c_str());
        }
        return mode_sent;
    }

    void state_callback(mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void pose_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        current_pose_.header.frame_id = "world_enu";
        current_pose_received_ = true;
        if (!last_finish_pose_initialized_)
        {
            last_finish_pose_ = current_pose_;
            last_finish_pose_initialized_ = true;
        }
    }

    void cameraAim_callback(geometry_msgs::msg::Point::SharedPtr msg)
    {
        camera_aim_diff_ = *msg;
        last_camera_aim_time_ = node_->now();
    }

    bool hasContinuousInstanceIds(
        const drone_msgs::msg::K230AnimalTargets &msg) const
    {
        std::unordered_map<std::string, std::vector<uint32_t>> ids_by_label;

        for (const auto &target : msg.targets)
        {
            ids_by_label[target.label].push_back(target.label_instance_id);
        }

        for (auto &[label, ids] : ids_by_label)
        {
            std::sort(ids.begin(), ids.end());

            for (std::size_t i = 0; i < ids.size(); ++i)
            {
                const uint32_t expected = static_cast<uint32_t>(i + 1);

                if (ids[i] != expected)
                {
                    RCLCPP_WARN(
                        node_->get_logger(),
                        "目标帧校验失败：label=%s 的 label_instance_id 不连续，期望=%u，实际=%u。",
                        label.c_str(), expected, ids[i]);
                    return false;
                }
            }
        }

        return true;
    }

    bool isValidTargetsMessage(
        const drone_msgs::msg::K230AnimalTargets &msg) const
    {
        if (msg.scan_point_index < 0)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "目标帧校验失败：scan_point_index=%d 无效。",
                        msg.scan_point_index);
            return false; // 索引无效，校验失败
        }

        if (msg.target_count != msg.targets.size())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "目标帧校验失败：target_count=%u 与 targets.size()=%zu 不一致。",
                        msg.target_count, msg.targets.size());
            return false; // 计数不一致，校验失败
        }

        if (!hasContinuousInstanceIds(msg))
        {
            return false; // ID不连续，校验失败（具体警告在函数内部已输出）
        }

        // 所有校验都通过，消息有效
        return true;
    }

    void k230TargetsCallback(const drone_msgs::msg::K230AnimalTargets::SharedPtr msg)
    {
        // ========== 第一步：消息有效性校验 ==========
        // 校验扫描点索引、目标计数、实例ID连续性等
        // 如果无效，直接丢弃该消息，不进行任何处理
        if (!isValidTargetsMessage(*msg))
        {
            return;
        }

        const rclcpp::Time now = node_->now();
        active_scan_point_.has_received_targets_frame = true;
        active_scan_point_.last_targets_msg_time = now;

        // ========== 第二步：扫描点（Scan Point）激活逻辑 ==========
        // 扫描点代表无人机的一个悬停扫描位置，每个扫描点会收到多帧目标数据
        // 需要区分首次激活和后续数据更新
        if (!active_scan_point_.has_active_context)
        {
            // 情况1：还没有激活任何扫描点
            // 激活当前消息对应的扫描点，记录其基本信息
            active_scan_point_.latest_frame_seq = msg->frame_seq;
            active_scan_point_.has_active_context = true;
            active_scan_point_.scan_point_index = msg->scan_point_index; // 扫描点索引
            active_scan_point_.scan_point_x = msg->scan_point_x;         // 扫描点X坐标
            active_scan_point_.scan_point_y = msg->scan_point_y;         // 扫描点Y坐标
            active_scan_point_.scan_point_start_time = now;
            active_scan_point_.empty_targets_since =
                rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        }
        else if (!isSameScanPoint(msg->scan_point_index))
        {
            // 情况2：已有激活的扫描点，但新消息来自不同的扫描点
            // 当前扫描点尚未完成处理，暂时不接受新扫描点的数据
            RCLCPP_WARN(
                node_->get_logger(),
                "当前 scan point=%d 尚未完成，暂不切换到新的 scan point=%d。",
                active_scan_point_.scan_point_index,
                msg->scan_point_index);
            return; // 丢弃新扫描点的数据
        }

        if (msg->target_count == 0)
        {
            active_scan_point_.latest_frame_seq = msg->frame_seq;
            if (active_scan_point_.empty_targets_since.nanoseconds() == 0)
            {
                active_scan_point_.empty_targets_since = now;
            }
            return;
        }

        active_scan_point_.empty_targets_since =
            rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

        // 情况3：消息来自当前激活的扫描点，正常处理（继续执行）
        // ========== 第三步：更新扫描点的元数据 ==========
        // 记录最新收到的帧序号和时间戳，用于超时判断和状态管理
        active_scan_point_.latest_frame_seq = msg->frame_seq;    // 最新帧序号
        active_scan_point_.last_targets_msg_time = node_->now(); // 最后收到消息的时间

        // ========== 第四步：处理每个目标（增、改） ==========
        // 遍历消息中的所有目标，更新本地缓存的目标数据库
        for (const auto &target : msg->targets)
        {
            // 生成唯一标识当前目标的键
           // 键的组成：scan_point_index|label|label_instance_id
            const std::string key = makeVisionTargetKey(
                msg->scan_point_index,
                target.label,
                target.label_instance_id);

            // 在哈希表中查找该键是否已存在
            auto it = active_scan_point_.targets_by_key.find(key);

            if (it == active_scan_point_.targets_by_key.end())
            {
                // ========== 情况A：新目标（之前没见过） ==========
                // 创建新的目标条目（VisionTargetEntry）
                VisionTargetEntry entry;
                entry.key = key;                                    // 唯一键
                entry.scan_point_index = msg->scan_point_index;     // 所属扫描点
                entry.frame_seq = msg->frame_seq;                   // 帧序号
                entry.latest_frame_seq = msg->frame_seq;
                entry.label = target.label;                         // 标签（person/dog/cat等）
                entry.label_instance_id = target.label_instance_id; // 实例ID
                entry.target_msg = target;                          // 原始消息
                entry.state = VisionTargetState::PENDING;           // 初始状态：待处理
                entry.first_seen_time = node_->now();               // 首次出现时间
                entry.last_seen_time = node_->now();                // 最后出现时间

                // 添加到哈希表（用于快速查找）
                active_scan_point_.targets_by_key.emplace(key, entry);
                // 添加到顺序列表（用于按序处理，如FIFO）
                active_scan_point_.target_order.push_back(key);
            }
            else
            {
                // ========== 情况B：已存在的目标（之前见过） ==========
                // 更新目标数据，刷新最后出现时间
                it->second.frame_seq = msg->frame_seq;
                it->second.latest_frame_seq = msg->frame_seq;
                it->second.target_msg = target;           // 更新最新的目标消息
                it->second.last_seen_time = node_->now(); // 更新最后出现时间
            }
        }

        // ========== 第五步：自动选择下一个待跟踪目标 ==========
        // 如果没有正在跟踪的目标，从待处理队列中选择一个开始跟踪
        const bool executing_camera_aim = current_action_ && current_action_->getType() == ActionType::CAMERA_AIM;

        // 只有当前 mission 正在执行 camera_aim 动作时，才允许自动选择视觉目标。
        // 这样可以避免在 move / hover 等阶段提前把目标状态切到 TRACKING。
        if (executing_camera_aim && !hasCurrentTarget())
        {
            // 当前还没有正在处理的目标时，从目标队列中挑选下一个 PENDING 目标。
            VisionTargetEntry *next = pickNextPendingTarget();
            if (next != nullptr)
            {
                // 将选中的目标登记为当前目标，并切换到 TRACKING 状态，
                // 后续 executeCameraAim() 会基于这个目标的误差做对准控制。
                active_scan_point_.current_target_key = next->key;
                next->state = VisionTargetState::TRACKING;
                resetCameraAimTrackingState();

                // 输出目标切换日志，便于联调时确认 scan point 和目标选择是否正确。
                RCLCPP_INFO(
                    node_->get_logger(),
                    "已选择当前目标：scan_point=%d, key=%s, label=%s, instance_id=%u。已重置稳定计数和 PID 状态。",
                    next->scan_point_index,
                    next->key.c_str(),
                    next->label.c_str(),
                    next->label_instance_id);
            }
        }
    }

    bool isTerminalState(VisionTargetState state) const
    {
        return state == VisionTargetState::CAPTURED ||
               state == VisionTargetState::FAILED ||
               state == VisionTargetState::SKIPPED;
    }

    VisionTargetEntry *findTargetByResult(
        const drone_msgs::msg::K230RecordResult &msg)
    {
        for (auto &[key, target] : active_scan_point_.targets_by_key)
        {
            if (target.scan_point_index == msg.scan_point_index &&
                target.label == msg.label &&
                target.label_instance_id == msg.label_instance_id)
            {
                return &target;
            }
        }
        return nullptr;
    }

    void publishScanPointDone()
    {
        if (!active_scan_point_.has_active_context || active_scan_point_.scan_point_done_sent)
        {
            return;
        }

        drone_msgs::msg::K230ScanPointDone msg;
        msg.stamp = node_->now();
        msg.scan_point_index = active_scan_point_.scan_point_index;
        msg.scan_point_done = true;
        scan_point_done_pub_->publish(msg);

        active_scan_point_.scan_point_done_sent = true;
        RCLCPP_INFO(
            node_->get_logger(),
            "已发布 scan_point_done：scan_point=%d。",
            active_scan_point_.scan_point_index);
    }

    void tryFinishActiveScanPoint()
    {
        if (!active_scan_point_.has_active_context)
        {
            return;
        }

        if (active_scan_point_.targets_by_key.empty())
        {
            return;
        }

        for (const auto &[key, target] : active_scan_point_.targets_by_key)
        {
            if (!isTerminalState(target.state))
            {
                return;
            }
        }

        publishScanPointDone();
        resetActiveScanPoint();
    }

    void recordResultCallback(
        const drone_msgs::msg::K230RecordResult::SharedPtr msg)
    {
        if (!active_scan_point_.has_active_context)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "收到 record_result，但当前没有活动 scan point：scan_point=%d, label=%s, instance_id=%u。",
                msg->scan_point_index,
                msg->label.c_str(),
                msg->label_instance_id);
            return;
        }

        if (!isSameScanPoint(msg->scan_point_index))
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "收到非当前 scan point 的 record_result：current=%d, msg=%d。",
                active_scan_point_.scan_point_index,
                msg->scan_point_index);
            return;
        }

        VisionTargetEntry *target = findTargetByResult(*msg);
        if (target == nullptr)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "record_result 未匹配到目标：scan_point=%d, label=%s, instance_id=%u。",
                msg->scan_point_index,
                msg->label.c_str(),
                msg->label_instance_id);
            return;
        }

        if (target->state != VisionTargetState::CAPTURE_REQUESTED)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "目标当前状态不是 CAPTURE_REQUESTED，忽略本次结果：key=%s, state=%d。",
                target->key.c_str(),
                static_cast<int>(target->state));
            return;
        }
        target->image_name = msg->image_name;
        target->frame_seq = msg->frame_seq;
        target->latest_frame_seq = msg->frame_seq;

        if (msg->result_state == "captured")
        {
            target->state = VisionTargetState::CAPTURED;
        }
        else if (msg->result_state == "failed")
        {
            target->state = VisionTargetState::FAILED;
        }
        else if (msg->result_state == "skipped")
        {
            target->state = VisionTargetState::SKIPPED;
        }
        else
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "收到未知 result_state=%s，忽略本次 record_result。",
                msg->result_state.c_str());
            return;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "目标结果已更新：key=%s, result_state=%s, record_success=%s, image_name=%s。",
            target->key.c_str(),
            msg->result_state.c_str(),
            msg->record_success ? "true" : "false",
            msg->image_name.c_str());

        if (active_scan_point_.current_target_key == target->key &&
            isTerminalState(target->state))
        {
            active_scan_point_.current_target_key.clear();
            resetCameraAimTrackingState();
        }

        tryFinishActiveScanPoint();
    }

        std::string actionTypeToString(ActionType type) const
    {
        switch (type)
        {
        case ActionType::MOVE_TO_POSITION:
            return "move";
        case ActionType::HOVER:
            return "hover";
        case ActionType::CAMERA_AIM:
            return "camera_aim";
        case ActionType::LAND:
            return "land";
        case ActionType::TAKEOFF:
            return "takeoff";
        default:
            return "unknown";
        }
    }

    bool isSameScanPoint(int32_t scan_point_index) const
    {
        return active_scan_point_.has_active_context &&
               active_scan_point_.scan_point_index == scan_point_index;
    }

    void resetActiveScanPoint()
    {
        active_scan_point_.scan_point_index = -1;
        active_scan_point_.latest_frame_seq = 0;
        active_scan_point_.scan_point_x = 0.0;
        active_scan_point_.scan_point_y = 0.0;
        active_scan_point_.targets_by_key.clear();
        active_scan_point_.target_order.clear();
        active_scan_point_.current_target_key.clear();
        resetCameraAimTrackingState();
        active_scan_point_.last_targets_msg_time =
            rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        active_scan_point_.scan_point_start_time =
            rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        active_scan_point_.empty_targets_since =
            rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
        active_scan_point_.has_active_context = false;
        active_scan_point_.has_received_targets_frame = false;
        active_scan_point_.scan_point_done_sent = false;
    }

    bool hasCurrentTarget() const
    {
        return !active_scan_point_.current_target_key.empty() &&
               active_scan_point_.targets_by_key.find(active_scan_point_.current_target_key) !=
                   active_scan_point_.targets_by_key.end();
    }

    VisionTargetEntry *getCurrentTarget()
    {
        if (active_scan_point_.current_target_key.empty())
        {
            return nullptr;
        }

        auto it = active_scan_point_.targets_by_key.find(active_scan_point_.current_target_key);

        if (it == active_scan_point_.targets_by_key.end())
        {
            return nullptr;
        }

        return &it->second;
    }

    VisionTargetEntry *pickNextPendingTarget()
    {
        for (const std::string &key : active_scan_point_.target_order)
        {
            auto it = active_scan_point_.targets_by_key.find(key);
            if (it == active_scan_point_.targets_by_key.end())
            {
                continue;
            }

            if (it->second.state == VisionTargetState::PENDING)
            {
                return &it->second;
            }
        }

        return nullptr;
    }

    //- 进入 `move` 第一帧时初始化
    //- 记录起点、解析终点、提取起始 yaw 和目标 yaw
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer &tf_buffer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<drone_msgs::msg::K230CaptureReady>::SharedPtr capture_ready_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr step_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hover_active_pub_;
    rclcpp::Publisher<drone_msgs::msg::K230ScanPointDone>::SharedPtr scan_point_done_pub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr camera_aim_sub_;
    rclcpp::Subscription<drone_msgs::msg::K230AnimalTargets>::SharedPtr k230_targets_sub_;
    rclcpp::Subscription<drone_msgs::msg::K230RecordResult>::SharedPtr record_result_sub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;

    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Point camera_aim_diff_;
    rclcpp::Time last_camera_aim_time_;
    bool current_pose_received_ = false;
    rclcpp::Time last_status_publish_time_;
    bool force_status_publish_ = true;
    double status_publish_period_s_ = 1.0;
    double camera_aim_pid_p_ = 0.001;
    double camera_aim_pid_i_ = 0.0;
    double camera_aim_pid_d_ = 0.001;
    double camera_aim_target_timeout_s_ = 0.5;
    int camera_aim_stable_cycles_ = 20;
    double camera_aim_max_step_ = 0.05;
    double camera_aim_wait_first_targets_timeout_s_ = 2.0;
    double camera_aim_no_target_confirm_s_ = 2.0;
    double camera_aim_record_result_timeout_s_ = 5.0;
    double camera_aim_scan_point_timeout_s_ = 30.0;
    double land_setpoint_quiet_time_s_ = 0.2;
    bool vision_hover_active_ = false;
    bool vision_hover_active_published_ = false;
    Vec3dPID pid_cam_aim_;

    int aim_close_count_ = 0;
    bool land_mode_request_sent_ = false;
    bool land_setpoint_quiet_started_ = false;
    int land_low_altitude_count_ = 0;
    rclcpp::Time land_setpoint_quiet_start_time_;

    std::queue<std::shared_ptr<DroneAction>> action_queue_;
    std::shared_ptr<DroneAction> current_action_;

    geometry_msgs::msg::PoseStamped last_finish_pose_;
    bool last_finish_pose_initialized_ = false;

    int action_id_ = 0;
    ActiveScanPointContext active_scan_point_;
    MoveRuntimeState move_runtime_;
};
