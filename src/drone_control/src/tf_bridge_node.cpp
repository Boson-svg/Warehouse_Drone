#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <functional>
#include <memory>

#include "drone_control/sliding_window_average.h"

namespace offboard_run {
class BodyToEnuBridge {
    public:
    explicit BodyToEnuBridge(const rclcpp::Node::SharedPtr &node) 
                            : node_(node),
                            tf_buffer_(node_->get_clock()),
                            tf_listener_(tf_buffer_, node_, true),
                            static_tf_broadcaster_(node_),
                            sliding_window_(kSlidingWindowSize){
                            pub_world_pose_ = node_->declare_parameter<bool>("pub_world_pose", false);

                            tf_status_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/mavros_tf/status", 10);

                            px4_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                                "/mavros/local_position/pose", rclcpp::SensorDataQoS(),
                                std::bind(&BodyToEnuBridge::px4Pose_callback, this, std::placeholders::_1));

                            reset_server_ = node_->create_service<std_srvs::srv::Empty>(
                                "/reset_world_tf", std::bind(&BodyToEnuBridge::resetTransform_callback,
                                    this, std::placeholders::_1, std::placeholders::_2));
                            
                            if (pub_world_pose_) {
                                current_world_body_pos_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>
                                ("/current_world_body_pos", 10);
                            }

                            status_timer_ = node_->create_wall_timer(
                                            std::chrono::milliseconds(100),
                                            std::bind(&BodyToEnuBridge::timer_callback, this));
                            
                            RCLCPP_INFO(node_->get_logger(),
                                        "TF 桥接节点已启动，发布 world_body 位姿：%s",
                                        pub_world_pose_ ? "是" :"否");

    }

    private:
    static constexpr int kSlidingWindowSize = 10;

    void px4Pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        current_pose_.header.frame_id = "world_enu";
        received_pose_ = true;

        if (!tf_ready_) {
            sliding_window_.addPose(current_pose_);
            return;
        }

        if(!pub_world_pose_) {
            return;
        }

        try {
            auto current_world_body = tf_buffer_.transform(
                current_pose_, "world_body");
            current_world_body_pos_pub_->publish(current_world_body);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(node_->get_logger(), "本地位姿坐标变换失败：%s", ex.what());
        }
    }

    void resetTransform_callback (
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>) {
        tf_ready_ = false;
        sliding_window_.clear();
        RCLCPP_INFO(node_->get_logger(),
                    "世界坐标变换已重置，正在等待位姿样本。");
    }

    void timer_callback() {
        if (sliding_window_.getSize() >= kSlidingWindowSize && !tf_ready_) {
            initializeTransform();
            static_tf_broadcaster_.sendTransform(world_enu_to_world_body_);
            tf_ready_ = true;
            RCLCPP_INFO(node_->get_logger(), "已初始化坐标变换：world_enu -> world_body");
        }

        std_msgs::msg::Bool status;
        status.data = tf_ready_;
        tf_status_pub_->publish(status);
    }

    void initializeTransform() {
        (void)received_pose_;
        const auto start_pose = sliding_window_.computeAveragePose();
        world_enu_to_world_body_.header.stamp = node_->now();
        world_enu_to_world_body_.header.frame_id = "world_enu";
        world_enu_to_world_body_.child_frame_id = "world_body";
        world_enu_to_world_body_.transform.translation.x = start_pose.pose.position.x;
        world_enu_to_world_body_.transform.translation.y = start_pose.pose.position.y;
        world_enu_to_world_body_.transform.translation.z = start_pose.pose.position.z;
        world_enu_to_world_body_.transform.rotation = start_pose.pose.orientation;
    }
    
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tf_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_world_body_pos_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr px4_pose_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_server_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    SlidingWindowPoseAverage sliding_window_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TransformStamped world_enu_to_world_body_;
    bool tf_ready_ = false;
    bool received_pose_ = false;
    bool pub_world_pose_ = false;
};
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("tf_bridge_node");
    auto bridge = std::make_shared<offboard_run::BodyToEnuBridge>(node);
    (void)bridge;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
