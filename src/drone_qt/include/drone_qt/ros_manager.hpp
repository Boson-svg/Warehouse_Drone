#pragma once

#include <memory>
#include <thread>

#include <QObject>
#include <QString>
#include <QDateTime>
#include <QPoint>

#include <rclcpp/rclcpp.hpp>
#include <QByteArray>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "drone_qt/position_view_widget.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <QMap>

//自定义消息头文件
#include "drone_msgs/msg/drone_status.hpp"
#include "drone_msgs/msg/task_status.hpp"
#include "drone_msgs/msg/ready_status.hpp"
#include "drone_msgs/msg/grid_group.hpp"
#include "drone_msgs/msg/grid_point.hpp"
#include "drone_msgs/msg/world_group.hpp"
#include "drone_msgs/msg/world_point.hpp"
#include "drone_msgs/srv/start_task.hpp"
#include "drone_msgs/srv/start_offboard.hpp"
#include "drone_msgs/srv/upload_mission_yaml.hpp"
#include "drone_msgs/msg/barcode_capture.hpp"
#include "drone_msgs/msg/mission_summary.hpp"
#include "drone_msgs/srv/upload_mission_summary.hpp"

class RosManager : public QObject
{
    Q_OBJECT

    public:
        //构造函数和析构函数
        explicit RosManager(QObject *parent = nullptr);
        ~RosManager() override;

        //定义一个公共方法，用于启动ROS spinning线程
        void start();

        //定义一个公共方法，用于上传任务总结，包括路径点列表和任务总结信息
        void uploadMissionSummary(const QVector<WorldCoord> &path_points,
                          const drone_msgs::msg::MissionSummary &summary);

    signals:
        //定义一个信号，用于状态更新事件，包含连接状态、电量百分比、飞行模式和解锁状态等信息
        void statusUpdated(
            bool connected,
            float battery_percent,
            int flight_mode,
            bool armed,
            const QString &task_name);

        //更新控制程序action内容
        void action_statusUpdated(
            bool task_running,
            int action_step,
            int action_num,
            const QString &action_name);

        //控制程序返回的路线传输成功的信号
        void pathReadyChanged(bool ready);

        //定义一个信号，用于返回预规划路线的坐标点列表，参数为一个包含坐标点的QVector
        void returnWorldGroupUpdated(const QVector<WorldCoord> &points);

        //定义一个信号，用于命令执行结果事件，包含执行结果的成功与否以及相关消息
        void commandResult(bool success, const QString &message);
        void stopcommandResult(bool success, const QString &message);


        //定义一个信号，用于条形码捕获事件，包含捕获到的条形码数据
        void barcodeCaptured(
            const QString &barcode,
            const QByteArray &data,
            const QString &image_format,
            const QString &time_text);

        //定义一个信号，用于条形码捕获事件，不包含图像数据
        void visionBarcodeCaptured(
            const QString &barcode,
            const QString &time_text);

        //定义一个信号，用于位置更新事件，包含无人机的二维位置坐标与高度
        void positionUpdated(double x, double y, double z);
        void deltaUpdated(double dx, double dy, double dyaw, bool valid);

        //定义一个信号，用于告知是否上传了路线
        void pushFlagChanged(bool value);

        //机载端执行 offboard 启动服务后，把结果通知 UI
        void offboardCommandResult(bool success, const QString &message);

        //机载端执行第二段程序后，把结果通知 UI
        void takeoffProgramCommandResult(bool success, const QString &message);

        //机载端执行任务上传后，把结果通知 UI
        void missionUploadFinished(bool success, const QString &message, const QString &saved_path);

    public slots:
        //定义一个槽函数，用于启动任务
        void startTask();

        //定义一个槽函数，用于停止任务
        void stopTask();

        //用于请求启动offboard
        void requestStartOffboard();

    private:
        //负责设置ROS接口，如订阅、服务客户端等
        void setupRosInterfaces();

        //成员变量：一个标志表示是否已启动
        //一个ROS节点指针，两个状态订阅者，一个服务客户端，一个ROS执行器，以及一个线程用于ROS spinning
        bool started_{false};
        std::shared_ptr<rclcpp::Node> node_;
        rclcpp::Subscription<drone_msgs::msg::DroneStatus>::SharedPtr status_sub_;
        rclcpp::Subscription<drone_msgs::msg::TaskStatus>::SharedPtr task_status_sub_;
        rclcpp::Subscription<drone_msgs::msg::ReadyStatus>::SharedPtr ready_status_sub_;
        rclcpp::Subscription<drone_msgs::msg::WorldGroup>::SharedPtr return_world_group_sub_;
        rclcpp::Subscription<drone_msgs::msg::BarcodeCapture>::SharedPtr barcode_sub_;
        rclcpp::Subscription<drone_msgs::msg::BarcodeCapture>::SharedPtr vision_barcode_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr delta_sub_;
        rclcpp::Client<drone_msgs::srv::StartTask>::SharedPtr start_task_client_;
        rclcpp::Client<drone_msgs::srv::StartTask>::SharedPtr stop_push_client_;
        rclcpp::Client<drone_msgs::srv::StartOffboard>::SharedPtr start_offboard_client_;
        rclcpp::Client<drone_msgs::srv::UploadMissionSummary>::SharedPtr upload_mission_summary_client_;
        rclcpp::executors::SingleThreadedExecutor executor_;
        std::thread spin_thread_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string current_flight_mode{"UNKNOWN"};//当前飞行模式字符串
        std::string current_action_mode{"NO ACTION"};//当前动作字符串
        bool connected{false};//连接状态字符串
        int8_t flight_mode{0};//飞行模式数值
        bool armed{false};//解锁状态布尔值
        float battery_voltage{0.0};//电压数值
        float battery_percent{0.0};//电量百分比数值
        bool push_flag{false};//是否上传

        double latest_dx_ = 0.0;
        double latest_dy_ = 0.0;
        double latest_dyaw_ = 0.0;
        std::mutex delta_mutex_;
        std::chrono::steady_clock::time_point last_delta_msg_time_;
        bool has_delta_msg_{false};
};
