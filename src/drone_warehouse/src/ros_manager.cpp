#include "drone_warehouse/ros_manager.hpp"

/*********************ros移植部分***********************/
RosManager::RosManager(
    const RosTopicConfig &topic_config,
    QObject *parent)
        : QObject(parent),
        topic_config_(topic_config),
        node_(std::make_shared<rclcpp::Node>(
            topic_config_.node_name.toStdString()))
{
    // 这里沿用来源工程的做法：RosManager 一创建就把所有 ROS 订阅、服务客户端和定时器接口建好。
    // 后面 MainWindow 只需要负责调用 start() 启动 spin 线程，并接收这些 Qt 信号即可。
    setupRosInterfaces();
}
/******************************************************/

RosManager::~RosManager()
{
    //取消执行器的 spinning，并等待线程结束
    executor_.cancel();

    //如果线程可连接，则连接线程
    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

void RosManager::setupRosInterfaces()
{
    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅无人机状态话题，消息类型为自定义消息
    status_sub_ = node_->create_subscription<drone_msgs::msg::DroneStatus>(
        topic_config_.drone_status.toStdString(),
        status_qos,
        [this](const drone_msgs::msg::DroneStatus::SharedPtr msg)//回调
        {
            connected = msg->connected;
            battery_percent = msg->battery_percent;
            flight_mode = static_cast<int>(msg->flight_mode);
            armed = msg->armed;
            const QString task_name = QString::fromStdString(msg->state);

            //使用Qt的信号槽机制在线程安全的方式下发状态更新信号，包含连接状态、电量百分比、飞行模式和解锁状态等信息
            QMetaObject::invokeMethod(
                this,
                [this, task_name]() {
                    emit statusUpdated(
                        connected,
                        battery_percent,
                        flight_mode,
                        armed,
                        task_name);
                },
                Qt::QueuedConnection);
        });

    auto control_status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅控制程序状态话题，消息类型为自定义消息
    task_status_sub_ = node_->create_subscription<drone_msgs::msg::TaskStatus>(
        topic_config_.task_status.toStdString(),
        control_status_qos,
        [this](const drone_msgs::msg::TaskStatus::SharedPtr msg)//回调
        {
            const bool task_running = msg->task_running;
            const int action_step = msg->action_step;
            const int action_num = msg->action_num;
            const QString action_name = QString::fromStdString(msg->action_name);

            //使用Qt的信号槽机制在线程安全的方式下发状态更新信号，包含连接状态、电量百分比、飞行模式和解锁状态等信息
            QMetaObject::invokeMethod(
                this,
                [this, task_running, action_step, action_num, action_name]() {
                    emit action_statusUpdated(
                        task_running,
                        action_step,
                        action_num,
                        action_name);
                },
                Qt::QueuedConnection);
        });

    auto control_path_ready_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅控制程序的确认消息，消息类型为自定义消息
    ready_status_sub_ = node_->create_subscription<drone_msgs::msg::ReadyStatus>(
        topic_config_.path_ready.toStdString(),
        control_path_ready_qos,
        [this](const drone_msgs::msg::ReadyStatus::SharedPtr msg)//回调
        {
            const bool ready = msg->air_route_ready;

            QMetaObject::invokeMethod(
                this,
                [this, ready]() {
                    emit pathReadyChanged(ready);
                },
                Qt::QueuedConnection);
        });

    auto return_world_group_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅控制程序的路线消息，消息类型为自定义消息
    return_world_group_sub_ = node_->create_subscription<drone_msgs::msg::WorldGroup>(
        topic_config_.return_world_group.toStdString(),
        return_world_group_qos,
        [this](const drone_msgs::msg::WorldGroup::SharedPtr msg)//回调
        {
            QVector<WorldCoord> points;
            points.reserve(static_cast<int>(msg->points.size()));//预先分配空间以提高效率

            for (const auto &point : msg->points) {
                points.push_back(WorldCoord{point.x, point.y});
            }

            QMetaObject::invokeMethod(
                this,
                [this, points]() {
                    //参数为一个包含坐标点的QVector
                    emit returnWorldGroupUpdated(points);
                },
                Qt::QueuedConnection);
        });

    auto barcode_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅条形码捕获话题，消息类型为自定义消息
    barcode_sub_ = node_->create_subscription<drone_msgs::msg::BarcodeCapture>(
        topic_config_.barcode_capture.toStdString(),
        barcode_qos,
        [this](const drone_msgs::msg::BarcodeCapture::SharedPtr msg)
        {
            //将接收到的条形码捕获消息中的图像数据转换为QByteArray格式，以便在Qt界面中使用
            const QByteArray image_bytes(
                reinterpret_cast<const char*>(msg->image_data.data()),
                static_cast<int>(msg->image_data.size()));

            //使用Qt的信号槽机制在线程安全的方式下发条形码捕获信号，包含捕获到的条形码数据、图像数据、图像格式和时间文本等信息
            const QString barcode = QString::fromStdString(msg->barcode);
            const QString image_format = QString::fromStdString(msg->image_format);
            const qint64 time_ms =
                static_cast<qint64>(msg->stamp.sec) * 1000LL +
                static_cast<qint64>(msg->stamp.nanosec) / 1000000LL;
            const QString time_text =
                QDateTime::fromMSecsSinceEpoch(time_ms,Qt::LocalTime)
                .toString("yyyy-MM-dd hh:mm:ss");

            //使用QMetaObject::invokeMethod来在线程安全的方式下发信号，确保信号在Qt的事件循环中被正确处理
            QMetaObject::invokeMethod(
                this,
                [this, barcode, image_bytes, image_format, time_text]() {
                    emit barcodeCaptured(barcode, image_bytes, image_format, time_text);
                },
                Qt::QueuedConnection);
        });

    auto barcode_sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅条形码捕获话题，消息类型为自定义消息
    vision_barcode_sub_ = node_->create_subscription<drone_msgs::msg::BarcodeCapture>(
        topic_config_.vision_barcode.toStdString(),
        barcode_sub_qos,
        [this](const drone_msgs::msg::BarcodeCapture::SharedPtr msg)
        {
            //使用Qt的信号槽机制在线程安全的方式下发条形码捕获信号，包含捕获到的条形码数据、图像数据、图像格式和时间文本等信息
            const QString barcode = QString::fromStdString(msg->barcode);
            const qint64 time_ms =
                static_cast<qint64>(msg->stamp.sec) * 1000LL +
                static_cast<qint64>(msg->stamp.nanosec) / 1000000LL;
            const QString time_text =
                QDateTime::fromMSecsSinceEpoch(time_ms,Qt::LocalTime)
                .toString("yyyy-MM-dd hh:mm:ss");

            QMetaObject::invokeMethod(
                this,
                [this, barcode, time_text]() {
                    emit visionBarcodeCaptured(barcode, time_text);
                },
                Qt::QueuedConnection);
        });

    vision_servo_status_sub_ =
        node_->create_subscription<drone_msgs::msg::VisionServoStatus>(
            topic_config_.vision_servo_status.toStdString(),
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
            [this](const drone_msgs::msg::VisionServoStatus::SharedPtr msg)
            {
                if (!msg) {
                    return;
                }

                const QString state = QString::fromStdString(msg->state);
                const QString requested_target_id =
                    QString::fromStdString(msg->requested_target_id);
                const QString tracked_target_id =
                    QString::fromStdString(msg->tracked_target_id);
                const QString detail = QString::fromStdString(msg->detail);
                QString time_text;
                if (msg->stamp.sec != 0 || msg->stamp.nanosec != 0) {
                    const qint64 time_ms =
                        static_cast<qint64>(msg->stamp.sec) * 1000LL +
                        static_cast<qint64>(msg->stamp.nanosec) / 1000000LL;
                    time_text = QDateTime::fromMSecsSinceEpoch(
                        time_ms, Qt::LocalTime).toString("yyyy-MM-dd hh:mm:ss");
                }

                QMetaObject::invokeMethod(
                    this,
                    [this, active = msg->active, state, requested_target_id,
                     tracked_target_id, detail, time_text]() {
                        emit visionServoStatusUpdated(
                            active, state, requested_target_id,
                            tracked_target_id, detail, time_text);
                    },
                    Qt::QueuedConnection);
            });

    auto local_position_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅无人机本地位置话题，消息类型为geometry_msgs::msg::PoseStamped
    local_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_config_.local_position.toStdString(),
        local_position_qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            const double x = msg->pose.position.x;
            const double y = msg->pose.position.y;
            const double z = msg->pose.position.z;

            const double qx = msg->pose.orientation.x;
            const double qy = msg->pose.orientation.y;
            const double qz = msg->pose.orientation.z;
            const double qw = msg->pose.orientation.w;

            //使用Qt的信号槽机制在线程安全的方式下发位置更新信号，包含无人机的二维位置坐标和高度等信息
            QMetaObject::invokeMethod(
                this,
                [this, x, y, z, qx, qy, qz, qw]() {
                    emit positionUpdated(x, y, z, qx, qy, qz, qw);
                },
                Qt::QueuedConnection);
        });

    auto car_local_position_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅无人车本地位置话题，消息类型为geometry_msgs::msg::PoseStamped
    car_local_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/car/local_position",
        car_local_position_qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            const double x = msg->pose.position.x;
            const double y = msg->pose.position.y;
            const double z = msg->pose.position.z;

            const double qx = msg->pose.orientation.x;
            const double qy = msg->pose.orientation.y;
            const double qz = msg->pose.orientation.z;
            const double qw = msg->pose.orientation.w;

            //使用Qt的信号槽机制在线程安全的方式下发位置更新信号，包含无人机的二维位置坐标和高度等信息
            QMetaObject::invokeMethod(
                this,
                [this, x, y, z, qx, qy, qz, qw]() {
                    emit carpositionUpdated(x, y, z, qx, qy, qz, qw);
                },
                Qt::QueuedConnection);
        });

    // /keypad/s4_pressed 是小车 S4 的实时按键状态。地面站只转发状态，
    // 无人机任务仍由 Qt 主线程在 false -> true 的边沿上触发。
    car_keypad_s4_pressed_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/keypad/s4_pressed",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            if (!msg)
            {
                return;
            }

            QMetaObject::invokeMethod(
                this,
                [this, pressed = msg->data]() {
                    emit carKeypadS4PressedReceived(pressed);
                },
                Qt::QueuedConnection);
        });

    // 路线管理器发布的是短字符串状态。回调仍只负责转发，
    // 中文显示由 MainWindow 统一处理，避免通信层混入界面文案。
    car_route_state_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/route/state",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const std_msgs::msg::String::SharedPtr msg)
        {
            if (!msg)
            {
                return;
            }

            const QString state =
                QString::fromStdString(msg->data).trimmed().toUpper();
            QMetaObject::invokeMethod(
                this,
                [this, state]() {
                    emit carRouteStateReceived(state);
                },
                Qt::QueuedConnection);
        });

    car_control_mode_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/control/mode",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

auto delta_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    delta_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        topic_config_.pose_delta.toStdString(),
        delta_qos,
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(delta_mutex_);
            latest_dx_ = msg->x;
            latest_dy_ = msg->y;
            latest_dyaw_ = msg->z;
            last_delta_msg_time_ = std::chrono::steady_clock::now();//记录最后一次接收到delta消息的时间点
            has_delta_msg_ = true;//设置标志，表示已经接收到过delta消息
        });

    // Keep the latest complete settings for a vision node that starts later.
    industrial_camera_params_pub_ =
        node_->create_publisher<drone_msgs::msg::IndustrialCameraParams>(
            topic_config_.industrial_camera_params.toStdString(),
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    //创建一个服务客户端，用于调用任务启动服务
    start_task_client_ = node_->create_client<drone_msgs::srv::StartTask>(
        topic_config_.start_task_service.toStdString());

    //创建一个服务客户端，用于调用路线重传服务
    stop_push_client_ = node_->create_client<drone_msgs::srv::StartTask>(
        topic_config_.stop_push_service.toStdString());

    //创建一个服务客户端，用于调用offboard启动服务
    start_offboard_client_ = node_->create_client<drone_msgs::srv::StartOffboard>(
        topic_config_.start_offboard_service.toStdString());

    //创建一个服务客户端，用于调用任务路线与参数上传服务
    upload_mission_summary_client_ = node_->create_client<drone_msgs::srv::UploadMissionSummary>(
        topic_config_.upload_mission_service.toStdString());

    using namespace std::chrono_literals;
    timer_ = node_->create_wall_timer(
        0.5s,
        [this]()
        {
            double dx, dy, dyaw;
            bool delta_valid = false;
            {
                std::lock_guard<std::mutex> lock(delta_mutex_);//在访问共享数据时加锁，确保线程安全
                dx = latest_dx_;
                dy = latest_dy_;
                dyaw = latest_dyaw_;

                if (has_delta_msg_) {
                    const auto now = std::chrono::steady_clock::now();
                    //如果当前时间与最后一次接收到delta消息的时间点之间的间隔不超过1秒钟，则认为delta数据有效，否则认为无效
                    delta_valid = (now - last_delta_msg_time_) <= std::chrono::seconds(1);
                }
            }

            QMetaObject::invokeMethod(
                this,
                [this, dx, dy, dyaw, delta_valid]() {
                    emit deltaUpdated(dx, dy, dyaw, delta_valid);
                },
                Qt::QueuedConnection);
        });
}

void RosManager::start()
{
    //如果已经启动，则直接返回
    if (started_)
    {
        return;
    }

    //将ROS节点添加到执行器中，并启动一个线程来运行ROS spinning
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]()
    {
        executor_.spin();
    });

    started_ = true;
}

void RosManager::publishIndustrialCameraParams(const IndustrialCameraConfig &config)
{
    if (!industrial_camera_params_pub_)
    {
        return;
    }

    drone_msgs::msg::IndustrialCameraParams message;
    message.auto_exposure = config.auto_exposure;
    message.exposure_absolute = config.exposure_absolute;
    message.auto_exposure_priority = config.auto_exposure_priority;
    message.gain = config.gain;
    message.brightness = config.brightness;
    message.contrast = config.contrast;
    message.saturation = config.saturation;
    message.gamma = config.gamma;
    message.sharpness = config.sharpness;
    message.backlight_compensation = config.backlight_compensation;
    message.auto_white_balance = config.auto_white_balance;
    message.white_balance_temperature = config.white_balance_temperature;
    message.power_line_frequency = config.power_line_frequency;
    message.auto_focus = config.auto_focus;
    message.focus_absolute = config.focus_absolute;
    message.zoom_absolute = config.zoom_absolute;
    industrial_camera_params_pub_->publish(message);
}

/*********************ros移植部分***********************/

void RosManager::publishCarControlMode(const QString &mode)
{
    if (!car_control_mode_pub_)
    {
        return;
    }

    const QString normalized_mode = mode.trimmed().toUpper();
    if (normalized_mode != "DISABLED" && normalized_mode != "AUTO")
    {
        return;
    }

    std_msgs::msg::String message;
    message.data = normalized_mode.toStdString();
    car_control_mode_pub_->publish(message);
}
void RosManager::startTask()
{
    // 这一段严格沿用来源工程的语义：
    // 1. 调用配置中的 start_task 服务；
    // 2. 请求里的 task_name 固定写 action_task_run；
    // 3. 服务回包后，通过 commandResult 信号把结果回传给界面。
    if (!start_task_client_ || !start_task_client_->service_is_ready())
    {
        emit commandResult(false, QString("服务 %1 未就绪").arg(topic_config_.start_task_service));
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::StartTask::Request>();
    request->task_name = "action_task_run";//来源工程里的固定任务名，表示开始执行任务

    start_task_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::StartTask>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(
                this,
                [this, response]() {
                    emit commandResult(
                        response->success,
                        QString::fromStdString(response->message));
                },
                Qt::QueuedConnection);
        });
}
/******************************************************/

/*********************ros移植部分***********************/
void RosManager::stopTask()
{
    // 这一段也保持来源工程逻辑不变：
    // 1. 调用配置中的 stop_push 服务；
    // 2. 请求体里沿用 stop_task 这个 task_name；
    // 3. 用 stopcommandResult 把回包结果下发给界面。
    if (!stop_push_client_ || !stop_push_client_->service_is_ready())
    {
        emit stopcommandResult(false, QString("服务 %1 未就绪").arg(topic_config_.stop_push_service));
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::StartTask::Request>();
    request->task_name = "stop_task";//来源工程里的固定停止任务名

    stop_push_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::StartTask>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(
                this,
                [this, response]() {
                    emit stopcommandResult(
                        response->success,
                        QString::fromStdString(response->message));
                },
                Qt::QueuedConnection);
        });
}
/******************************************************/

/*********************ros移植部分***********************/
void RosManager::requestStartOffboard()
{
    // 这里保持来源工程的 offboard 启动逻辑：
    // 1. 调用配置中的 start_offboard 服务；
    // 2. request_source 固定写 ground_station_upload_flow；
    // 3. 用 offboardCommandResult 把执行结果回传给界面。
    if (!start_offboard_client_ || !start_offboard_client_->service_is_ready())
    {
        emit offboardCommandResult(false, QString("服务 %1 未就绪").arg(topic_config_.start_offboard_service));
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::StartOffboard::Request>();
    request->request_source = "ground_station_upload_flow";//沿用来源工程的固定请求来源标识

    start_offboard_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::StartOffboard>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(
                this,
                [this, response]() {
                    emit offboardCommandResult(
                        response->success,
                        QString::fromStdString(response->message));
                },
                Qt::QueuedConnection);
        });
}
/******************************************************/

/*********************ros移植部分***********************/
void RosManager::uploadMissionSummary(const QVector<WorldCoord> &path_points,
                                      const drone_msgs::msg::MissionSummary &summary)
{
    // 这是四个服务里依赖最多的一条链路，保持来源工程的行为：
    // 1. 调用配置中的 upload_mission 服务；
    // 2. 先检查服务是否就绪、路径是否为空；
    // 3. 请求里同时带 MissionSummary 和 WorldPoint 列表；
    // 4. 返回时把 success / message / saved_path 通过 missionUploadFinished 发给界面。
    if (!upload_mission_summary_client_ || !upload_mission_summary_client_->service_is_ready())
    {
        emit missionUploadFinished(
            false,
            QString("服务 %1 未就绪").arg(topic_config_.upload_mission_service),
            "");
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::UploadMissionSummary::Request>();
    request->summary = summary;//先把整份 mission 参数摘要写进请求体

    for (const auto &point : path_points)
    {
        drone_msgs::msg::WorldPoint world_point;
        world_point.x = point.x;
        world_point.y = point.y;
        world_point.z = point.z;
        world_point.yaw = point.yaw;
        request->points.push_back(world_point);//再把每一个世界坐标点追加进 points 列表
    }

    upload_mission_summary_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::UploadMissionSummary>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(
                this,
                [this, response]() {
                    emit missionUploadFinished(
                        response->success,
                        QString::fromStdString(response->message),
                        QString::fromStdString(response->saved_path));
                },
                Qt::QueuedConnection);
        });
}
/******************************************************/
