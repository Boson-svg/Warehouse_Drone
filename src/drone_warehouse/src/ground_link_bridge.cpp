#include "drone_warehouse/ground_link_bridge.hpp"

#include <QCoreApplication>
#include <QDataStream>
#include <QEventLoop>

#include <limits>

namespace lp = drone_msgs::link_protocol;

namespace
{
    // 这些是通信层保护上限，不是仓库业务参数。损坏帧中的长度字段
    // 可能被解析成几万字节，如果不限制，接收端会一直等待并表现为断链。
    constexpr int kMaxPayloadSize = 4096;
    constexpr int kSerialWatchdogIntervalMs = 1000;
    constexpr int kSerialInactivityTimeoutMs = 5000;

    // 告诉 QDataStream：多字节数字一律按低字节在前的小端顺序读写。
    // 每创建一个新的 QDataStream，都要先调用一次这个函数。
    void configureStream(QDataStream &stream)
    {
        stream.setByteOrder(QDataStream::LittleEndian);
    }

    // 从调用位置开始，后续浮点数按 float32（4 字节）读写，直到再次切换精度。
    void useSinglePrecision(QDataStream &stream)
    {
        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    }

    // 从调用位置开始，后续浮点数按 float64（8 字节）读写，直到再次切换精度。
    void useDoublePrecision(QDataStream &stream)
    {
        stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
    }

    // 把一段字符串或二进制数据写成：[uint16 长度][对应长度的数据内容]。
    // 接收端用 readSizedBytes() 按同样格式读取，避免手工计算字符串偏移位置。
    bool writeSizedBytes(QDataStream &stream, const QByteArray &bytes)
    {
        if (bytes.size() > std::numeric_limits<quint16>::max()) {
            stream.setStatus(QDataStream::WriteFailed);
            return false;
        }

        stream << static_cast<quint16>(bytes.size());
        if (!bytes.isEmpty() && stream.writeRawData(bytes.constData(), bytes.size()) != bytes.size()) {
            stream.setStatus(QDataStream::WriteFailed);
            return false;
        }
        return stream.status() == QDataStream::Ok;
    }

    // 读取 writeSizedBytes() 写出的数据：先取 uint16 长度，再读取该长度的数据内容。
    // 这里只检查当前这一段数据是否完整；后面是否还有字段，由调用者继续读取。
    bool readSizedBytes(QDataStream &stream, QByteArray &bytes)
    {
        quint16 size = 0;
        stream >> size;
        if (stream.status() != QDataStream::Ok || !stream.device() ||
            stream.device()->bytesAvailable() < size) {
            stream.setStatus(QDataStream::ReadPastEnd);
            return false;
        }

        bytes.resize(size);
        if (size > 0 && stream.readRawData(bytes.data(), size) != size) {
            stream.setStatus(QDataStream::ReadPastEnd);
            return false;
        }
        return true;
    }

    // 完整 payload 的所有字段都读完后调用。
    // 返回 true 表示读取过程没有报错，而且 payload 没有缺少或多出任何字节。
    bool streamFullyConsumed(const QDataStream &stream)
    {
        return stream.status() == QDataStream::Ok && stream.device() &&
               stream.device()->bytesAvailable() == 0;
    }

    // StartOffboard、StartTask、StopPush 的响应内容完全相同：
    // [uint8 是否成功][uint16 提示文字长度][提示文字]。
    // 三个响应处理函数都通过这里读取，避免分别写三遍相同的解析步骤。
    bool decodeSimpleResponse(
        const QByteArray &payload, quint8 &success, QByteArray &message)
    {
        QDataStream stream(payload);
        configureStream(stream);
        stream >> success;
        return readSizedBytes(stream, message) && streamFullyConsumed(stream);
    }

    // 计算协议帧尾部的 CRC16。接收端重新计算后与帧尾数值比较，
    // 用来发现串口传输过程中出现的字节损坏。
    uint16_t crc16_ccitt(const uint8_t *data, int length)
    {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
                } else {
                    crc = static_cast<uint16_t>(crc << 1);
                }
            }
        }
        return crc;
    }
}

GroundLinkBridge::GroundLinkBridge(
    const SerialPortConfig &serial_config)
    : QObject(),
      rclcpp::Node("ground_link_bridge"),
      serial_config_(serial_config)
{
    setupRosInterfaces();
    setupSerial();
}

void GroundLinkBridge::setupRosInterfaces()
{
    /*************** 创建与机载端相对应的通信接口 ***************/
    status_pub_ = this->create_publisher<drone_msgs::msg::DroneStatus>(
        "/serial/drone/status", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    task_status_pub_ = this->create_publisher<drone_msgs::msg::TaskStatus>(
        "/serial/drone/task/status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    path_ready_pub_ = this->create_publisher<drone_msgs::msg::ReadyStatus>(
        "/serial/drone/control/path_ready", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    return_world_group_pub_ = this->create_publisher<drone_msgs::msg::WorldGroup>(
        "/serial/drone/return/world_group", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    vision_barcode_pub_ = this->create_publisher<drone_msgs::msg::BarcodeCapture>(
        "/serial/drone/vision/barcode", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    vision_servo_status_pub_ =
        this->create_publisher<drone_msgs::msg::VisionServoStatus>(
            "/serial/control/vision_servo/status",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());

    delta_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/serial/drone/pose_yaw_compare/delta", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    local_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/serial/drone/local_position", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    // 数传模式下接收地面站发布的完整参数，并通过可靠串口帧发往机载端。
    industrial_camera_params_sub_ =
        this->create_subscription<drone_msgs::msg::IndustrialCameraParams>(
            "/serial/industrial_camera/params",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            [this](const drone_msgs::msg::IndustrialCameraParams::SharedPtr message) {
                if (!message) {
                    return;
                }
                const QByteArray payload = encodeIndustrialCameraParams(*message);
                if (payload.isEmpty()) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "failed to encode industrial camera parameters");
                    return;
                }
                sendPacket(lp::kTypeIndustrialCameraParams,
                           lp::kFlagNeedAck, payload, true);
            });

    upload_mission_summary_srv_ = this->create_service<drone_msgs::srv::UploadMissionSummary>(
        "/serial/drone/upload_mission_summary",
        [this](
        const std::shared_ptr<drone_msgs::srv::UploadMissionSummary::Request> request,
        std::shared_ptr<drone_msgs::srv::UploadMissionSummary::Response> response)
        {
            if (!request) {
                response->success = false;
                response->message = "请求为空";
                response->saved_path = "";
                response->action_count = 0;
                return;
            }
            // 空地协同故意发送“零路线点 + 不压缩”请求。机载端收到后直接使用
            // 固定的 ground_station.yaml，因此这里不能把空路线当作错误拦截。
            //调用服务后将数据填入
            const QByteArray payload = encodeUploadMissionSummaryRequest(*request, next_msg_id_++);
            //发送协议帧到串口，并返回生成的序列号
            const uint16_t seq = sendPacket(lp::kTypeUploadMissionSummaryReq, lp::kFlagNeedAck, payload, true);

            pending_upload_calls_[seq] = PendingUploadMissionSummaryCall{};

            const auto deadline = this->now() + rclcpp::Duration::from_seconds(8.0);
            while (rclcpp::ok()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
                onRetryTimer();

                auto it = pending_upload_calls_.find(seq);
                if (it != pending_upload_calls_.end() && it->second.done) {
                    response->success = it->second.success;
                    response->message = it->second.message;
                    response->saved_path = it->second.saved_path;
                    response->action_count = it->second.action_count;
                    pending_upload_calls_.erase(it);
                    return;
                }

                if (this->now() > deadline) {
                    response->success = false;
                    response->message = "等待 UploadMissionSummaryResp 超时";
                    response->saved_path = "";
                    response->action_count = 0;
                    pending_upload_calls_.erase(seq);
                    return;
                }
            }

            response->success = false;
            response->message = "节点停止，UploadMissionSummary 未完成";
            response->saved_path = "";
            response->action_count = 0;
            pending_upload_calls_.erase(seq);
        });

    start_offboard_srv_ = this->create_service<drone_msgs::srv::StartOffboard>(
        "/serial/drone/start_offboard",
        [this](
        const std::shared_ptr<drone_msgs::srv::StartOffboard::Request> request,
        std::shared_ptr<drone_msgs::srv::StartOffboard::Response> response)
        {
            if (!request) {
                response->success = false;
                response->message = "空请求";
                return;
            }

            const QByteArray payload = encodeStartOffboardRequest(*request);
            const uint16_t seq = sendPacket(lp::kTypeStartOffboardReq, lp::kFlagNeedAck, payload, true);

            pending_start_offboard_calls_[seq] = PendingStartOffboardCall{};

            const auto deadline = this->now() + rclcpp::Duration::from_seconds(5.0);
            while (rclcpp::ok()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
                onRetryTimer();

                auto it = pending_start_offboard_calls_.find(seq);
                if (it != pending_start_offboard_calls_.end() && it->second.done) {
                    response->success = it->second.success;
                    response->message = it->second.message;
                    pending_start_offboard_calls_.erase(it);
                    return;
                }

                if (this->now() > deadline) {
                    response->success = false;
                    response->message = "等待 StartOffboardResp 超时";
                    pending_start_offboard_calls_.erase(seq);
                    return;
                }
            }

            response->success = false;
            response->message = "节点停止，StartOffboard 未完成";
            pending_start_offboard_calls_.erase(seq);
        });

    start_task_srv_ = this->create_service<drone_msgs::srv::StartTask>(
        "/serial/drone/start_task",
        [this](
        const std::shared_ptr<drone_msgs::srv::StartTask::Request> request,
        std::shared_ptr<drone_msgs::srv::StartTask::Response> response)
        {
            if (!request) {
                response->success = false;
                response->message = "空请求";
                return;
            }

            const QByteArray payload = encodeStartTaskRequest(*request);
            const uint16_t seq = sendPacket(lp::kTypeStartTaskReq, lp::kFlagNeedAck, payload, true);

            pending_start_task_calls_[seq] = PendingStartTaskCall{};

            const auto deadline = this->now() + rclcpp::Duration::from_seconds(5.0);
            while (rclcpp::ok()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
                onRetryTimer();

                auto it = pending_start_task_calls_.find(seq);
                if (it != pending_start_task_calls_.end() && it->second.done) {
                    response->success = it->second.success;
                    response->message = it->second.message;
                    pending_start_task_calls_.erase(it);
                    return;
                }

                if (this->now() > deadline) {
                    response->success = false;
                    response->message = "等待 StartTaskResp 超时";
                    pending_start_task_calls_.erase(seq);
                    return;
                }
            }

            response->success = false;
            response->message = "节点停止，StartTask 未完成";
            pending_start_task_calls_.erase(seq);
        });

    stop_push_srv_ = this->create_service<drone_msgs::srv::StartTask>(
        "/serial/drone/stop_push",
        [this](
        const std::shared_ptr<drone_msgs::srv::StartTask::Request> request,
        std::shared_ptr<drone_msgs::srv::StartTask::Response> response)
        {
            if (!request) {
                response->success = false;
                response->message = "空请求";
                return;
            }

            const QByteArray payload = encodeStopPushRequest(*request);
            const uint16_t seq = sendPacket(lp::kTypeStopPushReq, lp::kFlagNeedAck, payload, true);

            pending_stop_push_calls_[seq] = PendingStopPushCall{};

            const auto deadline = this->now() + rclcpp::Duration::from_seconds(5.0);
            while (rclcpp::ok()) {
                QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
                onRetryTimer();

                auto it = pending_stop_push_calls_.find(seq);
                if (it != pending_stop_push_calls_.end() && it->second.done) {
                    response->success = it->second.success;
                    response->message = it->second.message;
                    pending_stop_push_calls_.erase(it);
                    return;
                }

                if (this->now() > deadline) {
                    response->success = false;
                    response->message = "等待 StopPushResp 超时";
                    pending_stop_push_calls_.erase(seq);
                    return;
                }
            }

            response->success = false;
            response->message = "节点停止，StopPush 未完成";
            pending_stop_push_calls_.erase(seq);
        });
    /************************************************************/
}


/************************ 用户层 *************************/
void GroundLinkBridge::setupSerial()
{
    serial_.setPortName(serial_config_.port_name);
    serial_.setBaudRate(serial_config_.baud_rate);
    serial_.setDataBits(serial_config_.data_bits);
    serial_.setParity(serial_config_.parity);
    serial_.setStopBits(serial_config_.stop_bits);
    serial_.setFlowControl(serial_config_.flow_control);

    QObject::connect(&serial_, &QSerialPort::readyRead, [this]() {
        onSerialReadyRead();
    });
    QObject::connect(
        &serial_, &QSerialPort::errorOccurred,
        [this](QSerialPort::SerialPortError error) {
            onSerialError(error);
        });
    QObject::connect(&serial_watchdog_timer_, &QTimer::timeout, [this]() {
        checkSerialHealth();
    });

    openSerial();
    serial_watchdog_timer_.start(kSerialWatchdogIntervalMs);
}

bool GroundLinkBridge::openSerial()
{
    if (serial_.isOpen()) {
        return true;
    }

    serial_.clearError();
    if (!serial_.open(QIODevice::ReadWrite)) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "failed to open serial port %s: %s; retrying",
            serial_config_.port_name.toStdString().c_str(),
            serial_.errorString().toStdString().c_str());
        return false;
    }

    rx_buffer_.clear();
    last_valid_packet_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
        this->get_logger(), "serial port opened: %s, baud=%d",
        serial_config_.port_name.toStdString().c_str(),
        serial_config_.baud_rate);
    return true;
}

void GroundLinkBridge::onSerialError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError ||
        error == QSerialPort::TimeoutError ||
        serial_reopen_in_progress_) {
        return;
    }

    RCLCPP_WARN(
        this->get_logger(), "serial port error %d: %s; scheduling reconnect",
        static_cast<int>(error),
        serial_.errorString().toStdString().c_str());

    serial_reopen_in_progress_ = true;
    if (serial_.isOpen()) {
        serial_.close();
    }
    rx_buffer_.clear();
    last_valid_packet_time_ = std::chrono::steady_clock::now();
    serial_reopen_in_progress_ = false;
}

void GroundLinkBridge::checkSerialHealth()
{
    if (!serial_.isOpen()) {
        openSerial();
        return;
    }

    const auto silent_for_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_valid_packet_time_).count();
    if (silent_for_ms < kSerialInactivityTimeoutMs) {
        return;
    }

    // 无人机状态是周期帧。5 秒没有任何有效帧时，重开串口可以恢复
    // USB 短暂掉线后留下的旧句柄，也可以清理已经失步的接收缓冲区。
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 30000,
        "no valid serial frame for %lld ms; reopening %s",
        static_cast<long long>(silent_for_ms),
        serial_config_.port_name.toStdString().c_str());

    serial_reopen_in_progress_ = true;
    serial_.close();
    rx_buffer_.clear();
    serial_reopen_in_progress_ = false;
    openSerial();
}

void GroundLinkBridge::onSerialReadyRead()
{
    rx_buffer_.append(serial_.readAll());

    //接收到数据后，进入第一层handlePacket处理函数，后续分路
    Packet packet;
    while (tryParseOnePacket(packet)) {
        last_valid_packet_time_ = std::chrono::steady_clock::now();
        handlePacket(packet);
    }
}

uint16_t GroundLinkBridge::sendPacket(uint8_t type, uint8_t flags, const QByteArray &payload, bool need_ack)
{
    const uint16_t seq = nextSeq();
    //将消息类型、标志位、序列号和载荷数据编码成一个完整的帧数据，准备发送
    const QByteArray frame = encodeFrame(type, flags, seq, payload);
    //发送协议帧到串口
    serial_.write(frame);

    //如果需要ACK，则将请求信息保存到待重试列表中，以便后续检查和重试
    if (need_ack) {
        PendingRequest pending;
        pending.request_type = type;
        pending.seq = seq;
        pending.retry_count = 0;
        pending.encoded_frame = frame;
        pending.deadline = this->now() + rclcpp::Duration::from_seconds(0.3);
        pending_requests_[seq] = pending;
    }
    return seq;
}

void GroundLinkBridge::handlePacket(const Packet &packet)
{
    //如果收到的是ACK帧，则调用handleAck函数处理，并返回
    if (packet.type == lp::kTypeAck) {
        handleAck(packet.seq);
        return;
    }

    //如果收到的帧需要ACK，则先发送ACK帧，通知对方已经收到消息，避免对方继续重试
    if (packet.flags & lp::kFlagNeedAck) {
        sendAck(packet.seq);
    }

    //根据帧的类型调用不同的处理逻辑，并发送相应的ACK或响应数据
    switch (packet.type) {
    case lp::kTypeDroneStatus:
        handleDroneStatusReport(packet.payload);
        break;
    case lp::kTypePathReady:
        handlePathReadyReport(packet.payload);
        break;
    case lp::kTypeTaskStatus:
        handleTaskStatusReport(packet.payload);
        break;
    case lp::kTypeReturnWorldGroup:
        handleReturnWorldGroupReport(packet.payload);
        break;
    case lp::kTypeVisionBarcode:
        handleVisionBarcodeReport(packet.payload);
        break;
    case lp::kTypeVisionServoStatus:
        handleVisionServoStatusReport(packet.payload);
        break;
    case lp::kTypeDelta:
        handleDeltaReport(packet.payload);
        break;
    case lp::kTypeLocalPosition:
        handleLocalPositionReport(packet.payload);
        break;
    
    case lp::kTypeUploadMissionSummaryResp:
        handleUploadMissionSummaryResponse(packet.seq, packet.payload);
        break;
    case lp::kTypeStartOffboardResp:
        handleStartOffboardResponse(packet.seq, packet.payload);
        break;
    case lp::kTypeStartTaskResp:
        handleStartTaskResponse(packet.seq, packet.payload);
        break;
    case lp::kTypeStopPushResp:
        handleStopPushResponse(packet.seq, packet.payload);
        break;
    default:
        RCLCPP_WARN(this->get_logger(), "unknown packet type: 0x%02X", packet.type);
        break;
    }
}

void GroundLinkBridge::sendAck(uint16_t seq)
{
    //发送ACK帧到串口，通知对方之前发送的消息已经收到，无需再重试
    const QByteArray empty_payload;
    const QByteArray frame = encodeFrame(lp::kTypeAck, lp::kFlagAck, seq, empty_payload);
    serial_.write(frame);
}

void GroundLinkBridge::handleAck(uint16_t seq)
{
    //收到ACK帧后，从待重试列表中删除对应的请求信息，表示请求已经成功完成，不需要再重试
    pending_requests_.erase(seq);
}

uint16_t GroundLinkBridge::nextSeq()
{
    //生成一个新的序列号，确保每个请求都有一个唯一的标识符，便于匹配响应和处理重试逻辑
    return next_seq_++;
}






/******************* 数据处理层(发送层) ********************/

QByteArray GroundLinkBridge::encodeUploadMissionSummaryRequest(
    const drone_msgs::srv::UploadMissionSummary::Request &request,
    uint16_t msg_id) const
{
    Q_UNUSED(msg_id);

    // 请求内容按以下顺序装入 payload：点数量、所有路线点、任务参数、frame 字符串。
    // 机载端 handleUploadMissionSummaryRequest() 必须按照同样顺序逐项读取。
    QByteArray payload;//创建一块空字节缓冲区
    QDataStream stream(&payload, QIODevice::WriteOnly);//创建一个数据流，让你可以方便地把整数、浮点数按二进制写到 payload
    configureStream(stream);

    const auto &points = request.points;
    if (points.size() > std::numeric_limits<quint16>::max()) {
        return {};
    }
    stream << static_cast<quint16>(points.size());//先写点的数量
    useSinglePrecision(stream);//路线点按照float32写入
    for (const auto &point : points) {
        stream << static_cast<float>(point.x);
        stream << static_cast<float>(point.y);
        stream << static_cast<float>(point.z);
        stream << static_cast<float>(point.yaw);
    }
    const auto &summary = request.summary;
    useDoublePrecision(stream);
    stream << static_cast<double>(summary.takeoff_altitude);
    stream << static_cast<double>(summary.move_altitude);
    stream << static_cast<double>(summary.start_altitude);
    stream << static_cast<double>(summary.yaw);
    stream << static_cast<double>(summary.tolerance);
    stream << static_cast<double>(summary.yaw_tolerance_deg);
    stream << static_cast<double>(summary.max_xy_speed_mps);
    stream << static_cast<double>(summary.max_z_speed_mps);
    stream << static_cast<double>(summary.max_yaw_rate_deg_s);
    stream << static_cast<double>(summary.takeoff_hover_duration);
    stream << static_cast<double>(summary.landing_hover_duration);
    stream << static_cast<double>(summary.move_hover_duration);

    stream << static_cast<quint8>(summary.add_hover_between_takeoff ? 1 : 0);
    stream << static_cast<quint8>(summary.add_hover_between_landing ? 1 : 0);
    stream << static_cast<quint8>(summary.add_hover_between_moves ? 1 : 0);
    stream << static_cast<quint8>(summary.auto_start_mission ? 1 : 0);
    stream << static_cast<quint8>(summary.compress_straight_segments ? 1 : 0);

    // Variable-length strings are length-prefixed to keep the binary layout deterministic.
    const auto &visual = summary.visual_servo;
    if (!writeSizedBytes(stream, QByteArray::fromStdString(summary.frame)) ||
        !writeSizedBytes(stream, QByteArray::fromStdString(visual.target_id))) {
        return {};
    }

    stream << static_cast<quint8>(summary.visual_servo.enabled ? 1 : 0);
    stream << static_cast<quint8>(visual.require_confirmed ? 1 : 0);
    if (!writeSizedBytes(stream, QByteArray::fromStdString(visual.image_x_axis)) ||
        !writeSizedBytes(stream, QByteArray::fromStdString(visual.image_y_axis))) {
        return {};
    }

    stream << static_cast<double>(visual.image_x_sign);
    stream << static_cast<double>(visual.image_y_sign);
    stream << static_cast<double>(visual.kp_x);
    stream << static_cast<double>(visual.ki_x);
    stream << static_cast<double>(visual.kd_x);
    stream << static_cast<double>(visual.kp_y);
    stream << static_cast<double>(visual.ki_y);
    stream << static_cast<double>(visual.kd_y);
    stream << static_cast<double>(visual.integral_limit);
    stream << static_cast<double>(visual.filter_alpha);
    stream << static_cast<double>(visual.enter_tolerance_x);
    stream << static_cast<double>(visual.enter_tolerance_y);
    stream << static_cast<double>(visual.exit_tolerance_x);
    stream << static_cast<double>(visual.exit_tolerance_y);
    stream << static_cast<double>(visual.settle_time_s);
    stream << static_cast<double>(visual.acquire_timeout_s);
    stream << static_cast<double>(visual.lost_timeout_s);
    stream << static_cast<double>(visual.overall_timeout_s);
    stream << static_cast<double>(visual.max_body_speed_mps);
    stream << static_cast<quint8>(visual.continue_on_timeout ? 1 : 0);

    if (stream.status() != QDataStream::Ok) {
        return {};
    }
    return payload;
}

QByteArray GroundLinkBridge::encodeStartOffboardRequest(
    const drone_msgs::srv::StartOffboard::Request &request) const
{
    //编码函数，用于将ROS服务请求对象编码成协议帧的载荷格式，准备发送给机载端
    QByteArray payload;//新建一个空的字节数组
    QDataStream stream(&payload, QIODevice::WriteOnly);//写进payload
    configureStream(stream);

    //取出请求里的字符串字段
    const QByteArray source = QByteArray::fromStdString(request.request_source);
    if (!writeSizedBytes(stream, source)) {
        return {};
    }
    //输出：[字符串长度][字符串原始字节]
    return payload;
}

QByteArray GroundLinkBridge::encodeStartTaskRequest(
    const drone_msgs::srv::StartTask::Request &request) const
{
    //编码函数，用于将ROS服务请求对象编码成协议帧的载荷格式，准备发送给机载端
    QByteArray payload;//新建一个空的字节数组
    QDataStream stream(&payload, QIODevice::WriteOnly);//写进payload
    configureStream(stream);

    //取出请求里的字符串字段
    const QByteArray task_name = QByteArray::fromStdString(request.task_name);
    if (!writeSizedBytes(stream, task_name)) {
        return {};
    }
    //输出：[字符串长度][字符串原始字节]
    return payload;
}

QByteArray GroundLinkBridge::encodeIndustrialCameraParams(
    const drone_msgs::msg::IndustrialCameraParams &message) const
{
    QByteArray payload;
    QDataStream stream(&payload, QIODevice::WriteOnly);
    configureStream(stream);

    // 固定字段顺序必须与 AirborneLinkBridge 的解码顺序完全一致。
    stream << static_cast<quint8>(message.auto_exposure ? 1 : 0);
    stream << static_cast<qint32>(message.exposure_absolute);
    stream << static_cast<quint8>(message.auto_exposure_priority ? 1 : 0);
    stream << static_cast<qint32>(message.gain);
    stream << static_cast<qint32>(message.brightness);
    stream << static_cast<qint32>(message.contrast);
    stream << static_cast<qint32>(message.saturation);
    stream << static_cast<qint32>(message.gamma);
    stream << static_cast<qint32>(message.sharpness);
    stream << static_cast<qint32>(message.backlight_compensation);
    stream << static_cast<quint8>(message.auto_white_balance ? 1 : 0);
    stream << static_cast<qint32>(message.white_balance_temperature);
    stream << static_cast<quint8>(message.power_line_frequency);
    stream << static_cast<quint8>(message.auto_focus ? 1 : 0);
    stream << static_cast<qint32>(message.focus_absolute);
    stream << static_cast<qint32>(message.zoom_absolute);

    return stream.status() == QDataStream::Ok ? payload : QByteArray();
}
QByteArray GroundLinkBridge::encodeStopPushRequest(
    const drone_msgs::srv::StartTask::Request &request) const
{
    //编码函数，用于将ROS服务请求对象编码成协议帧的载荷格式，准备发送给机载端
    QByteArray payload;//新建一个空的字节数组
    QDataStream stream(&payload, QIODevice::WriteOnly);//写进payload
    configureStream(stream);

    //取出请求里的字符串字段
    const QByteArray task_name = QByteArray::fromStdString(request.task_name);
    if (!writeSizedBytes(stream, task_name)) {
        return {};
    }
    //输出：[字符串长度][字符串原始字节]
    return payload;
}








/******************* 数据处理层(接收层) ********************/

// 对应机载端 publishDroneStatus()：连接状态、电量、飞行模式、解锁状态、状态文字。
void GroundLinkBridge::handleDroneStatusReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    configureStream(stream);
    useSinglePrecision(stream);

    quint8 connected = 0;
    float battery_percent = 0.0f;
    quint8 flight_mode = 0;
    quint8 armed = 0;
    QByteArray state;

    stream >> connected;
    stream >> battery_percent;
    stream >> flight_mode;
    stream >> armed;
    if (!readSizedBytes(stream, state) || !streamFullyConsumed(stream)) {
        RCLCPP_ERROR(this->get_logger(), "invalid DroneStatus payload");
        return;
    }

    drone_msgs::msg::DroneStatus msg;
    msg.connected = (connected != 0);
    msg.battery_percent = battery_percent;
    msg.flight_mode = static_cast<uint8_t>(flight_mode);
    msg.armed = (armed != 0);
    msg.state = state.toStdString();

    status_pub_->publish(msg);
}

// 对应机载端 publishTaskStatus()：任务是否运行、动作名称、动作步骤、动作编号。
void GroundLinkBridge::handleTaskStatusReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    configureStream(stream);

    quint8 task_running = 0;
    QByteArray action_name;
    qint32 action_step = 0;
    quint8 action_num = 0;

    stream >> task_running;
    if (!readSizedBytes(stream, action_name)) {
        RCLCPP_ERROR(this->get_logger(), "invalid TaskStatus payload");
        return;
    }
    stream >> action_step;
    stream >> action_num;
    if (!streamFullyConsumed(stream)) {
        RCLCPP_ERROR(this->get_logger(), "invalid TaskStatus payload");
        return;
    }

    drone_msgs::msg::TaskStatus msg;
    msg.task_running = (task_running != 0);
    msg.action_name = action_name.toStdString();
    msg.action_step = action_step;
    msg.action_num = action_num;

    task_status_pub_->publish(msg);
}

// 对应机载端 publishPathReady()，该 payload 只有一个表示是否就绪的字节。
void GroundLinkBridge::handlePathReadyReport(const QByteArray &payload)
{
    if (payload.size() != 1) {
        RCLCPP_ERROR(this->get_logger(), "invalid PathReady payload");
        return;
    }

    drone_msgs::msg::ReadyStatus msg;
    msg.air_route_ready = (static_cast<uint8_t>(payload[0]) != 0);
    path_ready_pub_->publish(msg);
}

// 对应机载端 publishReturnWorldGroup()：先读点数量，再连续读取每个点的 x、y。
void GroundLinkBridge::handleReturnWorldGroupReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    configureStream(stream);
    useSinglePrecision(stream);

    quint16 point_count = 0;
    stream >> point_count;

    if (stream.status() != QDataStream::Ok || !stream.device() ||
        stream.device()->bytesAvailable() != point_count * 8LL) {
        RCLCPP_ERROR(this->get_logger(), "invalid ReturnWorldGroup payload");
        return;
    }

    drone_msgs::msg::WorldGroup msg;
    msg.points.reserve(point_count);

    for (quint16 i = 0; i < point_count; ++i) {
        float x = 0.0f;
        float y = 0.0f;
        stream >> x;
        stream >> y;

        drone_msgs::msg::WorldPoint point;
        point.x = x;
        point.y = y;
        msg.points.push_back(point);
    }

    if (!streamFullyConsumed(stream)) {
        RCLCPP_ERROR(this->get_logger(), "invalid ReturnWorldGroup payload");
        return;
    }

    return_world_group_pub_->publish(msg);
}

// 对应机载端 publishVisionBarcode()：时间戳、条码文字、图片格式文字。
void GroundLinkBridge::handleVisionBarcodeReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    configureStream(stream);

    qint32 sec = 0;
    quint32 nanosec = 0;
    QByteArray barcode;
    QByteArray image_format;

    stream >> sec;
    stream >> nanosec;
    if (!readSizedBytes(stream, barcode) ||
        !readSizedBytes(stream, image_format) ||
        !streamFullyConsumed(stream)) {
        RCLCPP_ERROR(this->get_logger(), "invalid VisionBarcode payload");
        return;
    }

    drone_msgs::msg::BarcodeCapture msg;
    msg.stamp.sec = sec;
    msg.stamp.nanosec = nanosec;
    msg.barcode = barcode.toStdString();
    msg.image_format = image_format.toStdString();

    vision_barcode_pub_->publish(msg);
}

// Rebuild the ROS status message sent by the airborne bridge. No image payload
// is part of this frame; it contains only controller state and target metadata.
void GroundLinkBridge::handleVisionServoStatusReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    configureStream(stream);
    useDoublePrecision(stream);

    qint32 sec = 0;
    quint32 nanosec = 0;
    quint8 active = 0;
    QByteArray state;
    QByteArray requested_target_id;
    QByteArray tracked_target_id;
    quint32 target_sequence = 0;
    quint8 target_visible = 0;
    quint8 aligned = 0;
    double filtered_error_x = 0.0;
    double filtered_error_y = 0.0;
    QByteArray detail;

    stream >> sec;
    stream >> nanosec;
    stream >> active;
    if (!readSizedBytes(stream, state) ||
        !readSizedBytes(stream, requested_target_id) ||
        !readSizedBytes(stream, tracked_target_id)) {
        RCLCPP_ERROR(this->get_logger(), "invalid VisionServoStatus payload");
        return;
    }
    stream >> target_sequence;
    stream >> target_visible;
    stream >> aligned;
    stream >> filtered_error_x;
    stream >> filtered_error_y;
    if (!readSizedBytes(stream, detail) || !streamFullyConsumed(stream)) {
        RCLCPP_ERROR(this->get_logger(), "invalid VisionServoStatus payload");
        return;
    }

    drone_msgs::msg::VisionServoStatus msg;
    msg.stamp.sec = sec;
    msg.stamp.nanosec = nanosec;
    msg.active = (active != 0);
    msg.state = state.toStdString();
    msg.requested_target_id = requested_target_id.toStdString();
    msg.tracked_target_id = tracked_target_id.toStdString();
    msg.target_sequence = target_sequence;
    msg.target_visible = (target_visible != 0);
    msg.aligned = (aligned != 0);
    msg.filtered_error_x = filtered_error_x;
    msg.filtered_error_y = filtered_error_y;
    msg.detail = detail.toStdString();
    vision_servo_status_pub_->publish(msg);
}

// 对应机载端 publishDelta()，依次读取三个 float32：x、y、z。
void GroundLinkBridge::handleDeltaReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    configureStream(stream);
    useSinglePrecision(stream);

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    stream >> x;
    stream >> y;
    stream >> z;

    if (!streamFullyConsumed(stream)) {
        RCLCPP_ERROR(this->get_logger(), "invalid Delta payload");
        return;
    }

    geometry_msgs::msg::Vector3 msg;
    msg.x = x;
    msg.y = y;
    msg.z = z;

    delta_pub_->publish(msg);
}

// 对应机载端 publishLocalPosition()，依次读取x、y、z、qx、qy、qz、qw，分别是位置和四元数。
void GroundLinkBridge::handleLocalPositionReport(const QByteArray &payload)
{
    QDataStream stream(payload);
    configureStream(stream);
    useDoublePrecision(stream);

    double x = 0.0f;
    double y = 0.0f;
    double z = 0.0f;

    double qx = 0.0f;
    double qy = 0.0f;
    double qz = 0.0f;
    double qw = 0.0f;

    stream >> x;
    stream >> y;
    stream >> z;
    stream >> qx;
    stream >> qy;
    stream >> qz;
    stream >> qw;

    if (!streamFullyConsumed(stream)) {
        RCLCPP_ERROR(this->get_logger(), "invalid LocalPosition payload");
        return;
    }

    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
    msg.pose.orientation.x = qx;
    msg.pose.orientation.y = qy;
    msg.pose.orientation.z = qz;
    msg.pose.orientation.w = qw;

    local_position_pub_->publish(msg);
}

// 上传任务响应格式：是否成功、提示文字、保存路径、动作数量。
// 解析成功后，再根据 seq 找到正在等待这次结果的地面 ROS 服务调用。
void GroundLinkBridge::handleUploadMissionSummaryResponse(uint16_t seq, const QByteArray &payload)
{
    QDataStream stream(payload);
    configureStream(stream);

    quint8 success = 0;
    QByteArray message_bytes;
    QByteArray saved_path_bytes;
    quint32 action_count = 0;

    stream >> success;
    if (!readSizedBytes(stream, message_bytes) ||
        !readSizedBytes(stream, saved_path_bytes)) {
        RCLCPP_ERROR(this->get_logger(), "invalid UploadMissionSummaryResp payload");
        return;
    }
    stream >> action_count;
    if (!streamFullyConsumed(stream)) {
        RCLCPP_ERROR(this->get_logger(), "invalid UploadMissionSummaryResp payload");
        return;
    }

    const QString message = QString::fromUtf8(message_bytes);
    const QString saved_path = QString::fromUtf8(saved_path_bytes);

    pending_requests_.erase(seq);

    auto it = pending_upload_calls_.find(seq);
    if (it != pending_upload_calls_.end()) {
        it->second.success = (success != 0);
        it->second.message = message.toStdString();
        it->second.saved_path = saved_path.toStdString();
        it->second.action_count = action_count;
        it->second.done = true;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "UploadMissionSummaryResp: success=%d, message=%s, saved_path=%s, action_count=%u",
        static_cast<int>(success),
        message.toStdString().c_str(),
        saved_path.toStdString().c_str(),
        static_cast<unsigned int>(action_count));
}

void GroundLinkBridge::handleStartOffboardResponse(uint16_t seq, const QByteArray &payload)
{
    quint8 success = 0;
    QByteArray message_bytes;
    if (!decodeSimpleResponse(payload, success, message_bytes)) {
        RCLCPP_ERROR(this->get_logger(), "invalid StartOffboardResp payload");
        return;
    }

    const QString message = QString::fromUtf8(message_bytes);

    pending_requests_.erase(seq);

    auto it = pending_start_offboard_calls_.find(seq);
    if (it != pending_start_offboard_calls_.end()) {
        it->second.success = (success != 0);
        it->second.message = message.toStdString();
        it->second.done = true;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "StartOffboardResp: success=%d, message=%s",
        static_cast<int>(success),
        message.toStdString().c_str());
}

void GroundLinkBridge::handleStartTaskResponse(uint16_t seq, const QByteArray &payload)
{
    quint8 success = 0;
    QByteArray message_bytes;
    if (!decodeSimpleResponse(payload, success, message_bytes)) {
        RCLCPP_ERROR(this->get_logger(), "invalid StartTaskResp payload");
        return;
    }

    const QString message = QString::fromUtf8(message_bytes);

    pending_requests_.erase(seq);

    auto it = pending_start_task_calls_.find(seq);
    if (it != pending_start_task_calls_.end()) {
        it->second.success = (success != 0);
        it->second.message = message.toStdString();
        it->second.done = true;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "StartTaskResp: success=%d, message=%s",
        static_cast<int>(success),
        message.toStdString().c_str());
}

void GroundLinkBridge::handleStopPushResponse(uint16_t seq, const QByteArray &payload)
{
    quint8 success = 0;
    QByteArray message_bytes;
    if (!decodeSimpleResponse(payload, success, message_bytes)) {
        RCLCPP_ERROR(this->get_logger(), "invalid StopPushResp payload");
        return;
    }

    const QString message = QString::fromUtf8(message_bytes);

    pending_requests_.erase(seq);

    auto it = pending_stop_push_calls_.find(seq);
    if (it != pending_stop_push_calls_.end()) {
        it->second.success = (success != 0);
        it->second.message = message.toStdString();
        it->second.done = true;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "StopPushResp: success=%d, message=%s",
        static_cast<int>(success),
        message.toStdString().c_str());
}








/************************ 底层 *************************/

bool GroundLinkBridge::tryParseOnePacket(Packet &packet)
{
    while(true){
        //尝试从接收缓冲区中解析出完整的协议帧，直到无法再解析出新的帧为止
        while (rx_buffer_.size() >= 2) {
            const uint8_t b0 = static_cast<uint8_t>(rx_buffer_[0]);
            const uint8_t b1 = static_cast<uint8_t>(rx_buffer_[1]);
            if (b0 == lp::kSof1 && b1 == lp::kSof2) {
                break;
            }
            rx_buffer_.remove(0, 1);
        }

        if (rx_buffer_.size() < 11) {
            return false;
        }

        //从帧数据中提取载荷长度字段，计算出预期的帧长度，并与实际接收的帧长度进行比较，如果不匹配则认为帧不合法，继续尝试解析下一个帧
        const auto *data = reinterpret_cast<const uint8_t *>(rx_buffer_.constData());
        const uint16_t payload_len =
            static_cast<uint16_t>(data[7]) |
            (static_cast<uint16_t>(data[8]) << 8);

        if (payload_len > kMaxPayloadSize) {
            // 只删除一个字节再搜索帧头，避免损坏帧把后面的正常帧整段吞掉。
            rx_buffer_.remove(0, 1);
            continue;
        }

        //协议帧格式：2字节帧头 + 1字节版本 + 1字节类型 + 1字节标志 + 2字节序列号 + 2字节载荷长度 + N字节载荷 + 2字节CRC16校验
        const int frame_len = 9 + static_cast<int>(payload_len) + 2;
        if (rx_buffer_.size() < frame_len) {
            return false;
        }

        const QByteArray frame = rx_buffer_.left(frame_len);

        if (!validateFrame(frame)) {
            // CRC 错误时逐字节重新同步，不相信损坏帧中的长度字段。
            rx_buffer_.remove(0, 1);
            continue;
        }

        rx_buffer_.remove(0, frame_len);

        //从帧数据中提取消息类型、标志位、序列号和载荷数据，填充到Packet结构体中，供后续处理函数使用
        const auto *frame_data = reinterpret_cast<const uint8_t *>(frame.constData());
        packet.type = frame_data[3];
        packet.flags = frame_data[4];
        packet.seq = static_cast<uint16_t>(frame_data[5]) |
                    (static_cast<uint16_t>(frame_data[6]) << 8);
        packet.payload = frame.mid(9, payload_len);
        return true;
    }
}

QByteArray GroundLinkBridge::encodeFrame(
    uint8_t type,
    uint8_t flags,
    uint16_t seq,
    const QByteArray &payload) const
{
    //编码函数，用于将消息类型、标志位、序列号和载荷数据编码成一个完整的帧数据，准备发送
    if (payload.size() > std::numeric_limits<quint16>::max()) {
        return {};
    }

    QByteArray frame;
    frame.reserve(9 + payload.size() + 2);

    //帧格式：2字节帧头 + 1字节版本 + 1字节类型 + 1字节标志 + 2字节序列号 + 2字节载荷长度 + N字节载荷 + 2字节CRC16校验
    frame.append(static_cast<char>(lp::kSof1));
    frame.append(static_cast<char>(lp::kSof2));
    frame.append(static_cast<char>(lp::kVersion));
    frame.append(static_cast<char>(type));
    frame.append(static_cast<char>(flags));

    frame.append(static_cast<char>(seq & 0xFF));
    frame.append(static_cast<char>((seq >> 8) & 0xFF));

    const uint16_t payload_len = static_cast<uint16_t>(payload.size());
    frame.append(static_cast<char>(payload_len & 0xFF));
    frame.append(static_cast<char>((payload_len >> 8) & 0xFF));

    frame.append(payload);

    const uint8_t *crc_begin = reinterpret_cast<const uint8_t *>(frame.constData() + 2);
    const int crc_len = frame.size() - 2;
    const uint16_t crc = crc16_ccitt(crc_begin, crc_len);

    frame.append(static_cast<char>(crc & 0xFF));
    frame.append(static_cast<char>((crc >> 8) & 0xFF));

    return frame;
}

bool GroundLinkBridge::validateFrame(const QByteArray &frame) const
{
    //验证函数，用于检查接收到的帧数据是否符合协议格式和校验要求，确保数据的正确性和完整性
    if (frame.size() < 11) {
        return false;
    }

    const auto *data = reinterpret_cast<const uint8_t *>(frame.constData());

    //检查帧头、版本号、载荷长度和CRC校验等字段，确保帧的合法性和完整性
    if (data[0] != lp::kSof1 || data[1] != lp::kSof2) {
        return false;
    }

    if (data[2] != lp::kVersion) {
        return false;
    }

    //从帧数据中提取载荷长度字段，计算出预期的帧长度，并与实际接收的帧长度进行比较，如果不匹配则认为帧不合法
    const uint16_t payload_len =
        static_cast<uint16_t>(data[7]) |
        (static_cast<uint16_t>(data[8]) << 8);

    const int expected_size = 9 + static_cast<int>(payload_len) + 2;
    if (frame.size() != expected_size) {
        return false;
    }

    //从帧数据中提取CRC16校验字段，计算出接收到的帧数据的CRC16校验值，并与提取的CRC16值进行比较，如果不匹配则认为帧数据有误，需要丢弃
    const uint16_t recv_crc =
        static_cast<uint16_t>(data[frame.size() - 2]) |
        (static_cast<uint16_t>(data[frame.size() - 1]) << 8);

    //计算CRC16校验值时，需要从帧数据的第3个字节开始（即版本号字段），一直计算到载荷数据的最后一个字节，才能得到正确的CRC16值
    const uint16_t calc_crc = crc16_ccitt(data + 2, frame.size() - 4);

    //如果帧数据的CRC16校验通过，则认为帧数据合法，可以继续进行后续处理；否则认为帧数据有误，需要丢弃
    return recv_crc == calc_crc;
}

void GroundLinkBridge::onRetryTimer()
{
    const auto now = this->now();

    //检查待重试的请求是否超时，并进行重试
    for (auto it = pending_requests_.begin(); it != pending_requests_.end(); ) {
        auto &pending = it->second;
        if (now < pending.deadline) {
            ++it;
            continue;
        }

        //如果重试次数超过3次，则认为请求失败，删除请求信息，并记录错误日志
        if (pending.retry_count >= 3) {
            RCLCPP_ERROR(this->get_logger(), "request seq=%u timeout", pending.seq);
            it = pending_requests_.erase(it);
            continue;
        }

        //重试发送协议帧到串口，并更新重试次数和截止时间，继续检查下一个请求
        serial_.write(pending.encoded_frame);
        pending.retry_count += 1;
        pending.deadline = now + rclcpp::Duration::from_seconds(0.3);
        ++it;
    }
}
