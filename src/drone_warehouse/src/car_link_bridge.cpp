#include "drone_warehouse/car_link_bridge.hpp"

#include <limits>
#include <QDataStream>

#include "drone_warehouse/link_protocol.hpp"

namespace lp = drone_msgs::link_protocol;

namespace
{
constexpr int kMaxPayloadSize = 4096;

void configureStream(QDataStream &stream)
{
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
}

uint16_t crc16Ccitt(const uint8_t *data, int length)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000)
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}
}

CarLinkBridge::CarLinkBridge()
    : rclcpp::Node("ground_car_link_bridge")
{
    serial_port_ = this->declare_parameter<std::string>(
        "serial_port", "/dev/warehouse_car_serial");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 115200);

    setupRosInterfaces();
    setupSerial();
}

void CarLinkBridge::setupRosInterfaces()
{
    car_local_position_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/serial/car/local_position",
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    car_keypad_s4_pressed_pub_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/serial/car/keypad/s4_pressed",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    car_route_state_pub_ =
        this->create_publisher<std_msgs::msg::String>(
            "/serial/car/route_state",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    car_control_mode_sub_ =
        this->create_subscription<std_msgs::msg::String>(
            "/serial/car/control_mode",
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
            [this](const std_msgs::msg::String::SharedPtr message) {
                sendCarControlMode(message);
            });
}

void CarLinkBridge::setupSerial()
{
    serial_.setPortName(QString::fromStdString(serial_port_));
    serial_.setBaudRate(baud_rate_);
    serial_.setDataBits(QSerialPort::Data8);
    serial_.setParity(QSerialPort::NoParity);
    serial_.setStopBits(QSerialPort::OneStop);
    serial_.setFlowControl(QSerialPort::NoFlowControl);

    QObject::connect(&serial_, &QSerialPort::readyRead, [this]() {
        onSerialReadyRead();
    });

    if (!serial_.open(QIODevice::ReadWrite)) {
        RCLCPP_ERROR(
            this->get_logger(),
            "failed to open car serial port %s: %s",
            serial_port_.c_str(),
            serial_.errorString().toStdString().c_str());
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "car serial port opened: %s, baud=%d",
        serial_port_.c_str(),
        baud_rate_);
}

void CarLinkBridge::onSerialReadyRead()
{
    rx_buffer_.append(serial_.readAll());

    uint8_t type = 0;
    QByteArray payload;
    while (tryParseCarFrame(type, payload)) {
        if (type == lp::kTypecarLocalPosition) {
            publishCarLocalPosition(payload);
        } else if (type == lp::kTypeCarKeypadS4Pressed) {
            publishCarKeypadS4Pressed(payload);
        } else if (type == lp::kTypeCarRouteState) {
            publishCarRouteState(payload);
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "ignored unexpected car serial frame type: 0x%02X",
                type);
        }
    }
}

bool CarLinkBridge::tryParseCarFrame(uint8_t &type, QByteArray &payload)
{
    while (true) {
        while (rx_buffer_.size() >= 2) {
            const auto first = static_cast<uint8_t>(rx_buffer_[0]);
            const auto second = static_cast<uint8_t>(rx_buffer_[1]);
            if (first == lp::kSof1 && second == lp::kSof2) {
                break;
            }
            rx_buffer_.remove(0, 1);
        }

        if (rx_buffer_.size() < 11) {
            return false;
        }

        const auto *data =
            reinterpret_cast<const uint8_t *>(rx_buffer_.constData());
        const uint16_t payload_length =
            static_cast<uint16_t>(data[7]) |
            (static_cast<uint16_t>(data[8]) << 8);

        if (payload_length > kMaxPayloadSize) {
            rx_buffer_.remove(0, 1);
            continue;
        }

        const int frame_length = 9 + static_cast<int>(payload_length) + 2;
        if (rx_buffer_.size() < frame_length) {
            return false;
        }

        const QByteArray frame = rx_buffer_.left(frame_length);
        rx_buffer_.remove(0, frame_length);

        if (!validateFrame(frame)) {
            RCLCPP_WARN(this->get_logger(), "discarded invalid car serial frame");
            continue;
        }

        const auto *frame_data =
            reinterpret_cast<const uint8_t *>(frame.constData());
        type = frame_data[3];
        payload = frame.mid(9, payload_length);
        return true;
    }
}

bool CarLinkBridge::validateFrame(const QByteArray &frame) const
{
    if (frame.size() < 11) {
        return false;
    }

    const auto *data =
        reinterpret_cast<const uint8_t *>(frame.constData());
    if (data[0] != lp::kSof1 ||
        data[1] != lp::kSof2 ||
        data[2] != lp::kVersion) {
        return false;
    }

    const uint16_t payload_length =
        static_cast<uint16_t>(data[7]) |
        (static_cast<uint16_t>(data[8]) << 8);
    if (frame.size() != 9 + static_cast<int>(payload_length) + 2) {
        return false;
    }

    const uint16_t received_crc =
        static_cast<uint16_t>(data[frame.size() - 2]) |
        (static_cast<uint16_t>(data[frame.size() - 1]) << 8);
    const uint16_t calculated_crc =
        crc16Ccitt(data + 2, frame.size() - 4);
    return received_crc == calculated_crc;
}

void CarLinkBridge::publishCarLocalPosition(const QByteArray &payload)
{
    QDataStream stream(payload);
    configureStream(stream);

    geometry_msgs::msg::PoseStamped message;
    stream >> message.pose.position.x;
    stream >> message.pose.position.y;
    stream >> message.pose.position.z;
    stream >> message.pose.orientation.x;
    stream >> message.pose.orientation.y;
    stream >> message.pose.orientation.z;
    stream >> message.pose.orientation.w;

    if (stream.status() != QDataStream::Ok ||
        !stream.device() ||
        stream.device()->bytesAvailable() != 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "invalid car local position payload");
        return;
    }

    message.header.stamp = this->now();
    car_local_position_pub_->publish(message);
}

void CarLinkBridge::publishCarKeypadS4Pressed(const QByteArray &payload)
{
    // S4 状态帧固定为一个字节：按下时 true，常态为 false。
    if (payload.size() != 1) {
        RCLCPP_ERROR(
            this->get_logger(),
            "invalid car route start payload");
        return;
    }

    std_msgs::msg::Bool message;
    message.data = static_cast<uint8_t>(payload.at(0)) != 0;
    car_keypad_s4_pressed_pub_->publish(message);
}

void CarLinkBridge::publishCarRouteState(const QByteArray &payload)
{
    // 路线状态是短 ASCII 字符串。这里保留未来扩展状态，只拒绝空内容。
    const QString state =
        QString::fromUtf8(payload).trimmed().toUpper();
    if (state.isEmpty())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "ignored empty car route state payload");
        return;
    }

    std_msgs::msg::String message;
    message.data = state.toStdString();
    car_route_state_pub_->publish(message);
}

void CarLinkBridge::sendCarControlMode(
    const std_msgs::msg::String::SharedPtr message)
{
    if (!message || !serial_.isOpen()) {
        return;
    }

    const QString mode =
        QString::fromStdString(message->data).trimmed().toUpper();
    if (mode != "DISABLED" && mode != "AUTO") {
        RCLCPP_WARN(
            this->get_logger(),
            "ignored unsupported car control mode: %s",
            message->data.c_str());
        return;
    }

    const QByteArray payload = mode.toUtf8();
    const QByteArray frame = encodeFrame(
        lp::kTypeCarControlMode,
        0,
        tx_sequence_++,
        payload);
    if (frame.isEmpty() || serial_.write(frame) < 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "failed to write car control mode frame: %s",
            serial_.errorString().toStdString().c_str());
    }
}

QByteArray CarLinkBridge::encodeFrame(
    uint8_t type,
    uint8_t flags,
    uint16_t sequence,
    const QByteArray &payload) const
{
    if (payload.size() > std::numeric_limits<quint16>::max()) {
        return {};
    }

    QByteArray frame;
    frame.reserve(9 + payload.size() + 2);
    frame.append(static_cast<char>(lp::kSof1));
    frame.append(static_cast<char>(lp::kSof2));
    frame.append(static_cast<char>(lp::kVersion));
    frame.append(static_cast<char>(type));
    frame.append(static_cast<char>(flags));
    frame.append(static_cast<char>(sequence & 0xFF));
    frame.append(static_cast<char>((sequence >> 8) & 0xFF));

    const auto payload_length =
        static_cast<uint16_t>(payload.size());
    frame.append(static_cast<char>(payload_length & 0xFF));
    frame.append(static_cast<char>((payload_length >> 8) & 0xFF));
    frame.append(payload);

    const auto *crc_begin =
        reinterpret_cast<const uint8_t *>(frame.constData() + 2);
    const uint16_t crc =
        crc16Ccitt(crc_begin, frame.size() - 2);
    frame.append(static_cast<char>(crc & 0xFF));
    frame.append(static_cast<char>((crc >> 8) & 0xFF));
    return frame;
}
