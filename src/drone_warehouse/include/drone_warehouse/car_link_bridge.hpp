#pragma once

#include <cstdint>
#include <string>

#include <QByteArray>
#include <QSerialPort>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class CarLinkBridge : public rclcpp::Node
{
public:
    CarLinkBridge();
    ~CarLinkBridge() override = default;

private:
    void setupRosInterfaces();
    void setupSerial();
    void onSerialReadyRead();

    bool tryParseCarFrame(uint8_t &type, QByteArray &payload);
    bool validateFrame(const QByteArray &frame) const;
    void publishCarLocalPosition(const QByteArray &payload);
    void publishCarKeypadS4Pressed(const QByteArray &payload);
    void publishCarRouteState(const QByteArray &payload);
    void sendCarControlMode(
        const std_msgs::msg::String::SharedPtr message);

    QByteArray encodeFrame(
        uint8_t type,
        uint8_t flags,
        uint16_t sequence,
        const QByteArray &payload) const;

    std::string serial_port_;
    int baud_rate_{115200};

    QSerialPort serial_;
    QByteArray rx_buffer_;
    uint16_t tx_sequence_{0};

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        car_local_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        car_keypad_s4_pressed_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
        car_route_state_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
        car_control_mode_sub_;
};