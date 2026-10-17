#include <algorithm>
#include <memory>
#include <string>
#include <cerrno>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>

#include <array>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "drone_msgs/msg/barcode_capture.hpp"
#include "drone_msgs/msg/k230_animal_center.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class K230AnimalsUartRos2Node : public rclcpp::Node
{
public:
    K230AnimalsUartRos2Node() : Node("k230_animals_uart_ros2_node")
    {
        serial_port_ = this->declare_parameter<std::string>("serial_port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
        baudrate_ = this->declare_parameter<int>("baudrate", 460800);
        image_topic_ = this->declare_parameter<std::string>("image_topic", "/drone/image");
        detect_topic_ = this->declare_parameter<std::string>("detect_topic", "/k230/animals/detect");
        center_topic_ = this->declare_parameter<std::string>("center_topic", "/k230/animals/center");

        RCLCPP_INFO(this->get_logger(), "K230 animals UART ROS2 node started");
        RCLCPP_INFO(this->get_logger(), "serial_port=%s baudrate=%d", serial_port_.c_str(), baudrate_);
        RCLCPP_INFO(this->get_logger(), "image_topic=%s", image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "detect_topic=%s", detect_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "center_topic=%s", center_topic_.c_str());

        image_pub_ = this->create_publisher<drone_msgs::msg::BarcodeCapture>(
            image_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
        detect_pub_ = this->create_publisher<std_msgs::msg::String>(
            detect_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
        center_pub_ = this->create_publisher<drone_msgs::msg::K230AnimalCenter>(
            center_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

        open_serial();

        read_timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&K230AnimalsUartRos2Node::read_serial_once, this));
    }

    ~K230AnimalsUartRos2Node() override
    {
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }

private:
    static constexpr size_t kHeaderSize = 13;
    static constexpr size_t kCrcSize = 2;
    static constexpr size_t kMaxPayloadSize = 2 * 1024 * 1024;

    static speed_t baudrate_to_speed(int baudrate)
    {
        switch (baudrate)
        {
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        default:
            throw std::runtime_error("Unsupported baudrate");
        }
    }

    static uint32_t read_u32_be(const uint8_t *p)
    {
        return (static_cast<uint32_t>(p[0]) << 24) |
               (static_cast<uint32_t>(p[1]) << 16) |
               (static_cast<uint32_t>(p[2]) << 8) |
               static_cast<uint32_t>(p[3]);
    }

    static uint16_t read_u16_be(const uint8_t *p)
    {
        return (static_cast<uint16_t>(p[0]) << 8) |
               static_cast<uint16_t>(p[1]);
    }

    static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
    {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i)
        {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (int bit = 0; bit < 8; ++bit)
            {
                if (crc & 0x8000)
                {
                    crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
                }
                else
                {
                    crc = static_cast<uint16_t>(crc << 1);
                }
            }
        }
        return crc;
    }

    static bool is_likely_jpeg(const std::vector<uint8_t> &payload)
    {
        if (payload.size() < 4)
        {
            return false;
        }
        return payload[0] == 0xFF &&
               payload[1] == 0xD8 &&
               payload[payload.size() - 2] == 0xFF &&
               payload[payload.size() - 1] == 0xD9;
    }

    static bool json_bool_field_true(const std::string &json_text, const std::string &field)
    {
        const std::string key = "\"" + field + "\"";
        const size_t key_pos = json_text.find(key);
        if (key_pos == std::string::npos)
        {
            return false;
        }

        const size_t colon_pos = json_text.find(':', key_pos + key.size());
        if (colon_pos == std::string::npos)
        {
            return false;
        }

        size_t value_pos = colon_pos + 1;
        while (value_pos < json_text.size() && json_text[value_pos] == ' ')
        {
            ++value_pos;
        }

        return json_text.compare(value_pos, 4, "true") == 0 ||
               json_text.compare(value_pos, 1, "1") == 0;
    }

    static std::string json_string_field(const std::string &json_text, const std::string &field)
    {
        const std::string key = "\"" + field + "\"";
        const size_t key_pos = json_text.find(key);
        if (key_pos == std::string::npos)
        {
            return "";
        }

        const size_t colon_pos = json_text.find(':', key_pos + key.size());
        if (colon_pos == std::string::npos)
        {
            return "";
        }

        size_t quote_pos = json_text.find('"', colon_pos + 1);
        if (quote_pos == std::string::npos)
        {
            return "";
        }

        const size_t end_quote_pos = json_text.find('"', quote_pos + 1);
        if (end_quote_pos == std::string::npos || end_quote_pos <= quote_pos + 1)
        {
            return "";
        }

        return json_text.substr(quote_pos + 1, end_quote_pos - quote_pos - 1);
    }

    static bool json_number_field(const std::string &json_text, const std::string &field, double &value)
    {
        const std::string key = "\"" + field + "\"";
        const size_t key_pos = json_text.find(key);
        if (key_pos == std::string::npos)
        {
            return false;
        }

        const size_t colon_pos = json_text.find(':', key_pos + key.size());
        if (colon_pos == std::string::npos)
        {
            return false;
        }

        size_t value_pos = colon_pos + 1;
        while (value_pos < json_text.size() && json_text[value_pos] == ' ')
        {
            ++value_pos;
        }

        char *end_ptr = nullptr;
        const double parsed = std::strtod(json_text.c_str() + value_pos, &end_ptr);
        if (end_ptr == json_text.c_str() + value_pos)
        {
            return false;
        }

        value = parsed;
        return true;
    }

    void open_serial()
    {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0)
        {
            throw std::runtime_error("open serial failed: " + serial_port_ + " error=" + std::strerror(errno));
        }

        termios tty{};
        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
        }

        cfmakeraw(&tty);

        const speed_t speed = baudrate_to_speed(baudrate_);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
        {
            throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
        }

        tcflush(serial_fd_, TCIOFLUSH);

        RCLCPP_INFO(this->get_logger(), "serial opened successfully fd=%d", serial_fd_);
    }

    std::string make_image_barcode(uint32_t) const
    {
        return last_label_.empty() ? "动物" : last_label_;
    }

    void publish_image_packet(uint32_t seq, const std::vector<uint8_t> &payload)
    {
        if (!is_likely_jpeg(payload))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "skip non-jpeg image packet seq=%u bytes=%zu",
                seq,
                payload.size());
            return;
        }

        drone_msgs::msg::BarcodeCapture msg;
        msg.stamp = this->now();
        msg.barcode = make_image_barcode(seq);
        msg.image_format = "jpeg";
        msg.image_data = payload;

        image_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "published image barcode=%s bytes=%zu",
            msg.barcode.c_str(),
            msg.image_data.size());
    }

    void publish_detection_packet(uint32_t seq, const std::string &json_text)
    {
        std_msgs::msg::String detect_msg;
        detect_msg.data = json_text;
        detect_pub_->publish(detect_msg);

        if (!json_bool_field_true(json_text, "valid"))
        {
            return;
        }

        double norm_x = 0.0;
        double norm_y = 0.0;
        double score = 0.0;
        double cx = 0.0;
        double cy = 0.0;
        double err_x = 0.0;
        double err_y = 0.0;
        double stable_frames = 0.0;
        double count = 0.0;
        if (!json_number_field(json_text, "norm_x", norm_x) ||
            !json_number_field(json_text, "norm_y", norm_y) ||
            !json_number_field(json_text, "cx", cx) ||
            !json_number_field(json_text, "cy", cy) ||
            !json_number_field(json_text, "err_x", err_x) ||
            !json_number_field(json_text, "err_y", err_y))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "valid detection missing center fields: %s",
                json_text.c_str());
            return;
        }
        json_number_field(json_text, "score", score);
        json_number_field(json_text, "stable_frames", stable_frames);
        json_number_field(json_text, "count", count);

        drone_msgs::msg::K230AnimalCenter center_msg;
        center_msg.stamp = this->now();
        center_msg.seq = seq;
        center_msg.valid = true;
        center_msg.confirmed = json_bool_field_true(json_text, "confirmed");
        center_msg.stable_frames = static_cast<int32_t>(stable_frames);
        center_msg.count = static_cast<uint32_t>(count);
        center_msg.label = json_string_field(json_text, "label");
        center_msg.score = score;
        center_msg.cx = static_cast<int32_t>(cx);
        center_msg.cy = static_cast<int32_t>(cy);
        center_msg.err_x = static_cast<int32_t>(err_x);
        center_msg.err_y = static_cast<int32_t>(err_y);
        center_msg.norm_x = norm_x;
        center_msg.norm_y = norm_y;
        center_pub_->publish(center_msg);
    }

    void handle_packet(uint8_t type, uint32_t seq, const std::vector<uint8_t> &payload)
    {
        if (type == 1)
        {
            const std::string json_text(payload.begin(), payload.end());
            const std::string label = json_string_field(json_text, "label");
            if (json_bool_field_true(json_text, "valid") && !label.empty())
            {
                last_label_ = label;
            }

            publish_detection_packet(seq, json_text);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "DETECTION packet seq=%u bytes=%zu json=%s",
                seq,
                payload.size(),
                json_text.c_str());
            return;
        }

        if (type == 2)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "JPEG packet seq=%u bytes=%zu",
                seq,
                payload.size());
            publish_image_packet(seq, payload);
            return;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "unknown packet type=%u seq=%u bytes=%zu",
            type,
            seq,
            payload.size());
    }

    void parse_packets()
    {
        const std::array<uint8_t, 4> magic{'K', '2', '3', '0'};

        while (true)
        {
            if (rx_buffer_.size() < kHeaderSize + kCrcSize)
            {
                return;
            }

            auto magic_it = std::search(
                rx_buffer_.begin(),
                rx_buffer_.end(),
                magic.begin(),
                magic.end());

            if (magic_it == rx_buffer_.end())
            {
                if (rx_buffer_.size() > magic.size() - 1)
                {
                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.end() - (magic.size() - 1));
                }
                return;
            }

            if (magic_it != rx_buffer_.begin())
            {
                rx_buffer_.erase(rx_buffer_.begin(), magic_it);
            }

            if (rx_buffer_.size() < kHeaderSize + kCrcSize)
            {
                return;
            }

            const uint8_t type = rx_buffer_[4];
            const uint32_t seq = read_u32_be(&rx_buffer_[5]);
            const uint32_t payload_len = read_u32_be(&rx_buffer_[9]);

            if (payload_len > kMaxPayloadSize)
            {
                RCLCPP_WARN(this->get_logger(), "payload too large=%u, dropping one byte", payload_len);
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            const size_t packet_len = kHeaderSize + static_cast<size_t>(payload_len) + kCrcSize;
            if (rx_buffer_.size() < packet_len)
            {
                return;
            }

            const uint16_t rx_crc = read_u16_be(&rx_buffer_[kHeaderSize + payload_len]);
            const uint16_t calc_crc = crc16_ccitt(rx_buffer_.data(), kHeaderSize + payload_len);

            if (rx_crc != calc_crc)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "crc mismatch type=%u seq=%u len=%u rx=0x%04x calc=0x%04x",
                    type,
                    seq,
                    payload_len,
                    rx_crc,
                    calc_crc);
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }

            std::vector<uint8_t> payload(
                rx_buffer_.begin() + kHeaderSize,
                rx_buffer_.begin() + kHeaderSize + payload_len);

            handle_packet(type, seq, payload);
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + packet_len);
        }
    }

    void read_serial_once()
    {
        if (serial_fd_ < 0)
        {
            return;
        }

        std::array<uint8_t, 4096> temp{};
        size_t tick_bytes = 0;

        while (true)
        {
            const ssize_t n = read(serial_fd_, temp.data(), temp.size());

            if (n > 0)
            {
                rx_buffer_.insert(rx_buffer_.end(), temp.begin(), temp.begin() + n);
                tick_bytes += static_cast<size_t>(n);
                total_rx_bytes_ += static_cast<size_t>(n);
                continue;
            }

            if (n == 0)
            {
                break;
            }

            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                break;
            }

            RCLCPP_ERROR(
                this->get_logger(),
                "serial read failed: %s",
                std::strerror(errno));
            break;
        }

        if (tick_bytes > 0)
        {
            parse_packets();

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "uart rx tick_bytes=%zu buffer=%zu total=%zu",
                tick_bytes,
                rx_buffer_.size(),
                total_rx_bytes_);
        }

        if (rx_buffer_.size() > 4 * 1024 * 1024)
        {
            RCLCPP_WARN(this->get_logger(), "rx_buffer too large, clearing");
            rx_buffer_.clear();
        }
    }

    std::string serial_port_;
    std::string image_topic_;
    std::string detect_topic_;
    std::string center_topic_;
    std::string last_label_;
    int baudrate_;
    int serial_fd_{-1};
    rclcpp::Publisher<drone_msgs::msg::BarcodeCapture>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detect_pub_;
    rclcpp::Publisher<drone_msgs::msg::K230AnimalCenter>::SharedPtr center_pub_;
    rclcpp::TimerBase::SharedPtr read_timer_;
    std::vector<uint8_t> rx_buffer_;
    size_t total_rx_bytes_{0};
};

int main(int argc, const char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<K230AnimalsUartRos2Node>());
    rclcpp::shutdown();
    return 0;
}
