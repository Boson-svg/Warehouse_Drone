#include <chrono>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "drone_msgs/msg/barcode_capture.hpp"

using namespace std::chrono_literals;

static std::vector<uint8_t> readFileBytes(const std::string &file_path)
{
    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("failed to open image file: " + file_path);
    }

    return std::vector<uint8_t>(
        std::istreambuf_iterator<char>(file),
        std::istreambuf_iterator<char>());
}

static std::string getImageFormatFromPath(const std::string &file_path)
{
    const std::size_t dot_pos = file_path.find_last_of('.');
    if (dot_pos == std::string::npos || dot_pos + 1 >= file_path.size()) {
        return "jpg";
    }

    std::string ext = file_path.substr(dot_pos + 1);
    for (char &c : ext) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }

    if (ext == "jpeg") {
        return "jpg";
    }

    return ext;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 3) {
        std::cerr
            << "Usage: barcode_test_sender <IMAGE_PATH> <BARCODE_TEXT>\n"
            << "Example: barcode_test_sender YOUR_IMAGE_PATH_HERE CODE_001\n";
        rclcpp::shutdown();
        return 1;
    }

    const std::string image_path = argv[1];
    const std::string barcode_text = argv[2];

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto node = std::make_shared<rclcpp::Node>("barcode_test_sender");
    auto pub = node->create_publisher<drone_msgs::msg::BarcodeCapture>(
        "/drone/image", pub_qos);

    try {
        const std::vector<uint8_t> image_bytes = readFileBytes(image_path);
        const std::string image_format = getImageFormatFromPath(image_path);

        drone_msgs::msg::BarcodeCapture msg;
        msg.stamp = node->get_clock()->now();
        msg.barcode = barcode_text;
        msg.image_format = image_format;
        msg.image_data = image_bytes;

        RCLCPP_INFO(
            node->get_logger(),
            "image loaded: path=%s, bytes=%zu, format=%s, barcode=%s",
            image_path.c_str(),
            image_bytes.size(),
            image_format.c_str(),
            barcode_text.c_str());

        // 给订阅端一点时间建立连接
        rclcpp::sleep_for(500ms);

        pub->publish(msg);

        RCLCPP_INFO(node->get_logger(), "barcode capture message published");

        // 稍等一下，避免刚发完进程就退出
        rclcpp::sleep_for(1s);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}