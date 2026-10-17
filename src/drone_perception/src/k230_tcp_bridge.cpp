//
// K230 TCP Bridge — 将 K230 芯片通过 TCP 发来的 JPEG 快照帧桥接到 ROS 2 话题
//
// 网络协议（大端字节序）：
//
// ┌─────────────────┬────────────────────┬────────────────────────┬──────────────────┐
// │  Frame Header   │  Frame Name        │  Payload Header        │  JPEG Payload    │
// │  6 bytes        │  变长 (1~256)      │  8 bytes               │  变长 (1~20MB)   │
// ├─────────────────┼────────────────────┼────────────────────────┼──────────────────┤
// │ "KIMG"  magic 4B│ frame_name 字符串  │ payload_len  u32 BE    │ JPEG 原始数据     │
// │ name_len  u16 BE│ (UTF-8)            │ seq          u32 BE    │                  │
// └─────────────────┴────────────────────┴────────────────────────┴──────────────────┘
//
// 用法：ros2 run k230_snapshot_bridge k230_tcp_bridge
//          --ros-args -p listen_host:=0.0.0.0 -p listen_port:=9100
//

#include <arpa/inet.h>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "drone_msgs/msg/barcode_capture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
// TCP 帧协议常量
constexpr std::array<std::uint8_t, 4> kMagic{{'K', 'I', 'M', 'G'}};  // 帧起始魔数
constexpr std::size_t kFrameHeaderSize = 6U;       // magic(4) + name_len(2)
constexpr std::size_t kPayloadHeaderSize = 8U;     // payload_len(4) + seq(4)
constexpr std::size_t kMaxFrameNameLength = 256U;  // 帧名最大字节数
constexpr std::size_t kMaxPayloadLength = 20U * 1024U * 1024U;  // JPEG 最大 20MB

// 从字节缓冲区读取大端 16 位无符号整数
std::uint16_t readUInt16BE(const std::uint8_t *data)
{
  return static_cast<std::uint16_t>((static_cast<std::uint16_t>(data[0]) << 8U) |
                                    static_cast<std::uint16_t>(data[1]));
}

// 从字节缓冲区读取大端 32 位无符号整数
std::uint32_t readUInt32BE(const std::uint8_t *data)
{
  return (static_cast<std::uint32_t>(data[0]) << 24U) |
         (static_cast<std::uint32_t>(data[1]) << 16U) |
         (static_cast<std::uint32_t>(data[2]) << 8U) |
         static_cast<std::uint32_t>(data[3]);
}

// sockaddr_in → "IP:port" 字符串，用于日志
std::string sockaddrToString(const sockaddr_in &addr)
{
  char buffer[INET_ADDRSTRLEN] = {0};
  const char *result = inet_ntop(AF_INET, &addr.sin_addr, buffer, sizeof(buffer));
  if (result == nullptr) {
    return "unknown";
  }

  std::ostringstream out;
  out << result << ":" << ntohs(addr.sin_port);
  return out.str();
}
}  // namespace

class K230TcpBridgeNode : public rclcpp::Node
{
public:
  // 构造：声明参数、创建发布者、启动 TCP 监听线程
  K230TcpBridgeNode()
  : Node("k230_tcp_bridge")
  {
    // 通过 ros2 run --ros-args -p key:=val 可覆盖这些默认值
    listen_host_ = this->declare_parameter<std::string>("listen_host", "0.0.0.0");
    listen_port_ = this->declare_parameter<int>("listen_port", 9100);
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/drone/image");
    status_topic_ = this->declare_parameter<std::string>("status_topic", "/k230_snapshot/status");
    publish_status_ = this->declare_parameter<bool>("publish_status", true);

    // 图像话题使用 best_effort，允许丢帧以保证实时性
    image_pub_ = this->create_publisher<drone_msgs::msg::BarcodeCapture>(
      image_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    // 状态话题使用 reliable，确保日志不丢
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10).reliable());

    server_thread_ = std::thread(&K230TcpBridgeNode::serverLoop, this);
  }

  // 析构：通知线程退出 → 等待结束 → 关闭 socket
  ~K230TcpBridgeNode() override
  {
    stopRequested();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    if (server_socket_fd_ >= 0) {
      ::close(server_socket_fd_);
      server_socket_fd_ = -1;
    }
  }

private:
  // TCP 帧解析后的数据包，对应一个完整的 JPEG 快照
  struct SnapshotPacket
  {
    std::uint32_t seq = 0;                  // 帧序号（大端），来自 K230 的序列号
    std::string frame_name;                 // 帧名称（条码/标识）
    std::vector<std::uint8_t> jpeg_data;    // JPEG 图像原始数据
  };

  // 请求停止所有 I/O：设置原子标志 + shutdown socket 唤醒阻塞线程
  void stopRequested()
  {
    stop_requested_.store(true);
    if (server_socket_fd_ >= 0) {
      ::shutdown(server_socket_fd_, SHUT_RDWR);
    }
  }

  // 同时输出到 ROS 日志和 status 话题
  void publishStatus(const std::string &text)
  {
    RCLCPP_INFO(this->get_logger(), "%s", text.c_str());
    if (!publish_status_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  // 从 socket 精确读取 length 字节，处理多种异常：
  //   - EINTR:          被信号打断，重试
  //   - EAGAIN/EWOULDBLOCK: 超时无数据，若已请求退出则返回，否则休眠 10ms 后重试
  //   - recv==0:        对端关闭连接
  //   - 其他 errno:     不可恢复的错误
  bool readExact(int fd, void *buffer, std::size_t length, std::string *error_text)
  {
    std::uint8_t *bytes = static_cast<std::uint8_t *>(buffer);
    std::size_t offset = 0U;

    while (offset < length) {
      const ssize_t received = ::recv(fd, bytes + offset, length - offset, 0);
      if (received > 0) {
        offset += static_cast<std::size_t>(received);
        continue;
      }

      if (received == 0) {
        if (error_text != nullptr) {
          *error_text = "peer closed connection";
        }
        return false;
      }

      if (errno == EINTR) {
        continue;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        if (stop_requested_.load()) {
          if (error_text != nullptr) {
            *error_text = "shutdown requested";
          }
          return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      if (error_text != nullptr) {
        *error_text = std::string("recv failed: ") + std::strerror(errno);
      }
      return false;
    }

    return true;
  }

  // 按协议格式从 socket 读取一帧，每步都做范围校验
  // 步骤：读帧头 → 验魔数 → 读帧名 → 读 payload 头 → 读 JPEG 数据
  bool readPacket(int fd, SnapshotPacket *packet, std::string *error_text)
  {
    // 1. 读取帧头: magic(4B) + name_len(2B)
    std::array<std::uint8_t, kFrameHeaderSize> header{};
    if (!readExact(fd, header.data(), header.size(), error_text)) {
      return false;
    }

    // 校验魔数 "KIMG"，防止错位读取
    if (std::memcmp(header.data(), kMagic.data(), kMagic.size()) != 0) {
      if (error_text != nullptr) {
        *error_text = "bad magic";
      }
      return false;
    }

    // 2. 读取帧名
    const std::uint16_t frame_name_len = readUInt16BE(&header[4]);
    if (frame_name_len == 0U || frame_name_len > kMaxFrameNameLength) {
      if (error_text != nullptr) {
        *error_text = "invalid frame name length";
      }
      return false;
    }

    packet->frame_name.resize(frame_name_len);
    if (!readExact(fd, &packet->frame_name[0], frame_name_len, error_text)) {
      return false;
    }

    // 3. 读取 payload 头: payload_len(4B) + seq(4B)
    std::array<std::uint8_t, kPayloadHeaderSize> payload_header{};
    if (!readExact(fd, payload_header.data(), payload_header.size(), error_text)) {
      return false;
    }

    const std::uint32_t payload_len = readUInt32BE(payload_header.data());
    packet->seq = readUInt32BE(&payload_header[4]);
    if (payload_len == 0U || payload_len > kMaxPayloadLength) {
      if (error_text != nullptr) {
        *error_text = "invalid jpeg payload length";
      }
      return false;
    }

    // 4. 读取 JPEG 数据本体
    packet->jpeg_data.resize(payload_len);
    return readExact(fd, packet->jpeg_data.data(), payload_len, error_text);
  }

  // 快速校验 JPEG 数据：检查 SOI(0xFFD8) 和 EOI(0xFFD9) 标记
  bool isLikelyJpeg(const SnapshotPacket &packet) const
  {
    if (packet.jpeg_data.size() < 4U) {
      return false;
    }

    const std::size_t size = packet.jpeg_data.size();
    return packet.jpeg_data[0] == 0xFFU &&
           packet.jpeg_data[1] == 0xD8U &&
           packet.jpeg_data[size - 2U] == 0xFFU &&
           packet.jpeg_data[size - 1U] == 0xD9U;
  }

  // 组装 ROS 2 消息并发布，同时输出状态日志
  void publishPacket(const SnapshotPacket &packet, const std::string &peer_name)
  {
    drone_msgs::msg::BarcodeCapture msg;
    msg.stamp = this->now();
    msg.barcode = packet.frame_name;     // 帧名作为条码标识
    msg.image_format = "jpeg";
    msg.image_data = packet.jpeg_data;
    image_pub_->publish(msg);

    std::ostringstream status;
    status << "published frame from " << peer_name
           << " name=" << packet.frame_name
           << " seq=" << packet.seq
           << " size=" << packet.jpeg_data.size()
           << " bytes";
    publishStatus(status.str());
  }

  // 处理单个客户端连接：设置读超时 → 循环读帧 → 校验 → 发布
  void handleClient(int client_fd, const std::string &peer_name)
  {
    // 设置 1 秒 recv 超时，防止客户端断连时线程永久阻塞
    timeval timeout {};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    ::setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    publishStatus("client connected: " + peer_name);

    while (!stop_requested_.load()) {
      SnapshotPacket packet;
      std::string error_text;
      if (!readPacket(client_fd, &packet, &error_text)) {
        // 对端正常断开或节点主动关闭不视为错误
        if (error_text != "peer closed connection" && error_text != "shutdown requested") {
          publishStatus("client " + peer_name + " stopped: " + error_text);
        }
        break;
      }

      // 校验 JPEG 头尾，非 JPEG 数据丢弃并告警
      if (!isLikelyJpeg(packet)) {
        std::ostringstream status;
        status << "drop packet from " << peer_name
               << " name=" << packet.frame_name
               << " len=" << packet.jpeg_data.size();
        if (!packet.jpeg_data.empty()) {
          status << " head=" << std::hex << static_cast<int>(packet.jpeg_data.front())
                 << " tail=" << static_cast<int>(packet.jpeg_data.back()) << std::dec;
        }
        publishStatus(status.str());
        continue;
      }

      publishPacket(packet, peer_name);
    }
  }

  // TCP 服务端主循环（独立线程）：
  //   socket → bind → listen → select 等待连接 → accept → handleClient → close
  //   select 设置 1 秒超时以便定期检查 stop_requested_ 实现优雅退出
  void serverLoop()
  {
    server_socket_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket_fd_ < 0) {
      publishStatus(std::string("socket create failed: ") + std::strerror(errno));
      return;
    }

    // 允许端口复用，崩溃重启时可立即 bind
    int reuse_value = 1;
    (void)::setsockopt(server_socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse_value, sizeof(reuse_value));

    sockaddr_in server_addr {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(static_cast<std::uint16_t>(listen_port_));
    if (::inet_pton(AF_INET, listen_host_.c_str(), &server_addr.sin_addr) != 1) {
      publishStatus("invalid listen_host: " + listen_host_);
      return;
    }

    if (::bind(server_socket_fd_, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) != 0) {
      publishStatus(std::string("bind failed: ") + std::strerror(errno));
      return;
    }

    if (::listen(server_socket_fd_, 4) != 0) {
      publishStatus(std::string("listen failed: ") + std::strerror(errno));
      return;
    }

    publishStatus("listening on " + listen_host_ + ":" + std::to_string(listen_port_) +
                  ", topic=" + image_topic_);

    while (!stop_requested_.load()) {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(server_socket_fd_, &read_fds);

      // select 1 秒超时，确保能定期响应 stop_requested_ 退出信号
      timeval accept_timeout {};
      accept_timeout.tv_sec = 1;
      accept_timeout.tv_usec = 0;

      const int ready = ::select(server_socket_fd_ + 1, &read_fds, nullptr, nullptr, &accept_timeout);
      if (ready < 0) {
        if (stop_requested_.load()) {
          break;
        }
        if (errno == EINTR) {
          continue;
        }
        publishStatus(std::string("select failed: ") + std::strerror(errno));
        continue;
      }
      if (ready == 0) {
        continue;  // 超时，回到循环检查退出标志
      }

      sockaddr_in client_addr {};
      socklen_t client_len = sizeof(client_addr);
      const int client_fd = ::accept(server_socket_fd_,
                                     reinterpret_cast<sockaddr *>(&client_addr),
                                     &client_len);

      if (client_fd < 0) {
        if (stop_requested_.load()) {
          break;
        }

        if (errno == EINTR) {
          continue;
        }

        publishStatus(std::string("accept failed: ") + std::strerror(errno));
        continue;
      }

      const std::string peer_name = sockaddrToString(client_addr);
      handleClient(client_fd, peer_name);  // 同步处理一个客户端
      ::close(client_fd);                   // 客户端断开后关闭 fd
    }
  }

  // ROS 2 参数
  std::string listen_host_;
  int listen_port_ = 9100;
  std::string image_topic_;
  std::string status_topic_;
  bool publish_status_ = true;

  // ROS 2 发布者
  rclcpp::Publisher<drone_msgs::msg::BarcodeCapture>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // TCP 服务端状态
  std::thread server_thread_;
  std::atomic_bool stop_requested_ {false};
  int server_socket_fd_ = -1;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<K230TcpBridgeNode>();
  rclcpp::spin(node);   // 主线程阻塞，TCP 处理在独立线程中运行
  rclcpp::shutdown();
  return 0;
}
