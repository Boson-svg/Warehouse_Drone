#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <termios.h>

#include "drone_msgs/msg/barcode_capture.hpp"
#include "drone_msgs/msg/k230_animal_center.hpp"
#include "drone_msgs/msg/k230_animal_target.hpp"
#include "drone_msgs/msg/k230_animal_targets.hpp"
#include "drone_msgs/msg/k230_capture_ready.hpp"
#include "drone_msgs/msg/k230_record_result.hpp"
#include "drone_msgs/msg/k230_scan_point_done.hpp"
#include "drone_msgs/msg/world_group.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class K230AnimalsUartRos2Node : public rclcpp::Node
{
public:
	K230AnimalsUartRos2Node();
	~K230AnimalsUartRos2Node() override;

private:
	static constexpr size_t kHeaderSize = 13;
	static constexpr size_t kCrcSize = 2;
	static constexpr size_t kMaxPayloadSize = 2 * 1024 * 1024;
	static constexpr double kDefaultScanRadiusM = 0.28;
	static constexpr uint8_t kPacketTypeCaptureCommand = 4;
	static constexpr uint8_t kPacketTypeRecordResult = 5;
	static constexpr double kDefaultRecordDeadbandM = 0.1;
	static constexpr double kDefaultPendingCaptureTimeoutS = 5.0;
	static constexpr double kRoutePointEpsilonM = 1e-3;

	static speed_t baudrateToSpeed(int baudrate);
	static uint32_t readU32Be(const uint8_t *p);
	static uint16_t readU16Be(const uint8_t *p);
	static uint16_t crc16Ccitt(const uint8_t *data, size_t len);
	static bool isLikelyJpeg(const std::vector<uint8_t> &payload);
	static bool jsonBoolFieldTrue(const std::string &json_text, const std::string &field);
	static std::string jsonStringField(const std::string &json_text, const std::string &field);
	static bool jsonNumberField(const std::string &json_text, const std::string &field, double &value);

	void openSerial();
	std::string makeImageBarcode(uint32_t seq) const;
	void publishImagePacket(uint32_t seq, const std::vector<uint8_t> &payload);
	void publishDetectionPacket(uint32_t seq, const std::string &json_text);
	void publishTargetsPacket(uint32_t seq, const std::string &json_text);
	void handlePacket(uint8_t type, uint32_t seq, const std::vector<uint8_t> &payload);
	void parsePackets();
	void readSerialOnce();
	void publishHeartbeat();

	void handleRoutePoints(const drone_msgs::msg::WorldGroup::SharedPtr msg);
	bool routeMatchesScanPoints(const drone_msgs::msg::WorldGroup &msg) const;
	void handleLocalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void handleCaptureReady(const drone_msgs::msg::K230CaptureReady::SharedPtr msg);
	void handleScanPointDone(const drone_msgs::msg::K230ScanPointDone::SharedPtr msg);

	bool sendCaptureCommand(const drone_msgs::msg::K230CaptureReady &msg);
	bool writePacket(uint8_t type, uint32_t seq, const std::string &payload);

	bool isCaptureTargetRecorded(const drone_msgs::msg::K230CaptureReady &msg) const;
	bool pendingCaptureTimedOut();
	void clearTimedOutPendingCapture();
	void rememberPendingCapture(const drone_msgs::msg::K230CaptureReady &msg);
	void commitPendingCaptureIfMatched(const drone_msgs::msg::K230RecordResult &msg);

	void publishRecordResultFromPacket(uint32_t seq, const std::string &json_text);
	void publishSkippedRecordResult(
		const drone_msgs::msg::K230CaptureReady &msg,
		const std::string &reason);
	int32_t findScanPointOffsetByIndex(int32_t scan_point_index) const;

	int32_t findActiveScanPointOffset();

	std::string _serial_port;
	std::string _image_topic;
	std::string _detect_topic;
	std::string _center_topic;
	std::string _last_label;
	std::string _heartbeat_topic;
	std::string _targets_topic;
	std::string _route_topic;
	std::string _local_pose_topic;
	std::string _capture_ready_topic;
	std::string _record_result_topic;
	std::string _scan_point_done_topic;

	struct ScanPoint
	{
		int32_t index{-1};
		double x{0.0};
		double y{0.0};
		bool scanned{false};
	};

	struct RecordedPose
	{
		double x{0.0};
		double y{0.0};
		int32_t scan_point_index{-1};
		std::string label;
		uint32_t label_instance_id{0};
	};

	struct PendingCapture
	{
		bool active{false};
		uint32_t frame_seq{0};
		int32_t scan_point_index{-1};
		std::string label;
		uint32_t label_instance_id{0};
		double x{0.0};
		double y{0.0};
		rclcpp::Time request_time{0, 0, RCL_ROS_TIME};
	};

	std::vector<ScanPoint> _scan_points;
	std::vector<RecordedPose> _recorded_poses;
	PendingCapture _pending_capture;
	bool _has_local_pose{false};
	double _current_x{0.0};
	double _current_y{0.0};
	double _scan_radius_m{kDefaultScanRadiusM};
	double _record_deadband_m{kDefaultRecordDeadbandM};
	double _pending_capture_timeout_s{kDefaultPendingCaptureTimeoutS};

	int _baudrate{0};
	int _serial_fd{-1};

	rclcpp::Publisher<drone_msgs::msg::BarcodeCapture>::SharedPtr _image_pub;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _detect_pub;
	rclcpp::Publisher<drone_msgs::msg::K230AnimalCenter>::SharedPtr _center_pub;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _heartbeat_pub;
	rclcpp::Publisher<drone_msgs::msg::K230AnimalTargets>::SharedPtr _targets_pub;
	rclcpp::Publisher<drone_msgs::msg::K230RecordResult>::SharedPtr _record_result_pub;

	rclcpp::Subscription<drone_msgs::msg::WorldGroup>::SharedPtr _route_points_sub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _local_pose_sub;
	rclcpp::Subscription<drone_msgs::msg::K230CaptureReady>::SharedPtr _capture_ready_sub;
	rclcpp::Subscription<drone_msgs::msg::K230ScanPointDone>::SharedPtr _scan_point_done_sub;

	rclcpp::TimerBase::SharedPtr _heartbeat_timer;
	rclcpp::TimerBase::SharedPtr _read_timer;

	std::vector<uint8_t> _rx_buffer;
	size_t _total_rx_bytes{0};
};
