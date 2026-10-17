#include "drone_perception/qr_vision_node.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <zbar.h>

namespace
{
using SteadyClock = std::chrono::steady_clock;
static constexpr int kBpuInputWidthPx = 640;
static constexpr int kBpuInputHeightPx = 640;
static constexpr int kDefaultShelfCodeStableFrames = 3;
static constexpr int kDefaultShelfCodeLostToleranceFrames = 2;
static constexpr int kDefaultBarcodeCaptureJpegQuality = 90;
static constexpr float kDefaultOcrYoloPaddingRatio = 0.15F;
static constexpr float kDefaultPackageCapturePaddingRatio = 0.05F;
static constexpr double kDefaultCaptureCollectionDurationS = 2.0;
static constexpr double kDefaultPackageCaptureDeadzoneRatio = 0.25;
static constexpr double kDefaultPackageCaptureMinScore = 0.50;
static constexpr const char *kDefaultBarcodeCaptureTopic = "/drone/image";
static constexpr const char *kDefaultHoverActiveTopic = "/mission/hover_active";
static constexpr const char *kJpegImageFormat = "jpeg";
static constexpr const char *kMissingCaptureField = "NAN";
static constexpr std::size_t kCameraExposureControlIndex = 0U;
static constexpr std::size_t kCameraGainControlIndex = 1U;
static constexpr std::size_t kCameraWhiteBalanceControlIndex = 2U;
static constexpr std::size_t kCameraAutoExposureControlIndex = 0U;
static constexpr std::size_t kCameraAutoWhiteBalanceControlIndex = 1U;
static constexpr std::size_t kCameraAutoExposurePriorityControlIndex = 2U;
static constexpr int kCameraControlsPanelWidthPx = 720;
static constexpr int kCameraControlsPanelHeightPx = 360;
static constexpr int kCameraControlsSendDebounceMs = 120;
#if DRONE_PERCEPTION_HAS_BPU
static constexpr int32_t kYoloClassQrcode = 0;
static constexpr int32_t kYoloClassPackage = 1;
static constexpr int32_t kYoloClassShelfTag = 2;
static constexpr float kVisualCodeRoiPaddingXRatio = 0.15F;
static constexpr float kVisualCodeRoiPaddingYRatio = 0.15F;
static constexpr int kQrAdaptiveThresholdBlockSizePx = 31;
static constexpr double kQrAdaptiveThresholdOffset = 5.0;
static constexpr int kQrPreprocessPreviewMaxSizePx = 180;

#endif
static constexpr std::size_t kBpuInputYSize =
    static_cast<std::size_t>(kBpuInputWidthPx) *
    static_cast<std::size_t>(kBpuInputHeightPx);
static constexpr std::size_t kBpuInputUvSize = kBpuInputYSize / 4;
static constexpr std::size_t kBpuInputNv12Size = kBpuInputYSize + kBpuInputUvSize * 2;

double elapsedMs(
    const SteadyClock::time_point &start,
    const SteadyClock::time_point &end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

bool isValidCapturePrefixedCode(
    const std::string &code,
    const std::string &prefix)
{
  if (code.rfind(prefix, 0U) != 0U || code.size() <= prefix.size()) {
    return false;
  }

  bool has_payload = false;

  for (std::size_t i = prefix.size(); i < code.size(); ++i) {
    const unsigned char value = static_cast<unsigned char>(code[i]);

    if (std::isalnum(value) != 0 || code[i] == '_') {
      has_payload = true;
      continue;
    }

    return false;
  }

  return has_payload;
}

bool isValidShelfCaptureCode(const std::string &code)
{
  if (code.size() < 5U) {
    return false;
  }

  if (code[0] != 'A' && code[0] != 'B') {
    return false;
  }

  const std::size_t first_dash = code.find('-');
  const std::size_t second_dash = code.find(
      '-',
      first_dash == std::string::npos ? 0U : first_dash + 1U);

  if (first_dash != 1U || second_dash == std::string::npos) {
    return false;
  }

  if (code.find('-', second_dash + 1U) != std::string::npos) {
    return false;
  }

  const std::string middle = code.substr(first_dash + 1U, second_dash - first_dash - 1U);
  const std::string tail = code.substr(second_dash + 1U);

  if (middle.empty() || tail.empty()) {
    return false;
  }

  const auto is_digit = [](char value) {
    return std::isdigit(static_cast<unsigned char>(value)) != 0;
  };

  return std::all_of(middle.begin(), middle.end(), is_digit) &&
      std::all_of(tail.begin(), tail.end(), is_digit);
}

#if DRONE_PERCEPTION_HAS_BPU
struct ParsedVisualCode
{
  std::string category;
  std::string code;
};

struct VisualCodeScanStats
{
  int symbol_count{0};
  int accepted_count{0};
};

void configureVisualCodeScanner(
    zbar::ImageScanner &scanner)
{
  scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
  scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

std::string trimAndUppercase(const std::string &text)
{
  const auto begin = std::find_if_not(
      text.begin(),
      text.end(),
      [](unsigned char value) {
        return std::isspace(value) != 0;
      });

  const auto end = std::find_if_not(
      text.rbegin(),
      text.rend(),
      [](unsigned char value) {
        return std::isspace(value) != 0;
      }).base();

  if (begin >= end) {
    return {};
  }

  std::string normalized(begin, end);
  std::transform(
      normalized.begin(),
      normalized.end(),
      normalized.begin(),
      [](unsigned char value) {
        return static_cast<char>(std::toupper(value));
      });

  return normalized;
}

bool isValidShortPrefixedCode(
    const std::string &code,
    const std::string &prefix)
{
  if (code.rfind(prefix, 0U) != 0U || code.size() <= prefix.size()) {
    return false;
  }

  bool has_payload = false;

  for (std::size_t i = prefix.size(); i < code.size(); ++i) {
    const unsigned char value = static_cast<unsigned char>(code[i]);

    if (std::isalnum(value) != 0 || code[i] == '_') {
      has_payload = true;
      continue;
    }

    return false;
  }

  return has_payload;
}

void appendParsedVisualCode(
    const std::string &code,
    std::vector<ParsedVisualCode> &parsed_codes)
{
  if (isValidShortPrefixedCode(code, "SKU-")) {
    parsed_codes.push_back({"sku", code});
    return;
  }

  if (isValidShortPrefixedCode(code, "PKG-")) {
    parsed_codes.push_back({"pkg", code});
    return;
  }
}

std::vector<ParsedVisualCode> parseVisualCodes(const std::string &code)
{
  std::vector<ParsedVisualCode> parsed_codes;
  std::size_t begin = 0U;
  std::size_t token_count = 0U;

  while (begin <= code.size()) {
    const std::size_t separator = code.find('|', begin);
    const std::size_t end =
        separator == std::string::npos ? code.size() : separator;
    const std::string token = trimAndUppercase(code.substr(begin, end - begin));

    if (!token.empty()) {
      ++token_count;
      appendParsedVisualCode(token, parsed_codes);
    }

    if (separator == std::string::npos) {
      break;
    }

    begin = separator + 1U;
  }

  const bool has_sku = std::any_of(
      parsed_codes.begin(),
      parsed_codes.end(),
      [](const ParsedVisualCode &parsed_code) {
        return parsed_code.category == "sku";
      });
  const bool has_pkg = std::any_of(
      parsed_codes.begin(),
      parsed_codes.end(),
      [](const ParsedVisualCode &parsed_code) {
        return parsed_code.category == "pkg";
      });

  if (token_count != 2U || parsed_codes.size() != 2U || !has_sku || !has_pkg) {
    return {};
  }

  return parsed_codes;
}

std::string formatParsedVisualCodeCategories(
    const std::vector<ParsedVisualCode> &parsed_codes)
{
  if (parsed_codes.empty()) {
    return "ignored";
  }

  std::string categories;

  for (const ParsedVisualCode &parsed_code : parsed_codes) {
    if (!categories.empty()) {
      categories += ",";
    }

    categories += parsed_code.category;
  }

  return categories;
}
#endif
}  // namespace

QrVisionNode::QrVisionNode()
    : Node("qr_vision_node")
{
  declareParameters();
  barcode_capture_pub_ = this->create_publisher<drone_msgs::msg::BarcodeCapture>(
      barcode_capture_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  initializeBpuDetector();
  initializeBpuOcrPipeline();
  initializeVisualCodeDecoder();

#if DRONE_PERCEPTION_HAS_REALSENSE
  if (camera_input_mode_ == "d435_direct") {
    initializeDirectCapture();
  }
#else
  if (camera_input_mode_ == "d435_direct") {
    throw std::runtime_error(
        "camera_input_mode=d435_direct requires librealsense2, but this build "
        "was compiled without it (camera_input_mode=ros only)");
  }
#endif

#if DRONE_PERCEPTION_HAS_REALSENSE && DRONE_PERCEPTION_HAS_RDK_MEDIA_CODEC
  // The video-stream worker consumes the same D435ColorFrame as detection and
  // pushes H.264 toward the local MediaMTX started by start_d435.sh. It must be
  // started after initializeDirectCapture() so the capture slot exists.
  if (camera_input_mode_ == "d435_direct" && video_stream_enabled_) {
    startVideoStream();
  }
#endif

  // HighGUI windows must be created and driven from the main thread (GTK
  // initializes there). The window is created here; the worker thread only
  // renders frames, and a wall timer on the executor thread runs imshow/waitKey.
  if (debug_view_)
  {
    try
    {
      cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
      debug_window_created_ = true;
    }
    catch (const cv::Exception &e)
    {
      debug_view_ = false;
      debug_window_created_ = false;
      RCLCPP_WARN(get_logger(), "Cannot open debug window: %s", e.what());
    }
  }

  if (debug_window_created_)
  {
    debug_display_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        [this]() { updateDebugDisplay(); });
  }

  initializeSubscriptions();

  RCLCPP_INFO(
      get_logger(),
      "RGB BPU video stream ready. mode=color qr_decode=true yolo_bpu=%s ocr_rec_bpu=%s color=%s camera_info=%s",
      enable_bpu_ ? "true" : "false",
      enable_bpu_ocr_ ? "true" : "false",
      color_topic_.c_str(),
      camera_info_topic_.c_str());

  startVisionWorker();
}

QrVisionNode::~QrVisionNode()
{
  color_sub_.reset();
  hover_active_sub_.reset();
  camera_info_sub_.reset();
  camera_controls_timer_.reset();
  camera_param_client_.reset();
  // Stop the frame producers first (no new submits), then drain the encoder
  // worker and close the FFmpeg/MediaMTX pipe.
  stopVisionWorker();

#if DRONE_PERCEPTION_HAS_REALSENSE && DRONE_PERCEPTION_HAS_RDK_MEDIA_CODEC
  stopVideoStream();
#endif

  if (debug_window_created_)
  {
    cv::destroyWindow(window_name_);
  }
}

void QrVisionNode::initializeVisualCodeDecoder()
{
#if DRONE_PERCEPTION_HAS_BPU
  visual_code_scanner_ = std::make_unique<zbar::ImageScanner>();
  configureVisualCodeScanner(*visual_code_scanner_);
  qr_clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));
#endif
}

void QrVisionNode::startVisionWorker()
{
  {
    std::lock_guard<std::mutex> lock(vision_worker_mutex_);
    vision_worker_running_ = true;
  }

  vision_worker_ = std::thread(&QrVisionNode::visionWorkerLoop, this);
}

void QrVisionNode::stopVisionWorker()
{
  {
    std::lock_guard<std::mutex> lock(vision_worker_mutex_);
    vision_worker_running_ = false;
    latest_color_frame_ = {};
    pending_hover_events_.clear();
  }

  vision_worker_cv_.notify_all();

#if DRONE_PERCEPTION_HAS_REALSENSE
  // The direct-capture worker may be blocked in
  // D435ColorCapture::waitForLatest(), which only unblocks when the capture
  // stops (its condition variable is signaled from stop()).
  if (d435_capture_) {
    d435_capture_->stop();
  }
#endif

  if (vision_worker_.joinable()) {
    vision_worker_.join();
  }
}

void QrVisionNode::visionWorkerLoop()
{
#if DRONE_PERCEPTION_HAS_REALSENSE
  if (camera_input_mode_ == "d435_direct") {
    runDirectVisionLoop();
    return;
  }
#endif
  runRosVisionLoop();
}

void QrVisionNode::runRosVisionLoop()
{
  while (rclcpp::ok()) {
    PendingColorFrame color_frame;
    PendingHoverEvent hover_event;
    bool has_color_frame = false;
    bool has_hover_event = false;

    {
      std::unique_lock<std::mutex> lock(vision_worker_mutex_);
      vision_worker_cv_.wait(lock, [this] {
        return !vision_worker_running_ ||
               static_cast<bool>(latest_color_frame_.message) ||
               !pending_hover_events_.empty();
      });

      if (!vision_worker_running_) {
        break;
      }

      const bool hover_is_next =
          !pending_hover_events_.empty() &&
          (!latest_color_frame_.message ||
           pending_hover_events_.front().sequence < latest_color_frame_.sequence);

      if (hover_is_next) {
        hover_event = pending_hover_events_.front();
        pending_hover_events_.pop_front();
        has_hover_event = true;
      } else if (latest_color_frame_.message) {
        color_frame = std::move(latest_color_frame_);
        latest_color_frame_ = {};
        has_color_frame = true;
      }
    }

    if (has_hover_event) {
      applyHoverActive(hover_event.active);
      continue;
    }

    if (!has_color_frame || !color_frame.message) {
      continue;
    }

    const auto process_t0 = SteadyClock::now();
    last_queue_wait_ms_ = elapsedMs(color_frame.enqueue_time, process_t0);
    queue_wait_window_.push(last_queue_wait_ms_);
    updateFrameAge(*color_frame.message);

    cv_bridge::CvImageConstPtr color_bridge;
    try {
      color_bridge = cv_bridge::toCvShare(color_frame.message, "bgr8");
      processFrame(color_bridge, process_t0, "color");
    } catch (const cv_bridge::Exception &ex) {
      ++failed_frame_count_;
      last_process_ms_ = elapsedMs(process_t0, SteadyClock::now());
      process_window_.push(last_process_ms_);
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          log_throttle_ms_,
          "cv_bridge color conversion failed: %s",
          ex.what());
    }

    reportPerformanceBaseline();
  }
}

void QrVisionNode::TimingWindow::push(double value)
{
  if (value < 0.0) {
    return;
  }

  samples.push_back(value);
  if (samples.size() > kCapacity) {
    samples.pop_front();
  }
}

double QrVisionNode::TimingWindow::percentile(double ratio) const
{
  if (samples.empty()) {
    return 0.0;
  }

  std::vector<double> sorted(samples.begin(), samples.end());
  std::sort(sorted.begin(), sorted.end());
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);
  const std::size_t index = static_cast<std::size_t>(
      std::ceil(clamped_ratio * static_cast<double>(sorted.size()))) - 1U;
  return sorted[std::min(index, sorted.size() - 1U)];
}

void QrVisionNode::declareParameters()
{
  color_topic_ = this->declare_parameter<std::string>(
      "color_topic", "/camera/camera/color/image_raw");
  camera_info_topic_ = this->declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/color/camera_info");
  window_name_ = this->declare_parameter<std::string>(
      "window_name", "QR D435i View");
  debug_view_ = this->declare_parameter<bool>("debug_view", true);
  qr_preprocess_enabled_ = this->declare_parameter<bool>(
      "qr_preprocess_enabled",
      true);
  camera_controls_enabled_ = this->declare_parameter<bool>(
      "camera_controls_enabled",
      false);
  camera_param_node_ = this->declare_parameter<std::string>(
      "camera_param_node",
      "/camera/camera");
  (void)this->declare_parameter<std::string>(
      "camera_controls_window_name",
      "D435i Controls");
  camera_controls_update_period_ms_ = this->declare_parameter<int>(
      "camera_controls_update_period_ms",
      200);
  barcode_capture_topic_ = this->declare_parameter<std::string>(
      "barcode_capture_topic",
      kDefaultBarcodeCaptureTopic);
  hover_active_topic_ = this->declare_parameter<std::string>(
      "hover_active_topic",
      kDefaultHoverActiveTopic);
  camera_input_mode_ = this->declare_parameter<std::string>(
      "camera_input_mode", "ros");
  d435_serial_ = this->declare_parameter<std::string>(
      "d435_serial", "327122074056");
  d435_width_ = this->declare_parameter<int>("d435_width", 640);
  d435_height_ = this->declare_parameter<int>("d435_height", 480);
  d435_fps_ = this->declare_parameter<int>("d435_fps", 30);
  d435_wait_timeout_ms_ = this->declare_parameter<int>(
      "d435_wait_timeout_ms", 2000);
  d435_reconnect_delay_ms_ = this->declare_parameter<int>(
      "d435_reconnect_delay_ms", 2000);
  direct_input_debug_bgr_ = this->declare_parameter<bool>(
      "direct_input_debug_bgr", true);
  video_stream_enabled_ = this->declare_parameter<bool>(
      "video_stream_enabled", false);
  video_stream_width_ = this->declare_parameter<int>(
      "video_stream_width", 640);
  video_stream_height_ = this->declare_parameter<int>(
      "video_stream_height", 480);
  video_stream_fps_ = this->declare_parameter<int>(
      "video_stream_fps", 30);
  video_stream_bitrate_kbps_ = this->declare_parameter<int>(
      "video_stream_bitrate_kbps", 1500);
  video_stream_gop_ = this->declare_parameter<int>(
      "video_stream_gop", 15);
  video_stream_publish_url_ = this->declare_parameter<std::string>(
      "video_stream_publish_url", "rtsp://127.0.0.1:8554/d435i");
  log_throttle_ms_ = this->declare_parameter<int>("log_throttle_ms", 2000);
  shelf_code_stable_frames_ = this->declare_parameter<int>(
      "shelf_code_stable_frames",
      kDefaultShelfCodeStableFrames);
  shelf_code_lost_tolerance_frames_ = this->declare_parameter<int>(
      "shelf_code_lost_tolerance_frames",
      kDefaultShelfCodeLostToleranceFrames);
  barcode_capture_jpeg_quality_ = this->declare_parameter<int>(
      "barcode_capture_jpeg_quality",
      kDefaultBarcodeCaptureJpegQuality);

  enable_bpu_ = this->declare_parameter<bool>(
      "enable_bpu",
      DRONE_PERCEPTION_HAS_BPU != 0);
  enable_bpu_ocr_ = this->declare_parameter<bool>(
      "enable_bpu_ocr",
      DRONE_PERCEPTION_HAS_BPU != 0);
  bpu_model_path_ = this->declare_parameter<std::string>(
      "bpu_model_path",
      "/home/sunrise/warehouse_ws/src/drone_perception/best(2)_bayese_640x640_nv12.bin");
  ocr_rec_model_path_ = this->declare_parameter<std::string>(
      "ocr_rec_model_path",
      "/home/sunrise/warehouse_ws/src/drone_perception/en_PP-OCRv3_rec_48x320_rgb.bin");
  ocr_yolo_padding_ratio_ = static_cast<float>(this->declare_parameter<double>(
      "ocr_yolo_padding_ratio",
      kDefaultOcrYoloPaddingRatio));
  package_capture_padding_ratio_ = static_cast<float>(this->declare_parameter<double>(
      "package_capture_padding_ratio",
      kDefaultPackageCapturePaddingRatio));
  capture_collection_duration_s_ = this->declare_parameter<double>(
      "capture_collection_duration_s",
      kDefaultCaptureCollectionDurationS);
  package_capture_deadzone_ratio_ = this->declare_parameter<double>(
      "package_capture_deadzone_ratio",
      kDefaultPackageCaptureDeadzoneRatio);
  package_capture_min_score_ = this->declare_parameter<double>(
      "package_capture_min_score",
      kDefaultPackageCaptureMinScore);

  if (log_throttle_ms_ <= 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "log_throttle_ms must be greater than 0, reset to 2000");
    log_throttle_ms_ = 2000;
  }

  if (camera_controls_update_period_ms_ < 30)
  {
    RCLCPP_WARN(
        get_logger(),
        "camera_controls_update_period_ms must be at least 30, reset to 200");
    camera_controls_update_period_ms_ = 200;
  }

  if (shelf_code_stable_frames_ <= 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "shelf_code_stable_frames must be greater than 0, reset to %d",
        kDefaultShelfCodeStableFrames);
    shelf_code_stable_frames_ = kDefaultShelfCodeStableFrames;
  }

  if (shelf_code_lost_tolerance_frames_ < 0)
  {
    RCLCPP_WARN(
        get_logger(),
        "shelf_code_lost_tolerance_frames must be greater than or equal to 0, reset to %d",
        kDefaultShelfCodeLostToleranceFrames);
    shelf_code_lost_tolerance_frames_ = kDefaultShelfCodeLostToleranceFrames;
  }

  if (ocr_yolo_padding_ratio_ < 0.0F)
  {
    RCLCPP_WARN(
        get_logger(),
        "ocr_yolo_padding_ratio must be greater than or equal to 0, reset to %.2f",
        kDefaultOcrYoloPaddingRatio);
    ocr_yolo_padding_ratio_ = kDefaultOcrYoloPaddingRatio;
  }

  if (package_capture_padding_ratio_ < 0.0F)
  {
    RCLCPP_WARN(
        get_logger(),
        "package_capture_padding_ratio must be greater than or equal to 0, reset to %.2f",
        kDefaultPackageCapturePaddingRatio);
    package_capture_padding_ratio_ = kDefaultPackageCapturePaddingRatio;
  }

  if (capture_collection_duration_s_ <= 0.0)
  {
    RCLCPP_WARN(
        get_logger(),
        "capture_collection_duration_s must be greater than 0, reset to %.2f",
        kDefaultCaptureCollectionDurationS);
    capture_collection_duration_s_ = kDefaultCaptureCollectionDurationS;
  }

  if (package_capture_deadzone_ratio_ <= 0.0 ||
      package_capture_deadzone_ratio_ > 0.5)
  {
    RCLCPP_WARN(
        get_logger(),
        "package_capture_deadzone_ratio must be in (0, 0.5], reset to %.2f",
        kDefaultPackageCaptureDeadzoneRatio);
    package_capture_deadzone_ratio_ = kDefaultPackageCaptureDeadzoneRatio;
  }

  if (package_capture_min_score_ < 0.0 ||
      package_capture_min_score_ > 1.0)
  {
    RCLCPP_WARN(
        get_logger(),
        "package_capture_min_score must be in [0, 1], reset to %.2f",
        kDefaultPackageCaptureMinScore);
    package_capture_min_score_ = kDefaultPackageCaptureMinScore;
  }

  if (barcode_capture_jpeg_quality_ < 1 || barcode_capture_jpeg_quality_ > 100)
  {
    RCLCPP_WARN(
        get_logger(),
        "barcode_capture_jpeg_quality must be in [1, 100], reset to %d",
        kDefaultBarcodeCaptureJpegQuality);
    barcode_capture_jpeg_quality_ = kDefaultBarcodeCaptureJpegQuality;
  }

  if (camera_input_mode_ != "ros" && camera_input_mode_ != "d435_direct")
  {
    RCLCPP_WARN(
        get_logger(),
        "camera_input_mode must be 'ros' or 'd435_direct', got '%s'; reset to 'ros'",
        camera_input_mode_.c_str());
    camera_input_mode_ = "ros";
  }

  if (d435_wait_timeout_ms_ < 100)
  {
    RCLCPP_WARN(
        get_logger(),
        "d435_wait_timeout_ms must be at least 100, reset to 2000");
    d435_wait_timeout_ms_ = 2000;
  }

  if (d435_reconnect_delay_ms_ < 100)
  {
    RCLCPP_WARN(
        get_logger(),
        "d435_reconnect_delay_ms must be at least 100, reset to 2000");
    d435_reconnect_delay_ms_ = 2000;
  }
}

void QrVisionNode::initializeBpuDetector()
{
#if DRONE_PERCEPTION_HAS_BPU
  if (!enable_bpu_) {
    return;
  }

  if (bpu_model_path_.empty()) {
    RCLCPP_WARN(
        get_logger(),
        "BPU inference is enabled but bpu_model_path is empty");
    enable_bpu_ = false;
    return;
  }

  try {
    bpu_detector_ = std::make_unique<BpuYoloDetector>(bpu_model_path_);

    RCLCPP_INFO(
        get_logger(),
        "BPU detector initialized. model=%s",
        bpu_model_path_.c_str());
  } catch (const std::exception &e) {
    bpu_detector_.reset();
    enable_bpu_ = false;

    RCLCPP_ERROR(
        get_logger(),
        "Failed to initialize BPU detector: %s",
        e.what());
  }
#else
  if (enable_bpu_) {
    RCLCPP_WARN(
        get_logger(),
        "BPU inference requested but this build has no RDK BPU SDK");
    enable_bpu_ = false;
  }
#endif
}

void QrVisionNode::initializeBpuOcrPipeline()
{
#if DRONE_PERCEPTION_HAS_BPU
  if (!enable_bpu_ocr_) {
    return;
  }

  try {
    BpuOcrConfig config{};
    config.rec_model_path = ocr_rec_model_path_;
    config.enable_detection_model = false;
    bpu_ocr_pipeline_ = std::make_unique<BpuOcrPipeline>(config);

    RCLCPP_INFO(
        get_logger(),
        "BPU OCR rec pipeline initialized. rec_model=%s yolo_padding_ratio=%.2f",
        config.rec_model_path.empty() ? "<auto>" : config.rec_model_path.c_str(),
        ocr_yolo_padding_ratio_);
  } catch (const std::exception &e) {
    bpu_ocr_pipeline_.reset();
    enable_bpu_ocr_ = false;

    RCLCPP_ERROR(
        get_logger(),
        "Failed to initialize BPU OCR pipeline: %s",
        e.what());
  }
#else
  if (enable_bpu_ocr_) {
    RCLCPP_WARN(
        get_logger(),
        "BPU OCR requested but this build has no RDK BPU SDK");
  }

  enable_bpu_ocr_ = false;
#endif
}

bool QrVisionNode::prepareBpuInput(const cv::Mat &color_image)
{
  bpu_preprocess_timing_ = {};
  const auto total_t0 = SteadyClock::now();

  if (color_image.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "BPU input image is empty");

    return false;
  }

  if (color_image.channels() != 3) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "BPU input expects BGR image with 3 channels, got %d",
        color_image.channels());

    return false;
  }

  try {
    const float scale = std::min(
        static_cast<float>(kBpuInputWidthPx) / static_cast<float>(color_image.cols),
        static_cast<float>(kBpuInputHeightPx) / static_cast<float>(color_image.rows));
    const int resized_width = std::clamp(
        static_cast<int>(std::round(static_cast<float>(color_image.cols) * scale)),
        1,
        kBpuInputWidthPx);
    const int resized_height = std::clamp(
        static_cast<int>(std::round(static_cast<float>(color_image.rows) * scale)),
        1,
        kBpuInputHeightPx);
    const int pad_x = (kBpuInputWidthPx - resized_width) / 2;
    const int pad_y = (kBpuInputHeightPx - resized_height) / 2;

    const auto resize_t0 = SteadyClock::now();
    cv::Mat resized_bgr;
    cv::resize(
        color_image,
        resized_bgr,
        cv::Size(resized_width, resized_height),
        0.0,
        0.0,
        cv::INTER_LINEAR);
    const auto resize_t1 = SteadyClock::now();
    bpu_preprocess_timing_.resize_ms = elapsedMs(resize_t0, resize_t1);

    const auto letterbox_t0 = SteadyClock::now();
    cv::Mat letterbox_bgr(
        kBpuInputHeightPx,
        kBpuInputWidthPx,
        color_image.type(),
        cv::Scalar(114, 114, 114));
    resized_bgr.copyTo(letterbox_bgr(cv::Rect(
        pad_x,
        pad_y,
        resized_width,
        resized_height)));
    const auto letterbox_t1 = SteadyClock::now();
    bpu_preprocess_timing_.letterbox_ms = elapsedMs(letterbox_t0, letterbox_t1);

    bpu_letterbox_.scale = scale;
    bpu_letterbox_.pad_x = pad_x;
    bpu_letterbox_.pad_y = pad_y;

    const auto color_convert_t0 = SteadyClock::now();
    cv::Mat yuv_i420;
    cv::cvtColor(letterbox_bgr, yuv_i420, cv::COLOR_BGR2YUV_I420);

    if (!yuv_i420.isContinuous()) {
      yuv_i420 = yuv_i420.clone();
    }
    const auto color_convert_t1 = SteadyClock::now();
    bpu_preprocess_timing_.color_convert_ms =
        elapsedMs(color_convert_t0, color_convert_t1);

    const auto nv12_pack_t0 = SteadyClock::now();
    bpu_input_nv12_.resize(kBpuInputNv12Size);

    const uint8_t *y_plane = yuv_i420.ptr<uint8_t>(0);
    const uint8_t *u_plane = y_plane + kBpuInputYSize;
    const uint8_t *v_plane = u_plane + kBpuInputUvSize;
    uint8_t *nv12_data = bpu_input_nv12_.data();

    std::memcpy(nv12_data, y_plane, kBpuInputYSize);

    uint8_t *uv_plane = nv12_data + kBpuInputYSize;

    for (std::size_t i = 0; i < kBpuInputUvSize; ++i) {
      uv_plane[i * 2] = u_plane[i];
      uv_plane[i * 2 + 1] = v_plane[i];
    }
    const auto nv12_pack_t1 = SteadyClock::now();
    bpu_preprocess_timing_.nv12_pack_ms = elapsedMs(nv12_pack_t0, nv12_pack_t1);
  } catch (const cv::Exception &e) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "Failed to prepare BPU input: %s",
        e.what());

    return false;
  }

  bpu_preprocess_timing_.total_ms = elapsedMs(total_t0, SteadyClock::now());
  return true;
}

void QrVisionNode::updateVisualCodeStability(
    const std::vector<DecodedVisualCode> &decoded_codes)
{
  updateVisualCodeCategoryStability(decoded_codes, "sku", sku_code_state_, "qr");
  updateVisualCodeCategoryStability(decoded_codes, "pkg", pkg_code_state_, "qr");
}

std::string QrVisionNode::composeCaptureBarcode(bool require_complete) const
{
  const std::string pkg_code =
      isValidCapturePrefixedCode(pkg_code_state_.stable_code, "PKG-")
          ? pkg_code_state_.stable_code
          : "";
  const std::string sku_code =
      isValidCapturePrefixedCode(sku_code_state_.stable_code, "SKU-")
          ? sku_code_state_.stable_code
          : "";
  const std::string shelf_code =
      isValidShelfCaptureCode(shelf_code_state_.stable_code)
          ? shelf_code_state_.stable_code
          : "";

  const bool complete_capture =
      !pkg_code.empty() && !sku_code.empty() && !shelf_code.empty();

  if (require_complete && !complete_capture) {
    return {};
  }

  if (!require_complete &&
      pkg_code.empty() && sku_code.empty() && shelf_code.empty()) {
    return {};
  }

  return pkg_code + "|" + sku_code + "|" + shelf_code;
}

std::string QrVisionNode::composeCaptureBarcodeWithNan(bool allow_all_nan) const
{
  const bool has_qr_group =
      isValidCapturePrefixedCode(pkg_code_state_.stable_code, "PKG-") &&
      isValidCapturePrefixedCode(sku_code_state_.stable_code, "SKU-");
  const bool has_shelf_code =
      isValidShelfCaptureCode(shelf_code_state_.stable_code);

  if (!allow_all_nan && !has_qr_group && !has_shelf_code) {
    return {};
  }

  const std::string pkg_code =
      has_qr_group ? pkg_code_state_.stable_code : kMissingCaptureField;
  const std::string sku_code =
      has_qr_group ? sku_code_state_.stable_code : kMissingCaptureField;
  const std::string shelf_code =
      has_shelf_code ? shelf_code_state_.stable_code : kMissingCaptureField;

  return pkg_code + "|" + sku_code + "|" + shelf_code;
}

void QrVisionNode::handleHoverActive(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(vision_worker_mutex_);
    if (!vision_worker_running_) {
      return;
    }

    pending_hover_events_.push_back({next_work_sequence_++, msg->data});
  }

  vision_worker_cv_.notify_one();
}

void QrVisionNode::applyHoverActive(bool next_hover_active)
{

  if (next_hover_active == hover_active_) {
    return;
  }

  if (hover_active_ && !next_hover_active && !full_capture_sent_in_hover_) {
#if DRONE_PERCEPTION_HAS_BPU
    if (!publishBestBufferedCapture()) {
      publishTextOnlyCapture(composeCaptureBarcodeWithNan(false));
    }
#else
    publishTextOnlyCapture(composeCaptureBarcodeWithNan(false));
#endif
  }

  hover_active_ = next_hover_active;

  if (hover_active_) {
    resetHoverCaptureState();
  } else {
#if DRONE_PERCEPTION_HAS_BPU
    resetCaptureCandidateState();
#endif
  }
}

void QrVisionNode::resetHoverCaptureState()
{
  full_capture_sent_in_hover_ = false;
  last_published_package_barcode_.clear();
  shelf_code_state_ = {};
  sku_code_state_ = {};
  pkg_code_state_ = {};
#if DRONE_PERCEPTION_HAS_BPU
  resetCaptureCandidateState();
  last_ocr_regions_.clear();
  debug_raw_symbol_.clear();
  debug_raw_symbol_type_.clear();
  debug_qr_preprocess_preview_.release();
  debug_qr_preprocess_mode_.clear();
  hover_capture_start_time_ = this->now();
#endif
}

void QrVisionNode::publishBarcodeCapture(const cv::Mat &color_image)
{
  if (!hover_active_) {
    return;
  }

  const std::string barcode = composeCaptureBarcode(true);

  if (barcode.empty() || barcode == last_published_package_barcode_) {
    return;
  }

  (void)publishFullFrameCapture(barcode, color_image);
}

void QrVisionNode::publishTextOnlyCapture(const std::string &barcode)
{
  if (!barcode_capture_pub_ || barcode.empty()) {
    return;
  }

  drone_msgs::msg::BarcodeCapture msg;
  msg.stamp = this->now();
  msg.barcode = barcode;
  barcode_capture_pub_->publish(msg);

  RCLCPP_INFO(
      get_logger(),
      "published partial barcode capture barcode=%s",
      barcode.c_str());
}

bool QrVisionNode::publishFullFrameCapture(
    const std::string &barcode,
    const cv::Mat &color_image)
{
  if (!barcode_capture_pub_ || barcode.empty()) {
    return false;
  }

#if DRONE_PERCEPTION_HAS_BPU
  std::vector<uint8_t> jpeg_data;

  if (!encodeFrameCaptureJpeg(color_image, jpeg_data)) {
    return false;
  }

  drone_msgs::msg::BarcodeCapture msg;
  msg.stamp = this->now();
  msg.barcode = barcode;
  msg.image_format = kJpegImageFormat;
  msg.image_data = std::move(jpeg_data);

  barcode_capture_pub_->publish(msg);
  last_published_package_barcode_ = barcode;
  full_capture_sent_in_hover_ = true;

  RCLCPP_INFO(
      get_logger(),
      "published frame barcode capture barcode=%s bytes=%zu",
      barcode.c_str(),
      msg.image_data.size());
  return true;
#else
  (void)color_image;
  RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "full frame barcode capture requested but this build has no BPU support");
  return false;
#endif
}

void QrVisionNode::updateVisualCodeCategoryStability(
    const std::vector<DecodedVisualCode> &decoded_codes,
    const std::string &category,
    CodeStabilityState &state,
    const char *source_name)
{
  const auto selected_code = std::find_if(
      decoded_codes.begin(),
      decoded_codes.end(),
      [&category](const DecodedVisualCode &decoded_code) {
        return decoded_code.category == category;
      });

  if (selected_code == decoded_codes.end()) {
    ++state.lost_count;

    if (state.lost_count > shelf_code_lost_tolerance_frames_) {
      state.candidate_code.clear();
      state.candidate_symbol_type.clear();
      state.stable_code.clear();
      state.stable_symbol_type.clear();
      state.candidate_count = 0;
    }

    return;
  }

  state.lost_count = 0;

  if (selected_code->code == state.candidate_code) {
    ++state.candidate_count;
  } else {
    state.candidate_code = selected_code->code;
    state.candidate_symbol_type = selected_code->symbol_type;
    state.candidate_count = 1;
  }

  if (state.candidate_count < shelf_code_stable_frames_) {
    return;
  }

  state.stable_code = state.candidate_code;
  state.stable_symbol_type = state.candidate_symbol_type;

  RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "stable %s code category=%s symbol_type=%s code=%s stable_frames=%d",
      source_name,
      category.c_str(),
      state.stable_symbol_type.c_str(),
      state.stable_code.c_str(),
      state.candidate_count);
}

#if DRONE_PERCEPTION_HAS_BPU
std::vector<QrVisionNode::DecodedVisualCode> QrVisionNode::decodeVisualCodesFromDetections(
    const cv::Mat &source_image,
    const std::vector<BpuYoloDetection> &detections)
{
  visual_code_timing_ = {};
  std::vector<DecodedVisualCode> decoded_codes;
  debug_qr_preprocess_preview_.release();
  debug_qr_preprocess_mode_.clear();

  if (source_image.empty() || detections.empty() || !visual_code_scanner_) {
    return decoded_codes;
  }

  zbar::ImageScanner &scanner = *visual_code_scanner_;

  const cv::Point2f image_center(
      static_cast<float>(source_image.cols) * 0.5F,
      static_cast<float>(source_image.rows) * 0.5F);

  for (const BpuYoloDetection &detection : detections) {
    if (detection.class_id != kYoloClassQrcode) {
      continue;
    }

    const BpuImageRect image_rect = mapBpuDetectionToImageRect(
        detection,
        source_image.cols,
        source_image.rows);
    const float x_min = image_rect.x_min;
    const float y_min = image_rect.y_min;
    const float x_max = image_rect.x_max;
    const float y_max = image_rect.y_max;
    const float width = x_max - x_min;
    const float height = y_max - y_min;

    if (width <= 1.0F || height <= 1.0F) {
      continue;
    }

    const float pad_x = width * kVisualCodeRoiPaddingXRatio;
    const float pad_y = height * kVisualCodeRoiPaddingYRatio;
    const int roi_x_min = std::max(0, static_cast<int>(x_min - pad_x));
    const int roi_y_min = std::max(0, static_cast<int>(y_min - pad_y));
    const int roi_x_max = std::min(source_image.cols, static_cast<int>(x_max + pad_x));
    const int roi_y_max = std::min(source_image.rows, static_cast<int>(y_max + pad_y));
    const cv::Rect roi(
        roi_x_min,
        roi_y_min,
        roi_x_max - roi_x_min,
        roi_y_max - roi_y_min);

    if (roi.width <= 1 || roi.height <= 1) {
      continue;
    }

    ++visual_code_timing_.roi_count;
    const auto gray_t0 = SteadyClock::now();
    const cv::Mat roi_mat = source_image(roi);
    cv::Mat raw_gray_roi;

    if (roi_mat.channels() == 3) {
      cv::cvtColor(roi_mat, raw_gray_roi, cv::COLOR_BGR2GRAY);
    } else {
      // source is already single-channel luma (direct hover path)
      raw_gray_roi = roi_mat;
    }

    if (!raw_gray_roi.isContinuous()) {
      raw_gray_roi = raw_gray_roi.clone();
    }
    const auto gray_t1 = SteadyClock::now();
    visual_code_timing_.gray_ms += elapsedMs(gray_t0, gray_t1);

    auto scan_gray_roi = [&](
        const cv::Mat &scan_roi,
        float coordinate_scale,
        const char *scan_mode) {
      cv::Mat continuous_roi = scan_roi;

      if (!continuous_roi.isContinuous()) {
        continuous_roi = continuous_roi.clone();
      }

      if (debug_view_) {
        debug_qr_preprocess_preview_ = continuous_roi.clone();
        debug_qr_preprocess_mode_ = scan_mode;
      }

      zbar::Image zbar_image(
          static_cast<unsigned int>(continuous_roi.cols),
          static_cast<unsigned int>(continuous_roi.rows),
          "Y800",
          continuous_roi.data,
          static_cast<unsigned long>(continuous_roi.total()));

      const auto scan_t0 = SteadyClock::now();
      const int raw_count = scanner.scan(zbar_image);
      const auto scan_t1 = SteadyClock::now();
      const double scan_ms = elapsedMs(scan_t0, scan_t1);

      if (std::strcmp(scan_mode, "raw_gray") == 0) {
        visual_code_timing_.raw_zbar_ms += scan_ms;
        ++visual_code_timing_.raw_scan_count;
      } else if (std::strcmp(scan_mode, "clahe_gray") == 0) {
        visual_code_timing_.clahe_zbar_ms += scan_ms;
        ++visual_code_timing_.clahe_scan_count;
      } else if (std::strcmp(scan_mode, "adaptive_binary") == 0) {
        visual_code_timing_.adaptive_zbar_ms += scan_ms;
        ++visual_code_timing_.adaptive_scan_count;
      }
      VisualCodeScanStats stats{};
      stats.symbol_count = raw_count;

      if (raw_count <= 0) {
        RCLCPP_DEBUG_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "zbar no code scan=%s roi=%dx%d at=%d,%d detection_score=%.3f",
            scan_mode,
            continuous_roi.cols,
            continuous_roi.rows,
            roi.x,
            roi.y,
            detection.score);
      }

      for (auto symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol) {
        const std::string raw_code = symbol->get_data();
        const std::string code = trimAndUppercase(raw_code);
        const std::string symbol_type = symbol->get_type_name();
        const bool is_qr_code = symbol_type.find("QR") != std::string::npos;
        const std::vector<ParsedVisualCode> parsed_codes = parseVisualCodes(code);
        const std::string categories = formatParsedVisualCodeCategories(parsed_codes);

        // Capture raw zbar output for debug display (before mode/parse filter)
        debug_raw_symbol_ = code;
        debug_raw_symbol_type_ = symbol_type;

        RCLCPP_DEBUG_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "zbar raw qr scan=%s symbol_type=%s qr_match=%s raw=%s normalized=%s category=%s roi=%dx%d at=%d,%d",
            scan_mode,
            symbol_type.c_str(),
            is_qr_code ? "true" : "false",
            raw_code.c_str(),
            code.c_str(),
            categories.c_str(),
            continuous_roi.cols,
            continuous_roi.rows,
            roi.x,
            roi.y);

        if (!is_qr_code || parsed_codes.empty()) {
          continue;
        }

        cv::Point2f code_center(
            static_cast<float>(roi.x) + static_cast<float>(roi.width) * 0.5F,
            static_cast<float>(roi.y) + static_cast<float>(roi.height) * 0.5F);

        const int location_size = symbol->get_location_size();

        if (location_size > 0) {
          code_center = cv::Point2f(0.0F, 0.0F);

          for (int i = 0; i < location_size; ++i) {
            code_center.x += static_cast<float>(roi.x) +
                static_cast<float>(symbol->get_location_x(i)) / coordinate_scale;
            code_center.y += static_cast<float>(roi.y) +
                static_cast<float>(symbol->get_location_y(i)) / coordinate_scale;
          }

          code_center.x /= static_cast<float>(location_size);
          code_center.y /= static_cast<float>(location_size);
        }

        const float dx = code_center.x - image_center.x;
        const float dy = code_center.y - image_center.y;

        for (const ParsedVisualCode &parsed_code : parsed_codes) {
          decoded_codes.push_back({
              parsed_code.code,
              parsed_code.category,
              symbol_type,
              static_cast<double>(dx * dx + dy * dy)});
          ++stats.accepted_count;
        }
      }

      zbar_image.set_data(nullptr, 0U);

      return stats;
    };

    VisualCodeScanStats scan_stats = scan_gray_roi(raw_gray_roi, 1.0F, "raw_gray");

    if (scan_stats.accepted_count > 0 || !qr_preprocess_enabled_) {
      continue;
    }

    try {
      const auto clahe_t0 = SteadyClock::now();
      cv::Mat clahe_roi;
      if (!qr_clahe_) {
        qr_clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));
      }
      qr_clahe_->apply(raw_gray_roi, clahe_roi);
      const auto clahe_t1 = SteadyClock::now();
      visual_code_timing_.clahe_ms += elapsedMs(clahe_t0, clahe_t1);

      scan_stats = scan_gray_roi(clahe_roi, 1.0F, "clahe_gray");

      if (scan_stats.accepted_count > 0) {
        continue;
      }

      const int min_side_px = std::min(clahe_roi.cols, clahe_roi.rows);
      int adaptive_block_size_px = std::min(kQrAdaptiveThresholdBlockSizePx, min_side_px);

      if (adaptive_block_size_px % 2 == 0) {
        --adaptive_block_size_px;
      }

      if (adaptive_block_size_px >= 3) {
        const auto adaptive_t0 = SteadyClock::now();
        cv::Mat adaptive_binary_roi;
        cv::adaptiveThreshold(
            clahe_roi,
            adaptive_binary_roi,
            255.0,
            cv::ADAPTIVE_THRESH_GAUSSIAN_C,
            cv::THRESH_BINARY,
            adaptive_block_size_px,
            kQrAdaptiveThresholdOffset);
        const auto adaptive_t1 = SteadyClock::now();
        visual_code_timing_.adaptive_ms += elapsedMs(adaptive_t0, adaptive_t1);
        (void)scan_gray_roi(adaptive_binary_roi, 1.0F, "adaptive_binary");
      }
    } catch (const cv::Exception &e) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          log_throttle_ms_,
          "QR preprocess failed: %s",
          e.what());
    }
  }

  std::sort(
      decoded_codes.begin(),
      decoded_codes.end(),
      [](const DecodedVisualCode &a, const DecodedVisualCode &b) {
        return a.center_distance_sq < b.center_distance_sq;
      });

  return decoded_codes;
}

bool QrVisionNode::encodeFrameCaptureJpeg(
    const cv::Mat &color_image,
    std::vector<uint8_t> &jpeg_data)
{
  jpeg_data.clear();

  if (color_image.empty()) {
    return false;
  }

  try {
    const std::vector<int> encode_params = {
        cv::IMWRITE_JPEG_QUALITY,
        barcode_capture_jpeg_quality_};

    return cv::imencode(".jpg", color_image, jpeg_data, encode_params) &&
        !jpeg_data.empty();
  } catch (const cv::Exception &e) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "frame capture jpeg encode failed: %s",
        e.what());
  }

  return false;
}

void QrVisionNode::resetCaptureCandidateState()
{
  best_package_capture_candidate_ = {};
  package_capture_candidate_count_ = 0;
  hover_capture_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

bool QrVisionNode::selectBestPackageInDeadzone(
    int image_width,
    int image_height,
    CaptureFrameCandidate &candidate) const
{
  candidate = {};

  if (image_width <= 0 || image_height <= 0 || last_bpu_detections_.empty()) {
    return false;
  }

  const double image_center_x = static_cast<double>(image_width) * 0.5;
  const double image_center_y = static_cast<double>(image_height) * 0.5;
  const double deadzone_radius =
      static_cast<double>(std::min(image_width, image_height)) *
      package_capture_deadzone_ratio_;
  const double deadzone_radius_sq = deadzone_radius * deadzone_radius;

  for (const BpuYoloDetection &detection : last_bpu_detections_) {
    if (detection.class_id != kYoloClassPackage ||
        static_cast<double>(detection.score) < package_capture_min_score_) {
      continue;
    }

    const BpuImageRect image_rect = mapBpuDetectionToImageRect(
        detection,
        image_width,
        image_height);

    if (image_rect.x_max - image_rect.x_min <= 1.0F ||
        image_rect.y_max - image_rect.y_min <= 1.0F) {
      continue;
    }

    const double package_center_x =
        (static_cast<double>(image_rect.x_min) + static_cast<double>(image_rect.x_max)) * 0.5;
    const double package_center_y =
        (static_cast<double>(image_rect.y_min) + static_cast<double>(image_rect.y_max)) * 0.5;
    const double dx = package_center_x - image_center_x;
    const double dy = package_center_y - image_center_y;
    const double center_distance_sq = dx * dx + dy * dy;

    if (center_distance_sq > deadzone_radius_sq) {
      continue;
    }

    const bool replace_candidate =
        !candidate.valid ||
        detection.score > candidate.package_score ||
        (std::abs(detection.score - candidate.package_score) <= 1.0e-6F &&
         center_distance_sq < candidate.center_distance_sq);

    if (!replace_candidate) {
      continue;
    }

    candidate.package_score = detection.score;
    candidate.center_distance_sq = center_distance_sq;
    candidate.valid = true;
  }

  return candidate.valid;
}

void QrVisionNode::bufferPackageCaptureCandidate(
    const cv::Mat &color_image,
    const std::uint8_t *yuyv,
    int yuyv_stride,
    int yuyv_width,
    int yuyv_height)
{
  if (!hover_active_ || hover_capture_start_time_.nanoseconds() <= 0) {
    return;
  }

  const bool has_bgr = !color_image.empty();
  const int image_width = has_bgr ? color_image.cols : yuyv_width;
  const int image_height = has_bgr ? color_image.rows : yuyv_height;

  if (image_width <= 0 || image_height <= 0 ||
      (!has_bgr && yuyv == nullptr)) {
    return;
  }

  const rclcpp::Time now_time = this->now();
  const double elapsed_s = (now_time - hover_capture_start_time_).seconds();

  if (elapsed_s < 0.0 || elapsed_s > capture_collection_duration_s_) {
    return;
  }

  CaptureFrameCandidate candidate;

  try {
    if (!selectBestPackageInDeadzone(image_width, image_height, candidate)) {
      return;
    }

    // Store the payload only for the selected frame, avoiding a per-frame full
    // BGR clone in the direct hover path.
    if (has_bgr) {
      candidate.image = color_image.clone();
    } else {
      candidate.yuyv.assign(
          yuyv,
          yuyv + static_cast<std::size_t>(yuyv_stride) * yuyv_height);
      candidate.yuyv_stride = yuyv_stride;
      candidate.yuyv_width = yuyv_width;
      candidate.yuyv_height = yuyv_height;
    }
  } catch (const cv::Exception &e) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "package capture candidate storage failed: %s",
        e.what());
    return;
  }

  candidate.stamp = now_time;
  ++package_capture_candidate_count_;

  const bool replace_best =
      !best_package_capture_candidate_.valid ||
      candidate.package_score > best_package_capture_candidate_.package_score ||
      (std::abs(candidate.package_score - best_package_capture_candidate_.package_score) <= 1.0e-6F &&
       candidate.center_distance_sq < best_package_capture_candidate_.center_distance_sq);

  if (!replace_best) {
    return;
  }

  best_package_capture_candidate_ = std::move(candidate);

  RCLCPP_DEBUG_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "best package capture candidate updated score=%.3f center_distance_px=%.1f "
      "elapsed_s=%.2f candidates=%d",
      best_package_capture_candidate_.package_score,
      std::sqrt(best_package_capture_candidate_.center_distance_sq),
      elapsed_s,
      package_capture_candidate_count_);
}

bool QrVisionNode::publishBestBufferedCapture()
{
  if (!best_package_capture_candidate_.valid) {
    RCLCPP_INFO(
        get_logger(),
        "no centered package capture candidate collected in first %.2fs",
        capture_collection_duration_s_);
    return false;
  }

  const std::string barcode = composeCaptureBarcodeWithNan(true);

  if (barcode == last_published_package_barcode_) {
    return false;
  }

  // The direct hover path stored YUYV; convert to BGR once at publish time
  // instead of building a full BGR on every hover frame.
  cv::Mat capture_bgr;

  if (!best_package_capture_candidate_.image.empty()) {
    capture_bgr = best_package_capture_candidate_.image;
  } else if (!best_package_capture_candidate_.yuyv.empty()) {
    cv::Mat yuyv_mat(
        best_package_capture_candidate_.yuyv_height,
        best_package_capture_candidate_.yuyv_width,
        CV_8UC2,
        const_cast<std::uint8_t *>(best_package_capture_candidate_.yuyv.data()),
        best_package_capture_candidate_.yuyv_stride);
    cv::cvtColor(yuyv_mat, capture_bgr, cv::COLOR_YUV2BGR_YUY2);
  }

  if (capture_bgr.empty()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "buffered package capture has no usable frame");
    return false;
  }

  const bool published = publishFullFrameCapture(barcode, capture_bgr);

  if (!published) {
    return false;
  }

  RCLCPP_INFO(
      get_logger(),
      "published best buffered package capture score=%.3f center_distance_px=%.1f "
      "candidates=%d",
      best_package_capture_candidate_.package_score,
      std::sqrt(best_package_capture_candidate_.center_distance_sq),
      package_capture_candidate_count_);

  return true;
}
#endif

void QrVisionNode::initializeSubscriptions()
{
  hover_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      hover_active_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(),
      std::bind(
          &QrVisionNode::handleHoverActive,
          this,
          std::placeholders::_1));

  if (camera_input_mode_ == "d435_direct") {
    RCLCPP_INFO(
        get_logger(),
        "Direct capture mode: no ROS color/camera_info subscription");
    return;
  }

  color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      color_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
          &QrVisionNode::handleColorFrame,
          this,
          std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
          &QrVisionNode::handleCameraInfo,
          this,
          std::placeholders::_1));

  RCLCPP_INFO(
      get_logger(),
      "Using color-only image input. color=%s camera_info=%s",
      color_topic_.c_str(),
      camera_info_topic_.c_str());
}

void QrVisionNode::handleCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg)
{
  (void)camera_info_msg;
  has_camera_info_ = true;
}

void QrVisionNode::updateFrameAge(const sensor_msgs::msg::Image &color_msg)
{
  last_frame_age_ms_ = -1.0;
  if (color_msg.header.stamp.sec == 0 && color_msg.header.stamp.nanosec == 0) {
    return;
  }

  try {
    const rclcpp::Time stamp(
        color_msg.header.stamp,
        get_clock()->get_clock_type());
    const double frame_age_ms = (this->now() - stamp).seconds() * 1000.0;
    if (frame_age_ms >= 0.0) {
      last_frame_age_ms_ = frame_age_ms;
      frame_age_window_.push(frame_age_ms);
    }
  } catch (const std::exception &) {
    last_frame_age_ms_ = -1.0;
  }
}

void QrVisionNode::updateFps()
{
  const rclcpp::Time current_frame_time = this->now();
  if (last_frame_time_.nanoseconds() > 0)
  {
    const double frame_interval_s = (current_frame_time - last_frame_time_).seconds();
    if (frame_interval_s > 0.0)
    {
      const double current_fps = 1.0 / frame_interval_s;
      smoothed_fps_ = smoothed_fps_ <= 0.0 ? current_fps : smoothed_fps_ * 0.9 + current_fps * 0.1;
    }
  }
  last_frame_time_ = current_frame_time;
}

void QrVisionNode::reportPerformanceBaseline()
{
#if DRONE_PERCEPTION_HAS_BPU
  if (!enable_bpu_) {
    return;
  }

  const auto now = SteadyClock::now();
  const std::uint64_t input_count = input_frame_count_.load();
  const std::uint64_t dropped_count = dropped_frame_count_.load();
  if (baseline_report_time_.time_since_epoch().count() == 0) {
    baseline_report_time_ = now;
    baseline_last_input_count_ = input_count;
    baseline_last_dropped_count_ = dropped_count;
    baseline_last_processed_count_ = processed_frame_count_;
    baseline_last_failed_count_ = failed_frame_count_;
    return;
  }

  const double interval_s =
      std::chrono::duration<double>(now - baseline_report_time_).count();
  if (interval_s < 1.0) {
    return;
  }

  const std::uint64_t input_delta = input_count - baseline_last_input_count_;
  const std::uint64_t dropped_delta = dropped_count - baseline_last_dropped_count_;
  const std::uint64_t process_delta =
      processed_frame_count_ - baseline_last_processed_count_;
  const std::uint64_t failed_delta = failed_frame_count_ - baseline_last_failed_count_;
  last_input_fps_ = static_cast<double>(input_delta) / interval_s;
  last_process_fps_ = static_cast<double>(process_delta) / interval_s;

  BpuYoloDetector::InferenceTimingStats infer_timing{};
  if (bpu_detector_) {
    infer_timing = bpu_detector_->lastTiming();
  }

  OcrTimingStats ocr_timing{};
  if (ocr_invoked_this_frame_ && bpu_ocr_pipeline_) {
    ocr_timing = bpu_ocr_pipeline_->lastTiming();
  }

  double callback_p50_ms = 0.0;
  double callback_p95_ms = 0.0;
  double callback_last_ms = 0.0;
  {
    std::lock_guard<std::mutex> lock(performance_mutex_);
    callback_p50_ms = callback_window_.percentile(0.50);
    callback_p95_ms = callback_window_.percentile(0.95);
    callback_last_ms = last_callback_ms_;
  }

  RCLCPP_INFO(
      get_logger(),
      "BPU_BASELINE input_fps=%.2f process_fps=%.2f failed=%llu "
      "drop=%llu(latest_frame_queue) frame_age_ms[p50=%.2f p95=%.2f last=%.2f] "
      "callback_ms[p50=%.2f p95=%.2f last=%.2f] "
      "process_ms[p50=%.2f p95=%.2f last=%.2f] "
      "queue_wait_ms[p50=%.2f p95=%.2f last=%.2f] "
      "pre_ms[resize=%.2f letterbox=%.2f color=%.2f nv12=%.2f total=%.2f] "
      "bpu_ms[memcpy=%.2f clean=%.2f submit=%.2f wait=%.2f release=%.2f "
      "invalidate=%.2f candidate=%.2f dfl=%.2f nms=%.2f total=%.2f] "
      "det_count[raw=%zu decoded=%zu final=%zu] "
      "qr[count=%d/%d/%d roi=%d gray=%.2f raw=%.2f clahe=%.2f+%.2f "
      "adaptive=%.2f+%.2f] "
      "ocr[invoked=%d boxes=%d crop=%.2f pre=%.2f infer=%.2f decode=%.2f total=%.2f]",
      last_input_fps_,
      last_process_fps_,
      static_cast<unsigned long long>(failed_delta),
      static_cast<unsigned long long>(dropped_delta),
      frame_age_window_.percentile(0.50),
      frame_age_window_.percentile(0.95),
      last_frame_age_ms_,
      callback_p50_ms,
      callback_p95_ms,
      callback_last_ms,
      process_window_.percentile(0.50),
      process_window_.percentile(0.95),
      last_process_ms_,
      queue_wait_window_.percentile(0.50),
      queue_wait_window_.percentile(0.95),
      last_queue_wait_ms_,
      bpu_preprocess_timing_.resize_ms,
      bpu_preprocess_timing_.letterbox_ms,
      bpu_preprocess_timing_.color_convert_ms,
      bpu_preprocess_timing_.nv12_pack_ms,
      bpu_preprocess_timing_.total_ms,
      infer_timing.input_memcpy_ms,
      infer_timing.input_cache_clean_ms,
      infer_timing.submit_ms,
      infer_timing.wait_ms,
      infer_timing.release_ms,
      infer_timing.output_invalidate_ms,
      infer_timing.candidate_ms,
      infer_timing.dfl_decode_ms,
      infer_timing.nms_ms,
      infer_timing.total_ms,
      infer_timing.raw_candidate_count,
      infer_timing.decoded_detection_count,
      infer_timing.final_detection_count,
      visual_code_timing_.raw_scan_count,
      visual_code_timing_.clahe_scan_count,
      visual_code_timing_.adaptive_scan_count,
      visual_code_timing_.roi_count,
      visual_code_timing_.gray_ms,
      visual_code_timing_.raw_zbar_ms,
      visual_code_timing_.clahe_ms,
      visual_code_timing_.clahe_zbar_ms,
      visual_code_timing_.adaptive_ms,
      visual_code_timing_.adaptive_zbar_ms,
      ocr_invoked_this_frame_ ? 1 : 0,
      ocr_timing.rec_box_count,
      ocr_timing.rec_crop_ms,
      ocr_timing.rec_preprocess_ms,
      ocr_timing.rec_infer_ms,
      ocr_timing.rec_decode_ms,
      ocr_timing.rec_total_ms);

  baseline_report_time_ = now;
  baseline_last_input_count_ = input_count;
  baseline_last_dropped_count_ = dropped_count;
  baseline_last_processed_count_ = processed_frame_count_;
  baseline_last_failed_count_ = failed_frame_count_;
#endif
}

void QrVisionNode::handleColorFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr &color_msg)
{
  const auto callback_t0 = SteadyClock::now();
  if (!color_msg) {
    return;
  }

  input_frame_count_.fetch_add(1U);
  {
    std::lock_guard<std::mutex> lock(vision_worker_mutex_);
    if (!vision_worker_running_) {
      return;
    }

    if (latest_color_frame_.message) {
      dropped_frame_count_.fetch_add(1U);
    }

    latest_color_frame_ = {
        next_work_sequence_++,
        color_msg,
        SteadyClock::now()};
  }

  vision_worker_cv_.notify_one();

  const double callback_ms = elapsedMs(callback_t0, SteadyClock::now());
  {
    std::lock_guard<std::mutex> lock(performance_mutex_);
    last_callback_ms_ = callback_ms;
    callback_window_.push(callback_ms);
  }
}

#if DRONE_PERCEPTION_HAS_REALSENSE
void QrVisionNode::initializeDirectCapture()
{
  // Stage A only accepts the validated 640x480@30 YUYV profile.
  if (d435_width_ != 640 || d435_height_ != 480 || d435_fps_ != 30) {
    throw std::runtime_error(
        "camera_input_mode=d435_direct: this stage only supports "
        "d435_width=640, d435_height=480, d435_fps=30 (got " +
        std::to_string(d435_width_) + "x" + std::to_string(d435_height_) +
        "@" + std::to_string(d435_fps_) + ")");
  }

  std::string serial = d435_serial_;

  if (serial.empty()) {
    serial = drone_perception::D435ColorCapture::detectSingleDeviceSerial();

    if (serial.empty()) {
      throw std::runtime_error(
          "camera_input_mode=d435_direct: d435_serial is empty and the number "
          "of D435i devices is not exactly one; refuse to auto-select");
    }

    RCLCPP_INFO(
        get_logger(),
        "Auto-selected the only D435i device serial=%s",
        serial.c_str());
  }

  if (camera_controls_enabled_) {
    camera_controls_enabled_ = false;
    RCLCPP_WARN(
        get_logger(),
        "camera_controls_enabled is ignored in d435_direct mode "
        "(no realsense2_camera_node to drive)");
  }

  drone_perception::D435ColorCapture::Config config;
  config.serial = serial;
  config.width = d435_width_;
  config.height = d435_height_;
  config.fps = d435_fps_;
  config.wait_timeout_ms = d435_wait_timeout_ms_;
  config.reconnect_delay_ms = d435_reconnect_delay_ms_;

  d435_capture_ = std::make_unique<drone_perception::D435ColorCapture>(config);

  if (!d435_capture_->start()) {
    const std::string error = d435_capture_->lastError();
    d435_capture_.reset();
    throw std::runtime_error(
        "camera_input_mode=d435_direct: failed to start D435i capture: " +
        error);
  }

  RCLCPP_INFO(
      get_logger(),
      "D435i direct capture started. serial=%s format=YUYV width=%d height=%d "
      "fps=%d wait_timeout_ms=%d reconnect_delay_ms=%d direct_input_debug_bgr=%s",
      serial.c_str(),
      config.width,
      config.height,
      config.fps,
      config.wait_timeout_ms,
      config.reconnect_delay_ms,
      direct_input_debug_bgr_ ? "true" : "false");

#if DRONE_PERCEPTION_HAS_BPU
  if (enable_bpu_) {
#if DRONE_PERCEPTION_HAS_NANO2D
    // Nano2D (GC820) binds n2d_open()/n2d_blit() to the calling thread via
    // thread-local storage, so the actual initialize() must run on the vision
    // worker thread (runDirectVisionLoop), not here on the executor thread.
    nano2d_ = std::make_unique<drone_perception::Nano2DPreprocessor>();
#else
    throw std::runtime_error(
        "camera_input_mode=d435_direct with enable_bpu=true requires the "
        "Nano2D hardware preprocessor, which is not available in this build");
#endif
  }
#endif
}

void QrVisionNode::runDirectVisionLoop()
{
  // Poll period bounds hover-event latency when the camera is idle or
  // disconnected; while frames are flowing waitForLatest() returns as soon as
  // a new frame is copied, so the poll adds no frame latency.
  const std::chrono::milliseconds hover_poll_interval(50);
  std::chrono::steady_clock::time_point last_stats_log = SteadyClock::now();

#if DRONE_PERCEPTION_HAS_NANO2D
  // GC820 Nano2D binds n2d_open()/n2d_blit() to the calling thread, so the
  // preprocessor must be initialized here on the worker thread.
  if (enable_bpu_ && nano2d_ && !nano2d_initialized_) {
    if (nano2d_->initialize(
            d435_width_,
            d435_height_,
            kBpuInputWidthPx,
            kBpuInputHeightPx)) {
      nano2d_initialized_ = true;
      RCLCPP_INFO(
          get_logger(),
          "Nano2D preprocessor initialized (worker thread). source=%dx%d model=%dx%d pad_y=80",
          d435_width_,
          d435_height_,
          kBpuInputWidthPx,
          kBpuInputHeightPx);
    } else {
      RCLCPP_ERROR(
          get_logger(),
          "Nano2D preprocessor init failed: %s",
          nano2d_->lastError().c_str());
    }
  }
#endif

  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(vision_worker_mutex_);

      if (!vision_worker_running_) {
        break;
      }

      if (!pending_hover_events_.empty()) {
        PendingHoverEvent hover_event = pending_hover_events_.front();
        pending_hover_events_.pop_front();
        lock.unlock();
        applyHoverActive(hover_event.active);
        continue;
      }
    }

    drone_perception::D435ColorFrame frame;

    if (d435_capture_ &&
        d435_capture_->waitForLatest(frame, hover_poll_interval)) {
      processDirectFrame(frame);
      reportPerformanceBaseline();
    }

    // Surface capture-level statistics (plan 8.2) that the BPU baseline report
    // cannot carry, so drop/timeout/reconnect trends are visible in the log.
    const auto now = SteadyClock::now();

    if (d435_capture_ &&
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_stats_log).count() >= 5000) {
      last_stats_log = now;
      RCLCPP_INFO(
          get_logger(),
          "D435_CAPTURE captured=%llu dropped=%llu wait_timeout=%llu reconnect=%llu",
          static_cast<unsigned long long>(d435_capture_->capturedCount()),
          static_cast<unsigned long long>(d435_capture_->droppedCount()),
          static_cast<unsigned long long>(d435_capture_->timeoutCount()),
          static_cast<unsigned long long>(d435_capture_->reconnectCount()));
    }
  }

#if DRONE_PERCEPTION_HAS_NANO2D
  // Tear down Nano2D on the same thread that opened it. Idempotent: the member
  // destructor's later shutdown() sees cleared handles and does nothing.
  if (nano2d_) {
    nano2d_->shutdown();
  }
#endif
}

void QrVisionNode::processDirectFrame(
    const drone_perception::D435ColorFrame &frame)
{
  const auto process_t0 = SteadyClock::now();

  if (frame.yuyv.empty() || frame.width <= 0 || frame.height <= 0) {
    ++failed_frame_count_;
    return;
  }

  input_frame_count_.fetch_add(1U);

#if DRONE_PERCEPTION_HAS_REALSENSE && DRONE_PERCEPTION_HAS_RDK_MEDIA_CODEC
  // Same frame is also offered to the H.264 encoder worker. Non-blocking
  // latest-frame slot: a slow encoder drops old frames, never stalls detection.
  if (video_stream_enabled_) {
    submitVideoFrame(frame);
  }
#endif

  updateFps();

  last_queue_wait_ms_ = elapsedMs(frame.received_at, process_t0);
  queue_wait_window_.push(last_queue_wait_ms_);

  const double frame_age_ms = elapsedMs(frame.received_at, SteadyClock::now());
  if (frame_age_ms >= 0.0) {
    last_frame_age_ms_ = frame_age_ms;
    frame_age_window_.push(frame_age_ms);
  }

  try {
    // Stage C (per-ROI): build a full BGR only for the debug window. When
    // hovering without a debug window, build full luma (YUYV Y) for QR decoding
    // instead; OCR converts only its shelf ROI and the capture converts once at
    // publish time, so no full-frame YUYV->BGR (~20 ms on the RDK) is needed.
    const bool need_display = debug_view_;
    const bool need_vision = need_display || hover_active_;
    cv::Mat bgr;
    cv::Mat gray;

    if (need_display) {
      cv::Mat yuyv_mat(
          frame.height,
          frame.width,
          CV_8UC2,
          const_cast<std::uint8_t *>(frame.yuyv.data()),
          frame.stride_bytes);
      cv::cvtColor(yuyv_mat, bgr, cv::COLOR_YUV2BGR_YUY2);

      if (bgr.empty()) {
        ++failed_frame_count_;
        return;
      }
    } else if (hover_active_) {
      cv::Mat yuyv_mat(
          frame.height,
          frame.width,
          CV_8UC2,
          const_cast<std::uint8_t *>(frame.yuyv.data()),
          frame.stride_bytes);
      cv::cvtColor(yuyv_mat, gray, cv::COLOR_YUV2GRAY_YUY2);
    }

#if DRONE_PERCEPTION_HAS_BPU
    bool bpu_inference_succeeded = false;
    ocr_invoked_this_frame_ = false;
    visual_code_timing_ = {};
    last_bpu_detections_.clear();

    if (enable_bpu_ && bpu_detector_) {
#if DRONE_PERCEPTION_HAS_NANO2D
      if (nano2d_ && nano2d_initialized_) {
        const auto preprocess_t0 = SteadyClock::now();
        drone_perception::Nv12FrameView nv12_view;
        const bool input_ready = nano2d_->convertYuyvToLetterboxNv12(
            frame.yuyv.data(),
            frame.stride_bytes,
            nv12_view);
        const auto preprocess_t1 = SteadyClock::now();

        if (input_ready) {
          const drone_perception::Nano2DLetterboxState nano2d_letterbox =
              nano2d_->letterboxState();
          bpu_letterbox_.scale = nano2d_letterbox.scale;
          bpu_letterbox_.pad_x = nano2d_letterbox.pad_x;
          bpu_letterbox_.pad_y = nano2d_letterbox.pad_y;

          try {
            const auto infer_t0 = SteadyClock::now();
            last_bpu_detections_ =
                bpu_detector_->inferNv12(nv12_view.data, nv12_view.size);
            bpu_inference_succeeded = true;
            const auto infer_t1 = SteadyClock::now();

            int qrcode_count = 0;
            int package_count = 0;
            int shelf_tag_count = 0;

            for (const BpuYoloDetection &detection : last_bpu_detections_) {
              if (detection.class_id == kYoloClassQrcode) {
                ++qrcode_count;
              } else if (detection.class_id == kYoloClassPackage) {
                ++package_count;
              } else if (detection.class_id == kYoloClassShelfTag) {
                ++shelf_tag_count;
              }
            }

            RCLCPP_DEBUG_THROTTLE(
                get_logger(),
                *get_clock(),
                log_throttle_ms_,
                "BPU inference ok (nano2d). mode=d435_direct "
                "nano2d_preprocess_ms=%.2f infer_ms=%.2f input_size=%zu "
                "detections=%zu qrcode=%d package=%d shelf_tag=%d "
                "letterbox_scale=%.4f letterbox_pad=%d,%d",
                elapsedMs(preprocess_t0, preprocess_t1),
                elapsedMs(infer_t0, infer_t1),
                nv12_view.size,
                last_bpu_detections_.size(),
                qrcode_count,
                package_count,
                shelf_tag_count,
                bpu_letterbox_.scale,
                bpu_letterbox_.pad_x,
                bpu_letterbox_.pad_y);
            // Log the mapped box of the strongest detection so letterbox
            // reverse-transform correctness (pad_y=80, scale=1.0) can be
            // validated headlessly against the 640x480 image coordinates.
            if (!last_bpu_detections_.empty()) {
              const auto best = std::max_element(
                  last_bpu_detections_.begin(),
                  last_bpu_detections_.end(),
                  [](const BpuYoloDetection &a, const BpuYoloDetection &b) {
                    return a.score < b.score;
                  });
              const BpuImageRect image_rect = mapBpuDetectionToImageRect(
                  *best,
                  frame.width,
                  frame.height);
              RCLCPP_INFO_THROTTLE(
                  get_logger(),
                  *get_clock(),
                  log_throttle_ms_,
                  "NANO2D_BOX class=%s score=%.3f "
                  "model=[%.0f,%.0f,%.0f,%.0f] image=[%.0f,%.0f,%.0f,%.0f]",
                  BpuYoloDetector::className(best->class_id),
                  best->score,
                  best->x_min_px,
                  best->y_min_px,
                  best->x_max_px,
                  best->y_max_px,
                  image_rect.x_min,
                  image_rect.y_min,
                  image_rect.x_max,
                  image_rect.y_max);
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                log_throttle_ms_,
                "BPU inference failed: %s",
                e.what());
          }
        } else {
          RCLCPP_ERROR_THROTTLE(
              get_logger(),
              *get_clock(),
              log_throttle_ms_,
              "Nano2D conversion failed: %s",
              nano2d_->lastError().c_str());
        }
      } else
#endif
      {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "camera_input_mode=d435_direct requires the Nano2D preprocessor "
            "for the BPU path, but it is not available in this build");
      }
    }
#endif

    if (need_vision) {
      runFrameBusinessLogic(
          bgr,
          gray,
          frame.yuyv.data(),
          frame.stride_bytes,
          frame.width,
          frame.height,
          "d435_direct");
    }

    const auto process_t1 = SteadyClock::now();
    last_process_ms_ = elapsedMs(process_t0, process_t1);
    process_window_.push(last_process_ms_);
#if DRONE_PERCEPTION_HAS_BPU
    if (enable_bpu_) {
      if (bpu_inference_succeeded) {
        ++processed_frame_count_;
      } else {
        ++failed_frame_count_;
      }
    }
#endif
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "video stream mode=d435_direct fps=%.1f size=%dx%d camera_info=%s "
        "process_ms=%.2f queue_wait_ms=%.2f debug_view=%s",
        smoothed_fps_,
        frame.width,
        frame.height,
        has_camera_info_ ? "true" : "false",
        last_process_ms_,
        last_queue_wait_ms_,
        debug_view_ ? "true" : "false");
  } catch (const std::exception &ex) {
    ++failed_frame_count_;
    last_process_ms_ = elapsedMs(process_t0, SteadyClock::now());
    process_window_.push(last_process_ms_);
    RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "direct capture frame processing failed: %s",
        ex.what());
  }
}
#endif

void QrVisionNode::runFrameBusinessLogic(
    const cv::Mat &color_image,
    const cv::Mat &gray_image,
    const std::uint8_t *yuyv,
    int yuyv_stride,
    int yuyv_width,
    int yuyv_height,
    const char *input_mode)
{
#if DRONE_PERCEPTION_HAS_BPU
  if (hover_active_ && enable_bpu_ocr_ && bpu_ocr_pipeline_) {
    try {
      recognizeShelfTagFromDetections(
          color_image,
          yuyv,
          yuyv_stride,
          yuyv_width,
          yuyv_height,
          last_bpu_detections_,
          input_mode);
    } catch (const std::exception &e) {
      last_ocr_regions_.clear();
      updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");

      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          log_throttle_ms_,
          "BPU OCR failed: %s",
          e.what());
    }
  } else {
    last_ocr_regions_.clear();
    if (hover_active_) {
      updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
    }
  }
#endif

#if DRONE_PERCEPTION_HAS_BPU
  if (hover_active_) {
    // QR decode source: full BGR (debug/ROS) or full luma (direct hover path).
    const cv::Mat &decode_source = gray_image.empty() ? color_image : gray_image;
    updateVisualCodeStability(
        decodeVisualCodesFromDetections(decode_source, last_bpu_detections_));
  } else {
    debug_raw_symbol_.clear();
    debug_raw_symbol_type_.clear();
    debug_qr_preprocess_preview_.release();
    debug_qr_preprocess_mode_.clear();
  }
#else
  (void)gray_image;
  (void)input_mode;
  (void)yuyv;
  (void)yuyv_stride;
  (void)yuyv_width;
  (void)yuyv_height;
  updateVisualCodeStability({});
  updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
#endif

#if DRONE_PERCEPTION_HAS_BPU
  bufferPackageCaptureCandidate(
      color_image,
      yuyv,
      yuyv_stride,
      yuyv_width,
      yuyv_height);
#else
  publishBarcodeCapture(color_image);
#endif

  if (debug_view_ && !color_image.empty()) {
    displayDebugFrame(color_image);
  }
}

void QrVisionNode::processFrame(
    const cv_bridge::CvImageConstPtr &color_bridge,
    const std::chrono::steady_clock::time_point &process_t0,
    const char *input_mode)
{
  if (!color_bridge || color_bridge->image.empty())
  {
    return;
  }

  updateFps();

#if DRONE_PERCEPTION_HAS_BPU
  bool bpu_inference_succeeded = false;
  ocr_invoked_this_frame_ = false;
  visual_code_timing_ = {};
  last_bpu_detections_.clear();

  if (enable_bpu_ && bpu_detector_) {
    const auto preprocess_t0 = SteadyClock::now();
    const bool input_ready = prepareBpuInput(color_bridge->image);
    const auto preprocess_t1 = SteadyClock::now();

    if (input_ready) {
      try {
        const auto infer_t0 = SteadyClock::now();

        const auto detections = bpu_detector_->inferNv12(
            bpu_input_nv12_.data(),
            bpu_input_nv12_.size());
        last_bpu_detections_ = detections;
        bpu_inference_succeeded = true;

        const auto infer_t1 = SteadyClock::now();
        int qrcode_count = 0;
        int package_count = 0;
        int shelf_tag_count = 0;

        for (const BpuYoloDetection &detection : detections) {
          if (detection.class_id == kYoloClassQrcode) {
            ++qrcode_count;
          } else if (detection.class_id == kYoloClassPackage) {
            ++package_count;
          } else if (detection.class_id == kYoloClassShelfTag) {
            ++shelf_tag_count;
          }
        }

        RCLCPP_DEBUG_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "BPU inference ok. mode=%s preprocess_ms=%.2f infer_ms=%.2f "
            "input_size=%zu detections=%zu qrcode=%d package=%d shelf_tag=%d "
            "letterbox_scale=%.4f letterbox_pad=%d,%d",
            input_mode,
            elapsedMs(preprocess_t0, preprocess_t1),
            elapsedMs(infer_t0, infer_t1),
            bpu_input_nv12_.size(),
            detections.size(),
            qrcode_count,
            package_count,
            shelf_tag_count,
            bpu_letterbox_.scale,
            bpu_letterbox_.pad_x,
            bpu_letterbox_.pad_y);
      } catch (const std::exception &e) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            log_throttle_ms_,
            "BPU inference failed: %s",
            e.what());
      }
    }
  }
#endif

  runFrameBusinessLogic(
      color_bridge->image,
      cv::Mat{},
      nullptr,
      0,
      0,
      0,
      input_mode);

  const auto process_t1 = SteadyClock::now();
  last_process_ms_ = elapsedMs(process_t0, process_t1);
  process_window_.push(last_process_ms_);
#if DRONE_PERCEPTION_HAS_BPU
  if (enable_bpu_) {
    if (bpu_inference_succeeded) {
      ++processed_frame_count_;
    } else {
      ++failed_frame_count_;
    }
  }
#endif
  RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "video stream mode=%s fps=%.1f size=%dx%d camera_info=%s "
      "process_ms=%.2f queue_wait_ms=%.2f debug_view=%s",
      input_mode,
      smoothed_fps_,
      color_bridge->image.cols,
      color_bridge->image.rows,
      has_camera_info_ ? "true" : "false",
      last_process_ms_,
      last_queue_wait_ms_,
      debug_view_ ? "true" : "false");
}

void QrVisionNode::displayDebugFrame(const cv::Mat &color_image)
{
  // HighGUI must be driven from the main thread. This worker-thread function
  // only renders the frame into a shared slot; updateDebugDisplay() (wall timer
  // on the executor thread) calls imshow/waitKey.
  if (!debug_window_created_)
  {
    return;
  }

  cv::Mat display = color_image.clone();

  cv::Mat overlay = display.clone();
  const int panel_bottom = enable_bpu_ocr_ ? 236 : 164;
  const int panel_right = std::min(display.cols - 12, 620);
  cv::rectangle(
      overlay,
      cv::Point(12, 12),
      cv::Point(panel_right, panel_bottom),
      cv::Scalar(30, 30, 30),
      cv::FILLED);
  cv::addWeighted(overlay, 0.45, display, 0.55, 0.0, display);

  cv::putText(
      display,
      cv::format("FPS: %.1f", smoothed_fps_),
      cv::Point(24, 38),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(255, 0, 255),
      1);
  cv::putText(
      display,
      cv::format("Size: %dx%d", color_image.cols, color_image.rows),
      cv::Point(24, 60),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(255, 255, 255),
      1);
  auto fit_status_text = [](std::string text) {
    static constexpr std::size_t kMaxStatusTextLength = 58U;

    if (text.size() > kMaxStatusTextLength) {
      text = text.substr(0U, kMaxStatusTextLength - 3U) + "...";
    }

    return text;
  };

  auto draw_visual_code_line = [&](
      const CodeStabilityState &state,
      const char *category,
      int y) {
    if (state.stable_code.empty()) {
      return y;
    }

    const bool is_ocr_code = state.stable_symbol_type == "OCR";
    const char *kind = is_ocr_code ? "OCR" : "QR";
    const std::string line = fit_status_text(cv::format(
        "%s %s: %s",
        kind,
        category,
        state.stable_code.c_str()));

    cv::putText(
        display,
        line,
        cv::Point(24, y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        is_ocr_code ? cv::Scalar(255, 210, 120) : cv::Scalar(120, 220, 255),
        1);

    return y + 22;
  };

  int visual_code_y = 106;
  visual_code_y = draw_visual_code_line(shelf_code_state_, "shelf", visual_code_y);
  visual_code_y = draw_visual_code_line(sku_code_state_, "sku", visual_code_y);
  (void)draw_visual_code_line(pkg_code_state_, "pkg", visual_code_y);

#if DRONE_PERCEPTION_HAS_BPU
  // Show raw zbar output before parsing filter
  if (!debug_raw_symbol_.empty()) {
    const std::string raw_line = cv::format("Raw zbar: %s %s",
        debug_raw_symbol_type_.c_str(),
        fit_status_text(debug_raw_symbol_).c_str());
    cv::putText(
        display,
        raw_line,
        cv::Point(24, visual_code_y + 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(180, 180, 180),
        1);
  }

  drawBpuDetections(display);
  drawOcrRegions(display);

  if (enable_bpu_ocr_ && !last_ocr_regions_.empty()) {
    int ocr_text_y = visual_code_y + 48;

    for (const OcrTextRegion &region : last_ocr_regions_) {
      const std::string ocr_line = cv::format(
          "OCR: %s (%.2f)",
          fit_status_text(region.text).c_str(),
          region.score);
      cv::putText(
          display,
          ocr_line,
          cv::Point(24, ocr_text_y),
          cv::FONT_HERSHEY_SIMPLEX,
          0.5,
          cv::Scalar(255, 210, 120),
          1);
      ocr_text_y += 22;

      if (ocr_text_y > panel_bottom - 8) {
        break;
      }
    }
  }

  drawBpuPerformanceHud(display);
  drawQrPreprocessPreview(display);
#endif

  {
    std::lock_guard<std::mutex> lock(debug_frame_mutex_);
    latest_debug_frame_ = std::move(display);
    debug_frame_ready_ = true;
  }
}

void QrVisionNode::updateDebugDisplay()
{
  cv::Mat frame;

  {
    std::lock_guard<std::mutex> lock(debug_frame_mutex_);
    if (!debug_frame_ready_)
    {
      return;
    }
    frame = latest_debug_frame_;
    debug_frame_ready_ = false;
  }

  if (frame.empty())
  {
    return;
  }

  try
  {
    cv::imshow(window_name_, frame);
    cv::waitKey(1);
  }
  catch (const cv::Exception &e)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "debug window display failed: %s",
        e.what());
  }
}

#if DRONE_PERCEPTION_HAS_BPU
void QrVisionNode::drawBpuPerformanceHud(cv::Mat &display)
{
  if (!enable_bpu_ || display.empty()) {
    return;
  }

  BpuYoloDetector::InferenceTimingStats infer_timing{};
  if (bpu_detector_) {
    infer_timing = bpu_detector_->lastTiming();
  }

  OcrTimingStats ocr_timing{};
  if (ocr_invoked_this_frame_ && bpu_ocr_pipeline_) {
    ocr_timing = bpu_ocr_pipeline_->lastTiming();
  }

  double callback_ms = 0.0;
  {
    std::lock_guard<std::mutex> lock(performance_mutex_);
    callback_ms = last_callback_ms_;
  }

  const std::vector<std::string> lines{
      cv::format(
          "FPS input %.1f  process %.1f  smooth %.1f",
          last_input_fps_,
          last_process_fps_,
          smoothed_fps_),
      cv::format(
          "Latest cb %.2f  prev process %.2f  queue %.2f ms",
          callback_ms,
          last_process_ms_,
          last_queue_wait_ms_),
      cv::format(
          "Age %.1fms  drop %llu  failed %llu",
          last_frame_age_ms_,
          static_cast<unsigned long long>(dropped_frame_count_.load()),
          static_cast<unsigned long long>(failed_frame_count_)),
      cv::format(
          "Pre resize %.2f  box %.2f  color %.2f ms",
          bpu_preprocess_timing_.resize_ms,
          bpu_preprocess_timing_.letterbox_ms,
          bpu_preprocess_timing_.color_convert_ms),
      cv::format(
          "Pre NV12 %.2f  total %.2f ms",
          bpu_preprocess_timing_.nv12_pack_ms,
          bpu_preprocess_timing_.total_ms),
      cv::format(
          "BPU copy %.2f  clean %.2f  submit %.2f ms",
          infer_timing.input_memcpy_ms,
          infer_timing.input_cache_clean_ms,
          infer_timing.submit_ms),
      cv::format(
          "BPU wait %.2f  total %.2f ms",
          infer_timing.wait_ms,
          infer_timing.total_ms),
      cv::format(
          "Post inv %.2f  cand %.2f  DFL %.2f  NMS %.2f ms",
          infer_timing.output_invalidate_ms,
          infer_timing.candidate_ms,
          infer_timing.dfl_decode_ms,
          infer_timing.nms_ms),
      cv::format(
          "Det raw %zu  decoded %zu  final %zu",
          infer_timing.raw_candidate_count,
          infer_timing.decoded_detection_count,
          infer_timing.final_detection_count),
      cv::format(
          "ZBar roi %d  scans %d/%d/%d",
          visual_code_timing_.roi_count,
          visual_code_timing_.raw_scan_count,
          visual_code_timing_.clahe_scan_count,
          visual_code_timing_.adaptive_scan_count),
      cv::format(
          "ZBar ms gray %.1f raw %.1f CLAHE %.1f+%.1f adapt %.1f+%.1f",
          visual_code_timing_.gray_ms,
          visual_code_timing_.raw_zbar_ms,
          visual_code_timing_.clahe_ms,
          visual_code_timing_.clahe_zbar_ms,
          visual_code_timing_.adaptive_ms,
          visual_code_timing_.adaptive_zbar_ms),
      cv::format(
          "OCR run%d box%d crop%.2f pre%.2f inf%.2f dec%.2f ms",
          ocr_invoked_this_frame_ ? 1 : 0,
          ocr_timing.rec_box_count,
          ocr_timing.rec_crop_ms,
          ocr_timing.rec_preprocess_ms,
          ocr_timing.rec_infer_ms,
          ocr_timing.rec_decode_ms)};

  static constexpr int kLineHeightPx = 18;
  static constexpr int kPanelMarginPx = 12;
  static constexpr double kFontScale = 0.38;
  const int status_panel_bottom = enable_bpu_ocr_ ? 236 : 164;
  const int available_height =
      display.rows - status_panel_bottom - 2 * kPanelMarginPx;
  if (available_height < kLineHeightPx ||
      display.cols <= 2 * kPanelMarginPx) {
    return;
  }

  const std::size_t visible_line_count = std::min(
      lines.size(),
      static_cast<std::size_t>(available_height / kLineHeightPx));
  const int panel_height =
      static_cast<int>(visible_line_count) * kLineHeightPx + kPanelMarginPx;
  const int panel_top = std::max(
      status_panel_bottom + kPanelMarginPx,
      display.rows - panel_height - kPanelMarginPx);
  const bool reserve_qr_preview =
      !debug_qr_preprocess_preview_.empty() && display.cols >= 640;
  const int preview_reserve_width = reserve_qr_preview
      ? kQrPreprocessPreviewMaxSizePx + 32
      : 0;
  const int panel_right =
      display.cols - kPanelMarginPx - preview_reserve_width;
  if (panel_right <= kPanelMarginPx) {
    return;
  }

  const cv::Rect panel_rect(
      kPanelMarginPx,
      panel_top,
      panel_right - kPanelMarginPx,
      display.rows - kPanelMarginPx - panel_top);
  if (panel_rect.width <= 20) {
    return;
  }

  cv::Mat panel = display(panel_rect);
  cv::Mat panel_background(panel.size(), panel.type(), cv::Scalar(25, 25, 25));
  cv::addWeighted(panel_background, 0.50, panel, 0.50, 0.0, panel);

  int y = panel_top + kLineHeightPx;
  const cv::Scalar text_color(255, 0, 255);
  const int text_width_px = panel_rect.width - 16;
  for (std::size_t i = 0U; i < visible_line_count; ++i) {
    std::string line = lines[i];
    int baseline = 0;
    const bool needs_truncation = cv::getTextSize(
        line,
        cv::FONT_HERSHEY_SIMPLEX,
        kFontScale,
        1,
        &baseline).width > text_width_px;
    while (needs_truncation && !line.empty() &&
           cv::getTextSize(
               line + "...",
               cv::FONT_HERSHEY_SIMPLEX,
               kFontScale,
               1,
               &baseline).width > text_width_px) {
      line.pop_back();
    }
    if (needs_truncation) {
      line += "...";
    }

    cv::putText(
        display,
        line,
        cv::Point(kPanelMarginPx + 8, y),
        cv::FONT_HERSHEY_SIMPLEX,
        kFontScale,
        text_color,
        1,
        cv::LINE_AA);
    y += kLineHeightPx;
  }
}
#endif

void QrVisionNode::initializeCameraControlDefaults()
{
  camera_numeric_controls_[kCameraExposureControlIndex] = {
      "rgb_camera.exposure",
      "exposure",
      1,
      10000,
      999,
      1,
      10000,
      999,
      999,
      998,
      rclcpp::ParameterType::PARAMETER_INTEGER,
      true,
      false};
  camera_numeric_controls_[kCameraGainControlIndex] = {
      "rgb_camera.gain",
      "gain",
      0,
      128,
      64,
      0,
      128,
      64,
      64,
      64,
      rclcpp::ParameterType::PARAMETER_INTEGER,
      true,
      false};
  camera_numeric_controls_[kCameraWhiteBalanceControlIndex] = {
      "rgb_camera.white_balance",
      "white_balance",
      2800,
      6500,
      4600,
      2800,
      6500,
      4600,
      4600,
      1800,
      rclcpp::ParameterType::PARAMETER_DOUBLE,
      true,
      false};

  camera_bool_controls_[kCameraAutoExposureControlIndex] = {
      "rgb_camera.enable_auto_exposure",
      "enable_auto_exposure",
      false,
      false,
      true,
      false};
  camera_bool_controls_[kCameraAutoWhiteBalanceControlIndex] = {
      "rgb_camera.enable_auto_white_balance",
      "enable_auto_white_balance",
      true,
      true,
      true,
      false};
  camera_bool_controls_[kCameraAutoExposurePriorityControlIndex] = {
      "rgb_camera.auto_exposure_priority",
      "auto_exposure_priority",
      false,
      false,
      true,
      false};

  for (CameraNumericControl &control : camera_numeric_controls_) {
    control.restore_value = control.fallback_default;
    control.restore_value_initialized = true;
  }

  for (CameraBoolControl &control : camera_bool_controls_) {
    control.restore_value = control.current_value;
    control.restore_value_initialized = true;
  }

  for (std::size_t i = 0U; i < camera_trackbar_contexts_.size(); ++i) {
    camera_trackbar_contexts_[i].node = this;
    camera_trackbar_contexts_[i].index = i;
  }

  updateCameraControlEnableStates();
}

void QrVisionNode::initializeCameraControls()
{
  initializeCameraControlDefaults();

  if (!camera_controls_enabled_) {
    return;
  }

  if (!debug_view_) {
    camera_controls_enabled_ = false;
    RCLCPP_WARN(
        get_logger(),
        "camera_controls_enabled requires debug_view=true because it uses OpenCV HighGUI");
    return;
  }

  if (camera_param_node_.empty()) {
    camera_param_node_ = "/camera/camera";
  }

  try {
    camera_param_client_ =
        std::make_shared<rclcpp::AsyncParametersClient>(this, camera_param_node_);

    for (std::size_t i = 0U; i < camera_numeric_controls_.size(); ++i) {
      CameraNumericControl &control = camera_numeric_controls_[i];
      control.current_value = std::clamp(
          control.current_value,
          control.min_value,
          control.max_value);
      control.last_sent_value = control.current_value;
      control.trackbar_value = control.current_value - control.min_value;

      cv::createTrackbar(
          control.label,
          window_name_,
          nullptr,
          std::max(1, control.max_value - control.min_value),
          &QrVisionNode::handleCameraTrackbarCallback,
          &camera_trackbar_contexts_[i]);
      updateCameraTrackbarPosition(i);
    }

    cv::setMouseCallback(
        window_name_,
        &QrVisionNode::handleCameraControlsMouseCallback,
        this);

    camera_controls_attached_ = true;
    camera_controls_status_text_ = "camera param node offline";

    camera_controls_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(camera_controls_update_period_ms_),
        [this]() {
          updateCameraControlClient();
        });

    RCLCPP_INFO(
        get_logger(),
        "D435i RGB controls embedded in debug_view. window=%s camera_param_node=%s refresh_ms=%d",
        window_name_.c_str(),
        camera_param_node_.c_str(),
        camera_controls_update_period_ms_);
  } catch (const cv::Exception &e) {
    camera_controls_enabled_ = false;
    camera_controls_attached_ = false;

    RCLCPP_WARN(
        get_logger(),
        "Cannot attach D435i controls to debug window: %s",
        e.what());
  }
}

void QrVisionNode::updateCameraControlClient()
{
  if (!camera_controls_enabled_ || !camera_param_client_) {
    return;
  }

  const bool service_ready = camera_param_client_->service_is_ready();

  if (!service_ready) {
    camera_param_node_online_ = false;
    camera_controls_status_text_ = "camera param node offline";
    return;
  }

  if (!camera_param_node_online_) {
    camera_param_node_online_ = true;
    camera_param_describe_requested_ = false;
    camera_param_get_requested_ = false;
    camera_controls_status_text_ = "camera param node online";
  }

  if (!camera_param_describe_requested_ && !camera_param_describe_in_flight_) {
    requestCameraControlDescriptors();
  }

  if (!camera_param_get_requested_ && !camera_param_get_in_flight_) {
    requestCameraControlValues();
  }

  sendPendingCameraControlParameters();
}

void QrVisionNode::requestCameraControlDescriptors()
{
  if (!camera_param_client_) {
    return;
  }

  std::vector<std::string> parameter_names;
  parameter_names.reserve(camera_numeric_controls_.size() + camera_bool_controls_.size());

  for (const CameraNumericControl &control : camera_numeric_controls_) {
    parameter_names.push_back(control.parameter_name);
  }

  for (const CameraBoolControl &control : camera_bool_controls_) {
    parameter_names.push_back(control.parameter_name);
  }

  camera_param_describe_requested_ = true;
  camera_param_describe_in_flight_ = true;

  try {
    camera_param_client_->describe_parameters(
        parameter_names,
        [this](auto future) {
          camera_param_describe_in_flight_ = false;

          try {
            applyCameraControlDescriptors(future.get());
            camera_controls_status_text_ = "camera descriptors loaded";
          } catch (const std::exception &e) {
            camera_controls_status_text_ = "describe failed";
            RCLCPP_WARN(
                get_logger(),
                "Failed to describe D435i RGB parameters: %s",
                e.what());
          }
        });
  } catch (const std::exception &e) {
    camera_param_describe_in_flight_ = false;
    camera_controls_status_text_ = "describe request failed";

    RCLCPP_WARN(
        get_logger(),
        "Failed to request D435i RGB parameter descriptors: %s",
        e.what());
  }
}

void QrVisionNode::requestCameraControlValues()
{
  if (!camera_param_client_) {
    return;
  }

  std::vector<std::string> parameter_names;
  parameter_names.reserve(camera_numeric_controls_.size() + camera_bool_controls_.size());

  for (const CameraNumericControl &control : camera_numeric_controls_) {
    parameter_names.push_back(control.parameter_name);
  }

  for (const CameraBoolControl &control : camera_bool_controls_) {
    parameter_names.push_back(control.parameter_name);
  }

  camera_param_get_requested_ = true;
  camera_param_get_in_flight_ = true;

  try {
    camera_param_client_->get_parameters(
        parameter_names,
        [this](auto future) {
          camera_param_get_in_flight_ = false;

          try {
            applyCameraControlValues(future.get());
            camera_controls_status_text_ = "fixed defaults pending";
          } catch (const std::exception &e) {
            camera_controls_status_text_ = "get parameters failed";
            RCLCPP_WARN(
                get_logger(),
                "Failed to read D435i RGB parameters: %s",
                e.what());
          }
        });
  } catch (const std::exception &e) {
    camera_param_get_in_flight_ = false;
    camera_controls_status_text_ = "get request failed";

    RCLCPP_WARN(
        get_logger(),
        "Failed to request D435i RGB parameter values: %s",
        e.what());
  }
}

void QrVisionNode::applyCameraControlDescriptors(
    const std::vector<rcl_interfaces::msg::ParameterDescriptor> &descriptors)
{
  for (const rcl_interfaces::msg::ParameterDescriptor &descriptor : descriptors) {
    for (std::size_t i = 0U; i < camera_numeric_controls_.size(); ++i) {
      CameraNumericControl &control = camera_numeric_controls_[i];

      if (descriptor.name != control.parameter_name) {
        continue;
      }

      const auto descriptor_type = static_cast<rclcpp::ParameterType>(descriptor.type);

      if (descriptor_type == rclcpp::ParameterType::PARAMETER_INTEGER ||
          descriptor_type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        control.parameter_type = descriptor_type;
      }

      int descriptor_min = control.fallback_min;
      int descriptor_max = control.fallback_max;

      if (!descriptor.integer_range.empty()) {
        descriptor_min = static_cast<int>(descriptor.integer_range.front().from_value);
        descriptor_max = static_cast<int>(descriptor.integer_range.front().to_value);
      } else if (!descriptor.floating_point_range.empty()) {
        descriptor_min = static_cast<int>(std::round(
            descriptor.floating_point_range.front().from_value));
        descriptor_max = static_cast<int>(std::round(
            descriptor.floating_point_range.front().to_value));
      }

      if (descriptor_min > descriptor_max) {
        std::swap(descriptor_min, descriptor_max);
      }

      if (descriptor_min == descriptor_max) {
        descriptor_min = control.fallback_min;
        descriptor_max = control.fallback_max;
      }

      control.min_value = descriptor_min;
      control.max_value = descriptor_max;
      control.current_value = std::clamp(
          control.current_value,
          control.min_value,
          control.max_value);
      control.last_sent_value = std::clamp(
          control.last_sent_value,
          control.min_value,
          control.max_value);

      if (camera_controls_attached_) {
        cv::setTrackbarMax(
            control.label,
            window_name_,
            std::max(1, control.max_value - control.min_value));
        updateCameraTrackbarPosition(i);
      }
    }
  }
}

void QrVisionNode::applyCameraControlValues(
    const std::vector<rclcpp::Parameter> &parameters)
{
  for (const rclcpp::Parameter &parameter : parameters) {
    for (std::size_t i = 0U; i < camera_numeric_controls_.size(); ++i) {
      CameraNumericControl &control = camera_numeric_controls_[i];

      if (parameter.get_name() != control.parameter_name) {
        continue;
      }

      int value = control.current_value;

      if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        value = static_cast<int>(parameter.as_int());
        control.parameter_type = rclcpp::ParameterType::PARAMETER_INTEGER;
      } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        value = static_cast<int>(std::round(parameter.as_double()));
        control.parameter_type = rclcpp::ParameterType::PARAMETER_DOUBLE;
      } else {
        continue;
      }

      control.current_value = std::clamp(value, control.min_value, control.max_value);
      control.last_sent_value = control.current_value;
      control.send_pending = false;
      updateCameraTrackbarPosition(i);
    }

    for (CameraBoolControl &control : camera_bool_controls_) {
      if (parameter.get_name() != control.parameter_name ||
          parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        continue;
      }

      control.current_value = parameter.as_bool();
      control.last_sent_value = control.current_value;
      control.send_pending = false;
    }
  }

  updateCameraControlEnableStates();
  resetCameraControlsToDefaults();
}

void QrVisionNode::updateCameraControlEnableStates()
{
  const bool auto_exposure =
      camera_bool_controls_[kCameraAutoExposureControlIndex].current_value;
  const bool auto_white_balance =
      camera_bool_controls_[kCameraAutoWhiteBalanceControlIndex].current_value;

  camera_bool_controls_[kCameraAutoExposureControlIndex].enabled = true;
  camera_bool_controls_[kCameraAutoWhiteBalanceControlIndex].enabled = true;
  camera_bool_controls_[kCameraAutoExposurePriorityControlIndex].enabled = auto_exposure;

  camera_numeric_controls_[kCameraExposureControlIndex].enabled = !auto_exposure;
  camera_numeric_controls_[kCameraGainControlIndex].enabled = !auto_exposure;
  camera_numeric_controls_[kCameraWhiteBalanceControlIndex].enabled = !auto_white_balance;
}

void QrVisionNode::resetCameraControlsToDefaults()
{
  for (CameraBoolControl &control : camera_bool_controls_) {
    const bool restore_value = control.restore_value_initialized
        ? control.restore_value
        : control.current_value;

    control.current_value = restore_value;
    control.send_pending = control.current_value != control.last_sent_value;
  }

  updateCameraControlEnableStates();

  for (std::size_t i = 0U; i < camera_numeric_controls_.size(); ++i) {
    CameraNumericControl &control = camera_numeric_controls_[i];
    const int restore_value = control.restore_value_initialized
        ? control.restore_value
        : control.fallback_default;

    control.current_value = std::clamp(
        restore_value,
        control.min_value,
        control.max_value);
    control.send_pending = control.enabled && control.current_value != control.last_sent_value;
    updateCameraTrackbarPosition(i);
  }

  camera_controls_status_text_ = "reset defaults pending";
}

void QrVisionNode::sendPendingCameraControlParameters()
{
  if (!camera_controls_enabled_ ||
      !camera_param_node_online_ ||
      !camera_param_client_ ||
      camera_param_set_in_flight_) {
    return;
  }

  const rclcpp::Time now_time = this->now();

  if (last_camera_param_send_time_.nanoseconds() > 0) {
    const int64_t elapsed_ms =
        (now_time - last_camera_param_send_time_).nanoseconds() / 1000000;

    if (elapsed_ms < kCameraControlsSendDebounceMs) {
      return;
    }
  }

  std::vector<rclcpp::Parameter> parameters;

  for (const CameraBoolControl &control : camera_bool_controls_) {
    if (!control.enabled ||
        !control.send_pending ||
        control.current_value == control.last_sent_value) {
      continue;
    }

    parameters.emplace_back(control.parameter_name, control.current_value);
  }

  if (parameters.empty()) {
    for (const CameraNumericControl &control : camera_numeric_controls_) {
      if (!control.enabled ||
          !control.send_pending ||
          control.current_value == control.last_sent_value) {
        continue;
      }

      if (control.parameter_type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        parameters.emplace_back(
            control.parameter_name,
            static_cast<double>(control.current_value));
      } else {
        parameters.emplace_back(
            control.parameter_name,
            static_cast<int64_t>(control.current_value));
      }
    }
  }

  if (parameters.empty()) {
    return;
  }

  camera_param_set_in_flight_ = true;
  last_camera_param_send_time_ = now_time;

  try {
    camera_param_client_->set_parameters(
        parameters,
        [this, parameters](auto future) {
          camera_param_set_in_flight_ = false;

          try {
            handleCameraParameterSetResults(parameters, future.get());
          } catch (const std::exception &e) {
            camera_controls_status_text_ = "set parameters failed";
            RCLCPP_WARN(
                get_logger(),
                "Failed to set D435i RGB parameters: %s",
                e.what());
          }
        });
  } catch (const std::exception &e) {
    camera_param_set_in_flight_ = false;
    camera_controls_status_text_ = "set request failed";

    RCLCPP_WARN(
        get_logger(),
        "Failed to send D435i RGB parameter request: %s",
        e.what());
  }
}

void QrVisionNode::handleCameraParameterSetResults(
    const std::vector<rclcpp::Parameter> &parameters,
    const std::vector<rcl_interfaces::msg::SetParametersResult> &results)
{
  const std::size_t result_count = std::min(parameters.size(), results.size());
  bool all_success = result_count == parameters.size();

  for (std::size_t i = 0U; i < result_count; ++i) {
    const rclcpp::Parameter &parameter = parameters[i];
    const rcl_interfaces::msg::SetParametersResult &result = results[i];

    if (!result.successful) {
      all_success = false;
      RCLCPP_WARN(
          get_logger(),
          "D435i parameter rejected. name=%s reason=%s",
          parameter.get_name().c_str(),
          result.reason.c_str());
    }

    for (CameraNumericControl &control : camera_numeric_controls_) {
      if (parameter.get_name() != control.parameter_name) {
        continue;
      }

      if (result.successful) {
        control.last_sent_value = control.current_value;
      }

      control.send_pending = false;
    }

    for (CameraBoolControl &control : camera_bool_controls_) {
      if (parameter.get_name() != control.parameter_name) {
        continue;
      }

      if (result.successful) {
        control.last_sent_value = control.current_value;
      }

      control.send_pending = false;
    }
  }

  camera_controls_status_text_ = all_success
      ? "camera parameters applied"
      : "camera parameter rejected";
}

void QrVisionNode::handleCameraTrackbarChange(std::size_t index, int position)
{
  if (camera_controls_updating_trackbar_ || index >= camera_numeric_controls_.size()) {
    return;
  }

  CameraNumericControl &control = camera_numeric_controls_[index];
  const int value = std::clamp(
      control.min_value + position,
      control.min_value,
      control.max_value);

  control.current_value = value;
  control.trackbar_value = control.current_value - control.min_value;

  if (control.enabled) {
    control.send_pending = true;
  }
}

void QrVisionNode::handleCameraControlsClick(int x, int y)
{
  const cv::Point window_point(x, y);

  if (!camera_controls_panel_rect_.contains(window_point)) {
    return;
  }

  const cv::Point point(
      window_point.x - camera_controls_panel_rect_.x,
      window_point.y - camera_controls_panel_rect_.y);

  if (camera_reset_button_rect_.contains(point)) {
    resetCameraControlsToDefaults();
    return;
  }

  for (std::size_t i = 0U; i < camera_bool_controls_.size(); ++i) {
    const CameraBoolControl &control = camera_bool_controls_[i];

    if (!control.enabled) {
      continue;
    }

    if (control.true_rect.contains(point)) {
      setCameraBoolControl(i, true);
      return;
    }

    if (control.false_rect.contains(point)) {
      setCameraBoolControl(i, false);
      return;
    }
  }
}

void QrVisionNode::setCameraBoolControl(std::size_t index, bool value)
{
  if (index >= camera_bool_controls_.size() ||
      !camera_bool_controls_[index].enabled ||
      camera_bool_controls_[index].current_value == value) {
    return;
  }

  CameraBoolControl &control = camera_bool_controls_[index];
  control.current_value = value;
  control.send_pending = true;
  updateCameraControlEnableStates();

  if (index == kCameraAutoExposureControlIndex && !value) {
    camera_numeric_controls_[kCameraExposureControlIndex].send_pending = true;
    camera_numeric_controls_[kCameraGainControlIndex].send_pending = true;
  }

  if (index == kCameraAutoWhiteBalanceControlIndex && !value) {
    camera_numeric_controls_[kCameraWhiteBalanceControlIndex].send_pending = true;
  }
}

void QrVisionNode::updateCameraTrackbarPosition(std::size_t index)
{
  if (index >= camera_numeric_controls_.size() || !camera_controls_attached_) {
    return;
  }

  CameraNumericControl &control = camera_numeric_controls_[index];
  const int position = std::clamp(
      control.current_value - control.min_value,
      0,
      std::max(1, control.max_value - control.min_value));

  camera_controls_updating_trackbar_ = true;
  cv::setTrackbarPos(
      control.label,
      window_name_,
      position);
  camera_controls_updating_trackbar_ = false;
  control.trackbar_value = position;
}

void QrVisionNode::drawCameraBoolControl(
    cv::Mat &panel,
    CameraBoolControl &control,
    int y)
{
  const cv::Scalar text_color = control.enabled
      ? cv::Scalar(230, 235, 235)
      : cv::Scalar(115, 120, 125);
  const cv::Scalar active_color(90, 220, 140);
  const cv::Scalar inactive_color = control.enabled
      ? cv::Scalar(90, 95, 105)
      : cv::Scalar(70, 74, 78);
  const cv::Scalar border_color = control.enabled
      ? cv::Scalar(185, 195, 205)
      : cv::Scalar(90, 94, 98);
  const int true_x = 360;
  const int false_x = 510;
  const int radius = 9;

  cv::putText(
      panel,
      control.label,
      cv::Point(28, y + 6),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      text_color,
      1);

  auto draw_option = [&](
      bool option_value,
      int center_x,
      const char *label,
      cv::Rect &rect) {
    const cv::Point center(center_x, y);
    rect = cv::Rect(center_x - 22, y - 18, 90, 36);

    cv::circle(
        panel,
        center,
        radius,
        option_value == control.current_value ? active_color : inactive_color,
        cv::FILLED);
    cv::circle(panel, center, radius, border_color, 1);

    cv::putText(
        panel,
        label,
        cv::Point(center_x + 18, y + 5),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        text_color,
        1);
  };

  draw_option(true, true_x, "true", control.true_rect);
  draw_option(false, false_x, "false", control.false_rect);

  if (!control.enabled) {
    cv::putText(
        panel,
        "(requires auto exposure=true)",
        cv::Point(590, y + 5),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(105, 110, 116),
        1);
  }
}

void QrVisionNode::drawCameraResetButton(cv::Mat &panel)
{
  camera_reset_button_rect_ = cv::Rect(
      std::max(24, panel.cols - 198),
      16,
      174,
      30);

  const cv::Scalar fill_color = camera_param_node_online_
      ? cv::Scalar(65, 95, 115)
      : cv::Scalar(62, 68, 74);
  const cv::Scalar border_color = camera_param_node_online_
      ? cv::Scalar(130, 205, 235)
      : cv::Scalar(105, 112, 120);
  const cv::Scalar text_color = camera_param_node_online_
      ? cv::Scalar(230, 245, 255)
      : cv::Scalar(185, 190, 195);

  cv::rectangle(
      panel,
      camera_reset_button_rect_,
      fill_color,
      cv::FILLED);
  cv::rectangle(
      panel,
      camera_reset_button_rect_,
      border_color,
      1);
  cv::putText(
      panel,
      "Reset Defaults",
      cv::Point(camera_reset_button_rect_.x + 18, camera_reset_button_rect_.y + 21),
      cv::FONT_HERSHEY_SIMPLEX,
      0.52,
      text_color,
      1);
}

void QrVisionNode::drawCameraControlsPanel(cv::Mat &display)
{
  camera_controls_panel_rect_ = cv::Rect();

  if (!camera_controls_enabled_ || !camera_controls_attached_ || display.empty()) {
    return;
  }

  const int video_width = display.cols;
  const int video_height = display.rows;
  const int panel_width = std::max(kCameraControlsPanelWidthPx, video_width);
  const int panel_height = kCameraControlsPanelHeightPx;

  cv::Mat canvas(
      video_height + panel_height,
      panel_width,
      display.type(),
      cv::Scalar(12, 14, 18));
  display.copyTo(canvas(cv::Rect(0, 0, video_width, video_height)));
  camera_controls_panel_rect_ = cv::Rect(
      0,
      video_height,
      panel_width,
      panel_height);

  try {
    cv::Mat panel = canvas(camera_controls_panel_rect_);
    panel.setTo(cv::Scalar(24, 28, 32));

    cv::rectangle(
        panel,
        cv::Point(0, 0),
        cv::Point(panel.cols, 58),
        cv::Scalar(38, 45, 51),
        cv::FILLED);

    cv::putText(
        panel,
        "D435i RGB Controls",
        cv::Point(24, 34),
        cv::FONT_HERSHEY_SIMPLEX,
        0.72,
        cv::Scalar(230, 245, 255),
        2);
    drawCameraResetButton(panel);

    const std::string status = camera_controls_status_text_.empty()
        ? std::string("camera param node offline")
        : camera_controls_status_text_;
    cv::putText(
        panel,
        cv::format("node: %s", camera_param_node_.c_str()),
        cv::Point(28, 82),
        cv::FONT_HERSHEY_SIMPLEX,
        0.48,
        cv::Scalar(190, 200, 205),
        1);
    cv::putText(
        panel,
        cv::format("status: %s", status.c_str()),
        cv::Point(28, 104),
        cv::FONT_HERSHEY_SIMPLEX,
        0.48,
        camera_param_node_online_ ? cv::Scalar(115, 225, 155) : cv::Scalar(100, 165, 255),
        1);

    cv::putText(
        panel,
        "Numeric sliders are attached to this debug window.",
        cv::Point(28, 134),
        cv::FONT_HERSHEY_SIMPLEX,
        0.45,
        cv::Scalar(160, 168, 174),
        1);

    const std::array<int, kCameraNumericControlCount> numeric_y = {162, 187, 212};

    for (std::size_t i = 0U; i < camera_numeric_controls_.size(); ++i) {
      const CameraNumericControl &control = camera_numeric_controls_[i];
      const cv::Scalar text_color = control.enabled
          ? cv::Scalar(225, 232, 235)
          : cv::Scalar(115, 120, 125);
      const std::string suffix = control.enabled
          ? std::string("")
          : std::string("  disabled by auto mode");

      cv::putText(
          panel,
          cv::format(
              "%s = %d  range [%d..%d]%s",
              control.parameter_name.c_str(),
              control.current_value,
              control.min_value,
              control.max_value,
              suffix.c_str()),
          cv::Point(28, numeric_y[i]),
          cv::FONT_HERSHEY_SIMPLEX,
          0.5,
          text_color,
          1);
    }

    cv::line(
        panel,
        cv::Point(24, 232),
        cv::Point(panel.cols - 24, 232),
        cv::Scalar(60, 68, 75),
        1);

    drawCameraBoolControl(
        panel,
        camera_bool_controls_[kCameraAutoExposureControlIndex],
        262);
    drawCameraBoolControl(
        panel,
        camera_bool_controls_[kCameraAutoWhiteBalanceControlIndex],
        296);
    drawCameraBoolControl(
        panel,
        camera_bool_controls_[kCameraAutoExposurePriorityControlIndex],
        330);

    display = canvas;
  } catch (const cv::Exception &e) {
    camera_controls_panel_rect_ = cv::Rect();
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        log_throttle_ms_,
        "Failed to draw D435i controls in debug view: %s",
        e.what());
  }
}

void QrVisionNode::handleCameraTrackbarCallback(int position, void *userdata)
{
  CameraTrackbarContext *context = static_cast<CameraTrackbarContext *>(userdata);

  if (context == nullptr || context->node == nullptr) {
    return;
  }

  context->node->handleCameraTrackbarChange(context->index, position);
}

void QrVisionNode::handleCameraControlsMouseCallback(
    int event,
    int x,
    int y,
    int flags,
    void *userdata)
{
  (void)flags;

  if (event != cv::EVENT_LBUTTONDOWN) {
    return;
  }

  QrVisionNode *node = static_cast<QrVisionNode *>(userdata);

  if (node == nullptr) {
    return;
  }

  node->handleCameraControlsClick(x, y);
}

#if DRONE_PERCEPTION_HAS_BPU
QrVisionNode::BpuImageRect QrVisionNode::mapBpuDetectionToImageRect(
    const BpuYoloDetection &detection,
    int image_width,
    int image_height) const
{
  if (bpu_letterbox_.scale <= 0.0F || image_width <= 0 || image_height <= 0) {
    return {};
  }

  const float image_width_f = static_cast<float>(image_width);
  const float image_height_f = static_cast<float>(image_height);
  const float pad_x = static_cast<float>(bpu_letterbox_.pad_x);
  const float pad_y = static_cast<float>(bpu_letterbox_.pad_y);

  const float x_min = (detection.x_min_px - pad_x) / bpu_letterbox_.scale;
  const float y_min = (detection.y_min_px - pad_y) / bpu_letterbox_.scale;
  const float x_max = (detection.x_max_px - pad_x) / bpu_letterbox_.scale;
  const float y_max = (detection.y_max_px - pad_y) / bpu_letterbox_.scale;

  return {
      std::clamp(std::min(x_min, x_max), 0.0F, image_width_f),
      std::clamp(std::min(y_min, y_max), 0.0F, image_height_f),
      std::clamp(std::max(x_min, x_max), 0.0F, image_width_f),
      std::clamp(std::max(y_min, y_max), 0.0F, image_height_f)};
}

QrVisionNode::BpuImageRect QrVisionNode::expandImageRect(
    const BpuImageRect &image_rect,
    float padding_ratio,
    int image_width,
    int image_height) const
{
  const float width_px = image_rect.x_max - image_rect.x_min;
  const float height_px = image_rect.y_max - image_rect.y_min;

  if (width_px <= 1.0F || height_px <= 1.0F || image_width <= 0 || image_height <= 0) {
    return {};
  }

  const float pad_x_px = width_px * padding_ratio;
  const float pad_y_px = height_px * padding_ratio;
  const float image_width_f = static_cast<float>(image_width);
  const float image_height_f = static_cast<float>(image_height);

  return {
      std::clamp(image_rect.x_min - pad_x_px, 0.0F, image_width_f),
      std::clamp(image_rect.y_min - pad_y_px, 0.0F, image_height_f),
      std::clamp(image_rect.x_max + pad_x_px, 0.0F, image_width_f),
      std::clamp(image_rect.y_max + pad_y_px, 0.0F, image_height_f)};
}

void QrVisionNode::recognizeShelfTagFromDetections(
    const cv::Mat &color_image,
    const std::uint8_t *yuyv,
    int yuyv_stride,
    int yuyv_width,
    int yuyv_height,
    const std::vector<BpuYoloDetection> &detections,
    const char *input_mode)
{
  last_ocr_regions_.clear();

  if (!bpu_ocr_pipeline_) {
    updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
    return;
  }

  const bool has_bgr = !color_image.empty();
  const int image_width = has_bgr ? color_image.cols : yuyv_width;
  const int image_height = has_bgr ? color_image.rows : yuyv_height;

  if (image_width <= 0 || image_height <= 0 || (!has_bgr && yuyv == nullptr)) {
    updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
    return;
  }

  const BpuYoloDetection *best_shelf_tag = nullptr;

  for (const BpuYoloDetection &detection : detections) {
    if (detection.class_id != kYoloClassShelfTag) {
      continue;
    }

    if (best_shelf_tag == nullptr || detection.score > best_shelf_tag->score) {
      best_shelf_tag = &detection;
    }
  }

  if (best_shelf_tag == nullptr) {
    updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
    return;
  }

  const BpuImageRect image_rect = mapBpuDetectionToImageRect(
      *best_shelf_tag,
      image_width,
      image_height);
  const BpuImageRect padded_rect = expandImageRect(
      image_rect,
      ocr_yolo_padding_ratio_,
      image_width,
      image_height);

  if (padded_rect.x_max - padded_rect.x_min <= 1.0F ||
      padded_rect.y_max - padded_rect.y_min <= 1.0F) {
    updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
    return;
  }

  const std::array<cv::Point2f, 4> shelf_tag_box = {
      cv::Point2f(padded_rect.x_min, padded_rect.y_min),
      cv::Point2f(padded_rect.x_max, padded_rect.y_min),
      cv::Point2f(padded_rect.x_max, padded_rect.y_max),
      cv::Point2f(padded_rect.x_min, padded_rect.y_max)};

  ocr_invoked_this_frame_ = true;

  if (has_bgr) {
    last_ocr_regions_ =
        bpu_ocr_pipeline_->recognizeTextBoxes(color_image, {shelf_tag_box});
  } else {
    // Direct hover path: no full BGR. Convert only the padded shelf-tag ROI
    // from YUYV, then offset OCR regions back into image coordinates.
    const int roi_x = std::max(0, static_cast<int>(padded_rect.x_min)) & ~1;
    const int roi_y = std::max(0, static_cast<int>(padded_rect.y_min));
    const int roi_x_max = std::min(image_width, static_cast<int>(padded_rect.x_max));
    const int roi_y_max = std::min(image_height, static_cast<int>(padded_rect.y_max));
    const cv::Rect roi(roi_x, roi_y, roi_x_max - roi_x, roi_y_max - roi_y);

    if (roi.width <= 1 || roi.height <= 1) {
      updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
      return;
    }

    cv::Mat yuyv_roi(
        roi.height,
        roi.width,
        CV_8UC2,
        const_cast<std::uint8_t *>(yuyv) +
            static_cast<std::size_t>(roi.y) * yuyv_stride +
            static_cast<std::size_t>(roi.x) * 2,
        yuyv_stride);
    cv::Mat roi_bgr;
    cv::cvtColor(yuyv_roi, roi_bgr, cv::COLOR_YUV2BGR_YUY2);

    if (roi_bgr.empty()) {
      updateVisualCodeCategoryStability({}, "shelf", shelf_code_state_, "ocr");
      return;
    }

    const std::array<cv::Point2f, 4> roi_box = {
        cv::Point2f(0.0F, 0.0F),
        cv::Point2f(static_cast<float>(roi.width), 0.0F),
        cv::Point2f(
            static_cast<float>(roi.width),
            static_cast<float>(roi.height)),
        cv::Point2f(0.0F, static_cast<float>(roi.height))};
    last_ocr_regions_ =
        bpu_ocr_pipeline_->recognizeTextBoxes(roi_bgr, {roi_box});

    for (OcrTextRegion &region : last_ocr_regions_) {
      for (cv::Point2f &point : region.box) {
        point.x += static_cast<float>(roi.x);
        point.y += static_cast<float>(roi.y);
      }
    }
  }

  std::vector<DecodedVisualCode> ocr_codes;
  std::string ocr_text = "none";
  float ocr_score = 0.0F;

  if (!last_ocr_regions_.empty()) {
    const OcrTextRegion &region = last_ocr_regions_.front();
    ocr_text = region.text;
    ocr_score = region.score;
    ocr_codes.push_back({region.text, "shelf", "OCR", 0.0});
  }

  updateVisualCodeCategoryStability(ocr_codes, "shelf", shelf_code_state_, "ocr");

  const OcrTimingStats &ocr_timing = bpu_ocr_pipeline_->lastTiming();
  RCLCPP_DEBUG_THROTTLE(
      get_logger(),
      *get_clock(),
      log_throttle_ms_,
      "BPU OCR shelf_tag mode=%s shelf_tag_score=%.3f roi=[%.0f,%.0f,%.0f,%.0f] "
      "text=%s text_score=%.3f rec[crop=%.2f pre=%.2f infer=%.2f decode=%.2f total=%.2f]",
      input_mode,
      best_shelf_tag->score,
      padded_rect.x_min,
      padded_rect.y_min,
      padded_rect.x_max,
      padded_rect.y_max,
      ocr_text.c_str(),
      ocr_score,
      ocr_timing.rec_crop_ms,
      ocr_timing.rec_preprocess_ms,
      ocr_timing.rec_infer_ms,
      ocr_timing.rec_decode_ms,
      ocr_timing.rec_total_ms);
}

void QrVisionNode::drawBpuDetections(cv::Mat &display) const
{
  for (const BpuYoloDetection &detection : last_bpu_detections_) {
    const BpuImageRect image_rect = mapBpuDetectionToImageRect(
        detection,
        display.cols,
        display.rows);

    if (image_rect.x_max - image_rect.x_min <= 1.0F ||
        image_rect.y_max - image_rect.y_min <= 1.0F) {
      continue;
    }

    const cv::Point top_left(
        static_cast<int>(image_rect.x_min),
        static_cast<int>(image_rect.y_min));
    const cv::Point bottom_right(
        static_cast<int>(image_rect.x_max),
        static_cast<int>(image_rect.y_max));

    cv::Scalar box_color(0, 255, 0);

    if (detection.class_id == kYoloClassQrcode) {
      box_color = cv::Scalar(120, 220, 255);
    } else if (detection.class_id == kYoloClassPackage) {
      box_color = cv::Scalar(120, 255, 160);
    } else if (detection.class_id == kYoloClassShelfTag) {
      box_color = cv::Scalar(255, 210, 120);
    }

    cv::rectangle(
        display,
        top_left,
        bottom_right,
        box_color,
        2);

    const std::string label = cv::format(
        "%s %.2f",
        BpuYoloDetector::className(detection.class_id),
        detection.score);
    const int label_y = top_left.y > 18 ? top_left.y - 6 : top_left.y + 18;

    cv::putText(
        display,
        label,
        cv::Point(top_left.x, label_y),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        box_color,
        1);
  }
}

void QrVisionNode::drawOcrRegions(cv::Mat &display) const
{
  for (const OcrTextRegion &region : last_ocr_regions_) {
    std::vector<cv::Point> polygon;
    polygon.reserve(region.box.size());

    for (const cv::Point2f &point : region.box) {
      polygon.emplace_back(
          static_cast<int>(std::round(point.x)),
          static_cast<int>(std::round(point.y)));
    }

    const cv::Point *points = polygon.data();
    const int point_count = static_cast<int>(polygon.size());

    cv::polylines(
        display,
        &points,
        &point_count,
        1,
        true,
        cv::Scalar(255, 180, 80),
        2);

    if (polygon.empty()) {
      continue;
    }

    cv::putText(
        display,
        cv::format("%s %.2f", region.text.c_str(), region.score),
        cv::Point(polygon.front().x, std::max(18, polygon.front().y - 6)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(255, 200, 120),
      1);
  }
}

void QrVisionNode::drawQrPreprocessPreview(cv::Mat &display) const
{
  if (display.empty() || debug_qr_preprocess_preview_.empty()) {
    return;
  }

  cv::Mat preview_bgr;

  if (debug_qr_preprocess_preview_.channels() == 1) {
    cv::cvtColor(debug_qr_preprocess_preview_, preview_bgr, cv::COLOR_GRAY2BGR);
  } else {
    preview_bgr = debug_qr_preprocess_preview_.clone();
  }

  if (preview_bgr.empty()) {
    return;
  }

  const double scale = std::min(
      static_cast<double>(kQrPreprocessPreviewMaxSizePx) / static_cast<double>(preview_bgr.cols),
      static_cast<double>(kQrPreprocessPreviewMaxSizePx) / static_cast<double>(preview_bgr.rows));
  const int preview_width = std::max(1, static_cast<int>(std::round(static_cast<double>(preview_bgr.cols) * scale)));
  const int preview_height = std::max(1, static_cast<int>(std::round(static_cast<double>(preview_bgr.rows) * scale)));

  cv::Mat resized_preview;
  cv::resize(
      preview_bgr,
      resized_preview,
      cv::Size(preview_width, preview_height),
      0.0,
      0.0,
      cv::INTER_NEAREST);

  const int margin_px = 16;
  const int label_height_px = 24;
  const int panel_width = resized_preview.cols + margin_px;
  const int panel_height = resized_preview.rows + label_height_px + margin_px;
  const int panel_x = std::max(0, display.cols - panel_width - margin_px);
  const int panel_y = std::max(0, display.rows - panel_height - margin_px);
  const cv::Rect panel_rect(
      panel_x,
      panel_y,
      std::min(panel_width, display.cols - panel_x),
      std::min(panel_height, display.rows - panel_y));

  if (panel_rect.width <= margin_px || panel_rect.height <= label_height_px) {
    return;
  }

  cv::Mat overlay = display.clone();
  cv::rectangle(
      overlay,
      panel_rect,
      cv::Scalar(20, 20, 20),
      cv::FILLED);
  cv::addWeighted(overlay, 0.58, display, 0.42, 0.0, display);

  const std::string label = debug_qr_preprocess_mode_.empty()
      ? std::string("QR preprocess")
      : cv::format("QR preprocess: %s", debug_qr_preprocess_mode_.c_str());
  cv::putText(
      display,
      label,
      cv::Point(panel_x + 8, panel_y + 17),
      cv::FONT_HERSHEY_SIMPLEX,
      0.45,
      cv::Scalar(120, 220, 255),
      1);

  const cv::Rect image_rect(
      panel_x + 8,
      panel_y + label_height_px,
      std::min(resized_preview.cols, display.cols - panel_x - 8),
      std::min(resized_preview.rows, display.rows - panel_y - label_height_px - 8));

  if (image_rect.width <= 0 || image_rect.height <= 0) {
    return;
  }

  resized_preview(cv::Rect(0, 0, image_rect.width, image_rect.height))
      .copyTo(display(image_rect));
  cv::rectangle(
      display,
      image_rect,
      cv::Scalar(120, 220, 255),
      1);
}
#endif

#if DRONE_PERCEPTION_HAS_REALSENSE && DRONE_PERCEPTION_HAS_RDK_MEDIA_CODEC
void QrVisionNode::startVideoStream()
{
  auto encoder = std::make_unique<drone_perception::RdkH264Encoder>();
  drone_perception::RdkH264Encoder::Config encoder_config;
  encoder_config.width = video_stream_width_;
  encoder_config.height = video_stream_height_;
  encoder_config.fps = video_stream_fps_;
  encoder_config.bitrate_kbps = video_stream_bitrate_kbps_;
  encoder_config.gop = video_stream_gop_;

  std::string error;
  if (!encoder->initialize(encoder_config, error)) {
    RCLCPP_ERROR(
        get_logger(),
        "video stream encoder init failed: %s (video_stream_enabled disabled)",
        error.c_str());
    video_stream_enabled_ = false;
    return;
  }
  video_encoder_ = std::move(encoder);

  {
    std::lock_guard<std::mutex> lock(video_stream_mutex_);
    video_stream_running_ = true;
  }

  video_stream_thread_ = std::thread(&QrVisionNode::videoStreamLoop, this);

  RCLCPP_INFO(
      get_logger(),
      "video stream worker started: %dx%d@%d H.264 CBR %d kbit/s GOP %d -> %s",
      video_stream_width_,
      video_stream_height_,
      video_stream_fps_,
      video_stream_bitrate_kbps_,
      video_stream_gop_,
      video_stream_publish_url_.c_str());
}

void QrVisionNode::stopVideoStream()
{
  {
    std::lock_guard<std::mutex> lock(video_stream_mutex_);
    video_stream_running_ = false;
    video_stream_frame_pending_ = false;
    latest_video_frame_ = {};
  }
  video_stream_cv_.notify_all();

  if (video_stream_thread_.joinable()) {
    video_stream_thread_.join();
  }

  video_encoder_.reset();

  RCLCPP_INFO(
      get_logger(),
      "video stream worker stopped: encoded=%llu dropped=%llu write_errors=%llu",
      static_cast<unsigned long long>(video_stream_encoded_count_.load()),
      static_cast<unsigned long long>(video_stream_dropped_count_.load()),
      static_cast<unsigned long long>(video_stream_write_errors_.load()));
}

void QrVisionNode::submitVideoFrame(const drone_perception::D435ColorFrame &frame)
{
  std::lock_guard<std::mutex> lock(video_stream_mutex_);
  if (!video_stream_running_) {
    return;
  }
  // Latest-frame slot: an unread frame is dropped, never queued unbounded.
  if (video_stream_frame_pending_) {
    ++video_stream_dropped_count_;
  }
  latest_video_frame_ = frame;
  video_stream_frame_pending_ = true;
  video_stream_cv_.notify_one();
}

void QrVisionNode::videoStreamLoop()
{
  // Encodes the newest YUYV frame into H.264 and feeds it to an FFmpeg
  // -c:v copy RTSP push toward the MediaMTX started by start_d435.sh. The
  // worker is independent of the detection path: when the link is slow the
  // pipe write blocks here only, and detection keeps running.
  //
  // FFmpeg is spawned with posix_spawn + POSIX_SPAWN_CLOEXEC_DEFAULT so the
  // node's inherited D435i camera fd does NOT leak into the child (popen would
  // leave it open, violating "only qr_vision_node holds the camera").
  const std::string ffmpeg_argv[] = {
      "ffmpeg", "-hide_banner", "-loglevel", "warning", "-fflags", "+genpts",
      "-f", "h264", "-i", "pipe:0", "-map", "0:v:0", "-c:v", "copy", "-an",
      "-f", "rtsp", "-rtsp_transport", "tcp", video_stream_publish_url_};
  std::vector<char *> args;
  args.reserve(sizeof(ffmpeg_argv) / sizeof(ffmpeg_argv[0]) + 1U);
  for (const std::string & arg : ffmpeg_argv) {
    args.push_back(const_cast<char *>(arg.c_str()));
  }
  args.push_back(nullptr);

  int pipe_fds[2] = {-1, -1};
  if (pipe2(pipe_fds, O_CLOEXEC) != 0) {
    RCLCPP_ERROR(
        get_logger(), "video stream: pipe2 failed: %s", std::strerror(errno));
    return;
  }

  const pid_t ffmpeg_pid = fork();
  if (ffmpeg_pid == 0) {
    // Child: wire the pipe read end to stdin, then close every other fd so
    // the inherited D435i camera fd never leaks into FFmpeg.
    dup2(pipe_fds[0], STDIN_FILENO);
    DIR * proc_fd_dir = opendir("/proc/self/fd");
    if (proc_fd_dir != nullptr) {
      const int dir_fd = dirfd(proc_fd_dir);
      struct dirent * entry = nullptr;
      while ((entry = readdir(proc_fd_dir)) != nullptr) {
        const int fd = std::atoi(entry->d_name);
        if (fd >= 3 && fd != dir_fd) {
          close(fd);
        }
      }
      closedir(proc_fd_dir);
    } else {
      for (int fd = 3; fd < 1024; ++fd) {
        close(fd);
      }
    }
    execvp("ffmpeg", args.data());
    _exit(127);
  }

  if (ffmpeg_pid < 0) {
    RCLCPP_ERROR(
        get_logger(), "video stream: failed to fork ffmpeg: %s",
        std::strerror(errno));
    close(pipe_fds[0]);
    close(pipe_fds[1]);
    return;
  }
  close(pipe_fds[0]);  // parent keeps only the write end

  std::vector<std::uint8_t> h264_buffer;
  std::string encode_error;

  while (true) {
    drone_perception::D435ColorFrame frame;
    {
      std::unique_lock<std::mutex> lock(video_stream_mutex_);
      video_stream_cv_.wait(lock, [this]() {
        return !video_stream_running_ || video_stream_frame_pending_;
      });
      if (!video_stream_running_) {
        break;
      }
      frame = std::move(latest_video_frame_);
      video_stream_frame_pending_ = false;
    }

    if (frame.yuyv.empty()) {
      continue;
    }

    h264_buffer.clear();
    encode_error.clear();
    if (!video_encoder_ ||
        !video_encoder_->encodeFrame(
            frame.yuyv.data(),
            frame.stride_bytes,
            frame.sequence,
            h264_buffer,
            encode_error)) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "video stream encode failed: %s",
          encode_error.c_str());
      continue;
    }

    ++video_stream_encoded_count_;

    if (!h264_buffer.empty()) {
      const std::uint8_t * data = h264_buffer.data();
      std::size_t remaining = h264_buffer.size();
      while (remaining > 0) {
        const ssize_t written = write(pipe_fds[1], data, remaining);
        if (written > 0) {
          data += written;
          remaining -= static_cast<std::size_t>(written);
          continue;
        }
        if (written < 0 && errno == EINTR) {
          continue;
        }
        ++video_stream_write_errors_;
        break;
      }
    }
  }

  close(pipe_fds[1]);  // EOF -> FFmpeg exits
  int child_status = 0;
  waitpid(ffmpeg_pid, &child_status, 0);
}
#endif
