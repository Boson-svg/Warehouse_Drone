#include "drone_perception/bpu_ocr_pipeline.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <cctype>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace
{
using SteadyClock = std::chrono::steady_clock;

static constexpr int32_t kSingleModelFileCount = 1;
static constexpr int32_t kMinModelCount = 1;
static constexpr int kDetInputWidthIndex = 3;
static constexpr int kDetInputHeightIndex = 2;
static constexpr int kRecInputWidthIndex = 3;
static constexpr int kRecInputHeightIndex = 2;

static constexpr char kCtcAlphabet[] =
    "0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~!\"#$%&'()*+,-./ ";

std::size_t shapeElementCount(const hbDNNTensorShape &shape)
{
  std::size_t element_count = 1U;

  for (int32_t i = 0; i < shape.numDimensions; ++i) {
    element_count *= static_cast<std::size_t>(std::max(1, shape.dimensionSize[i]));
  }

  return element_count;
}

std::pair<int, int> resolveSingleChannelMapShape(const hbDNNTensorShape &shape)
{
  std::vector<int> dims;
  dims.reserve(static_cast<std::size_t>(shape.numDimensions));

  for (int32_t i = 0; i < shape.numDimensions; ++i) {
    const int dim = shape.dimensionSize[i];

    if (dim > 1) {
      dims.push_back(dim);
    }
  }

  if (dims.size() < 2U) {
    throw std::runtime_error("cannot resolve single-channel map shape");
  }

  return {dims[dims.size() - 2U], dims[dims.size() - 1U]};
}

std::pair<int, int> resolveSequenceShape(const hbDNNTensorShape &shape)
{
  std::vector<int> dims;
  dims.reserve(static_cast<std::size_t>(shape.numDimensions));

  for (int32_t i = 0; i < shape.numDimensions; ++i) {
    const int dim = shape.dimensionSize[i];

    if (dim > 1) {
      dims.push_back(dim);
    }
  }

  if (dims.size() < 2U) {
    throw std::runtime_error("cannot resolve sequence shape");
  }

  return {dims[dims.size() - 2U], dims[dims.size() - 1U]};
}

std::vector<uint8_t> bgrToNv12(const cv::Mat &bgr_image)
{
  cv::Mat yuv_i420;
  cv::cvtColor(bgr_image, yuv_i420, cv::COLOR_BGR2YUV_I420);

  const std::size_t y_size =
      static_cast<std::size_t>(bgr_image.cols) *
      static_cast<std::size_t>(bgr_image.rows);
  const std::size_t uv_size = y_size / 4U;

  const uint8_t *y_plane = yuv_i420.ptr<uint8_t>(0);
  const uint8_t *u_plane = y_plane + y_size;
  const uint8_t *v_plane = u_plane + uv_size;

  std::vector<uint8_t> nv12(y_size + uv_size * 2U);
  std::memcpy(nv12.data(), y_plane, y_size);

  uint8_t *uv_plane = nv12.data() + y_size;

  for (std::size_t i = 0; i < uv_size; ++i) {
    uv_plane[i * 2U] = u_plane[i];
    uv_plane[i * 2U + 1U] = v_plane[i];
  }

  return nv12;
}

std::array<cv::Point2f, 4> rectToOrderedPoints(const cv::RotatedRect &rect)
{
  std::array<cv::Point2f, 4> points{};
  rect.points(points.data());

  std::sort(
      points.begin(),
      points.end(),
      [](const cv::Point2f &a, const cv::Point2f &b) {
        if (a.y == b.y) {
          return a.x < b.x;
        }

        return a.y < b.y;
      });

  std::array<cv::Point2f, 2> top{points[0], points[1]};
  std::array<cv::Point2f, 2> bottom{points[2], points[3]};

  std::sort(top.begin(), top.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
    return a.x < b.x;
  });
  std::sort(bottom.begin(), bottom.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
    return a.x < b.x;
  });

  return {top[0], top[1], bottom[1], bottom[0]};
}

bool isDigitChar(char value)
{
  return std::isdigit(static_cast<unsigned char>(value)) != 0;
}

double elapsedMs(
    const SteadyClock::time_point &start,
    const SteadyClock::time_point &end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

float clampFloat(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }

  if (value > max_value) {
    return max_value;
  }

  return value;
}

float contourUnclipDistance(
    const std::vector<cv::Point> &contour,
    float unclip_ratio)
{
  const double contour_area = std::abs(cv::contourArea(contour));
  const double contour_perimeter = cv::arcLength(contour, true);

  if (contour_area <= 0.0 || contour_perimeter <= 0.0) {
    return 0.0F;
  }

  return static_cast<float>(contour_area * static_cast<double>(unclip_ratio) / contour_perimeter);
}

}  // namespace

class BpuOcrPipeline::BpuDnnModel
{
public:
  explicit BpuDnnModel(const std::string &model_path)
  {
    loadModel(model_path);
  }

  ~BpuDnnModel()
  {
    releaseModel();
  }

  BpuDnnModel(const BpuDnnModel &) = delete;
  BpuDnnModel &operator=(const BpuDnnModel &) = delete;

  const hbDNNTensorProperties &inputProperties() const
  {
    return _input_properties;
  }

  const std::vector<hbDNNTensorProperties> &outputProperties() const
  {
    return _output_properties;
  }

  const std::string &modelName() const
  {
    return _model_name;
  }

  std::vector<std::vector<float>> infer(
      const void *input_data,
      std::size_t input_byte_size)
  {
    if (_dnn_handle == nullptr) {
      throw std::runtime_error("BPU model is not loaded");
    }

    if (!_input_tensor_allocated || _output_tensors.empty()) {
      throw std::runtime_error("BPU tensors are not allocated");
    }

    if (input_data == nullptr) {
      throw std::runtime_error("BPU input data is null");
    }

    const std::size_t expected_size =
        static_cast<std::size_t>(_input_properties.alignedByteSize);

    if (input_byte_size != expected_size) {
      throw std::runtime_error(
          "unexpected input size, expected=" + std::to_string(expected_size) +
          ", actual=" + std::to_string(input_byte_size));
    }

    std::memcpy(_input_tensor.sysMem[0].virAddr, input_data, input_byte_size);
    checkRet(
        hbSysFlushMem(&_input_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN),
        "hbSysFlushMem input clean");

    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

    hbDNNTensor *input_tensor = &_input_tensor;
    hbDNNTensor *output_tensors = _output_tensors.data();

    checkRet(
        hbDNNInfer(&task_handle, &output_tensors, input_tensor, _dnn_handle, &infer_ctrl_param),
        "hbDNNInfer");

    const int wait_ret = hbDNNWaitTaskDone(task_handle, 0);
    const int release_ret = hbDNNReleaseTask(task_handle);

    checkRet(wait_ret, "hbDNNWaitTaskDone");
    checkRet(release_ret, "hbDNNReleaseTask");

    std::vector<std::vector<float>> outputs;
    outputs.reserve(_output_tensors.size());

    for (std::size_t i = 0; i < _output_tensors.size(); ++i) {
      checkRet(
          hbSysFlushMem(&_output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE),
          "hbSysFlushMem output invalidate");

      const std::size_t value_count =
          shapeElementCount(_output_properties[i].validShape);
      const float *output_data = static_cast<const float *>(_output_tensors[i].sysMem[0].virAddr);
      outputs.emplace_back(output_data, output_data + value_count);
    }

    return outputs;
  }

  void printModelInfo() const
  {
    if (_dnn_handle == nullptr) {
      throw std::runtime_error("BPU model is not loaded");
    }

    std::cout << "model name: " << _model_name << std::endl;
    std::cout << "input count: " << _input_count << std::endl;
    std::cout << "output count: " << _output_count << std::endl;
  }

private:
  void loadModel(const std::string &model_path)
  {
    releaseModel();

    try {
      const char *model_files[] = {model_path.c_str()};

      checkRet(
          hbDNNInitializeFromFiles(&_packed_dnn_handle, model_files, kSingleModelFileCount),
          "hbDNNInitializeFromFiles");

      const char **model_name_list = nullptr;
      int32_t model_name_count = 0;

      checkRet(
          hbDNNGetModelNameList(&model_name_list, &model_name_count, _packed_dnn_handle),
          "hbDNNGetModelNameList");

      if (model_name_count < kMinModelCount) {
        throw std::runtime_error("no model found in bin file");
      }

      _model_name = model_name_list[0];

      checkRet(
          hbDNNGetModelHandle(&_dnn_handle, _packed_dnn_handle, _model_name.c_str()),
          "hbDNNGetModelHandle");

      checkRet(
          hbDNNGetInputCount(&_input_count, _dnn_handle),
          "hbDNNGetInputCount");

      checkRet(
          hbDNNGetOutputCount(&_output_count, _dnn_handle),
          "hbDNNGetOutputCount");

      queryTensorProperties();
      allocateTensors();
    } catch (...) {
      releaseModel();
      throw;
    }
  }

  void queryTensorProperties()
  {
    if (_input_count != 1) {
      throw std::runtime_error("unsupported input count: " + std::to_string(_input_count));
    }

    checkRet(
        hbDNNGetInputTensorProperties(&_input_properties, _dnn_handle, 0),
        "hbDNNGetInputTensorProperties");

    _output_properties.clear();
    _output_properties.resize(static_cast<std::size_t>(_output_count));

    for (int32_t i = 0; i < _output_count; ++i) {
      checkRet(
          hbDNNGetOutputTensorProperties(&_output_properties[static_cast<std::size_t>(i)], _dnn_handle, i),
          "hbDNNGetOutputTensorProperties");
    }
  }

  void allocateTensors()
  {
    releaseTensors();

    try {
      _input_tensor.properties = _input_properties;
      checkRet(
          hbSysAllocCachedMem(&_input_tensor.sysMem[0], _input_properties.alignedByteSize),
          "hbSysAllocCachedMem input");
      _input_tensor_allocated = true;

      _output_tensors.resize(static_cast<std::size_t>(_output_count));
      _output_tensor_allocated.assign(static_cast<std::size_t>(_output_count), 0U);

      for (int32_t i = 0; i < _output_count; ++i) {
        hbDNNTensor &tensor = _output_tensors[static_cast<std::size_t>(i)];
        tensor.properties = _output_properties[static_cast<std::size_t>(i)];

        checkRet(
            hbSysAllocCachedMem(&tensor.sysMem[0], tensor.properties.alignedByteSize),
            "hbSysAllocCachedMem output");

        _output_tensor_allocated[static_cast<std::size_t>(i)] = 1U;
      }
    } catch (...) {
      releaseTensors();
      throw;
    }
  }

  void releaseTensors()
  {
    if (_input_tensor_allocated) {
      hbSysFreeMem(&_input_tensor.sysMem[0]);
      _input_tensor = {};
      _input_tensor_allocated = false;
    }

    for (std::size_t i = 0; i < _output_tensors.size(); ++i) {
      if (i >= _output_tensor_allocated.size() || _output_tensor_allocated[i] == 0U) {
        continue;
      }

      hbSysFreeMem(&_output_tensors[i].sysMem[0]);
    }

    _output_tensors.clear();
    _output_tensor_allocated.clear();
  }

  void releaseModel()
  {
    releaseTensors();

    if (_packed_dnn_handle == nullptr) {
      return;
    }

    hbDNNRelease(_packed_dnn_handle);
    _packed_dnn_handle = nullptr;
    _dnn_handle = nullptr;
    _input_count = 0;
    _output_count = 0;
    _input_properties = {};
    _output_properties.clear();
    _model_name.clear();
  }

  static void checkRet(int ret, const std::string &step)
  {
    if (ret == 0) {
      return;
    }

    throw std::runtime_error(step + " failed, ret=" + std::to_string(ret));
  }

  hbPackedDNNHandle_t _packed_dnn_handle{nullptr};
  hbDNNHandle_t _dnn_handle{nullptr};
  std::string _model_name;
  int32_t _input_count{0};
  int32_t _output_count{0};
  hbDNNTensorProperties _input_properties{};
  std::vector<hbDNNTensorProperties> _output_properties;
  hbDNNTensor _input_tensor{};
  std::vector<hbDNNTensor> _output_tensors;
  bool _input_tensor_allocated{false};
  std::vector<uint8_t> _output_tensor_allocated;
};

BpuOcrPipeline::BpuOcrPipeline(const BpuOcrConfig &config)
    : _config(config)
{
  if (_config.rec_model_path.empty()) {
    _config.rec_model_path = defaultRecognitionModelPath();
  }

  if (_config.enable_detection_model && _config.det_model_path.empty()) {
    _config.det_model_path = defaultDetectionModelPath();
  }

  if (_config.enable_detection_model) {
    _det_model = std::make_unique<BpuDnnModel>(_config.det_model_path);
  }

  _rec_model = std::make_unique<BpuDnnModel>(_config.rec_model_path);

  const hbDNNTensorShape &rec_shape = _rec_model->inputProperties().validShape;

  if (rec_shape.numDimensions >= 4) {
    _config.rec_input_height_px = rec_shape.dimensionSize[kRecInputHeightIndex];
    _config.rec_input_width_px = rec_shape.dimensionSize[kRecInputWidthIndex];
  }
}

BpuOcrPipeline::~BpuOcrPipeline() = default;

std::vector<OcrTextRegion> BpuOcrPipeline::infer(const cv::Mat &bgr_image)
{
  _last_timing = {};

  if (bgr_image.empty()) {
    return {};
  }

  if (!_det_model) {
    throw std::runtime_error("OCR detection model is not loaded");
  }

  const auto det_preprocess_t0 = SteadyClock::now();
  const DetectionInput det_input = prepareDetectionInput(bgr_image);
  const auto det_preprocess_t1 = SteadyClock::now();

  const auto det_infer_t0 = SteadyClock::now();
  const std::vector<std::vector<float>> det_outputs =
      _det_model->infer(det_input.nv12.data(), det_input.nv12.size());
  const auto det_infer_t1 = SteadyClock::now();

  const auto det_postprocess_t0 = SteadyClock::now();
  const std::vector<DetectionBox> boxes =
      detectTextBoxes(det_outputs, bgr_image.size(), det_input);
  const auto det_postprocess_t1 = SteadyClock::now();

  _last_timing.det_preprocess_ms = elapsedMs(det_preprocess_t0, det_preprocess_t1);
  _last_timing.det_infer_ms = elapsedMs(det_infer_t0, det_infer_t1);
  _last_timing.det_postprocess_ms = elapsedMs(det_postprocess_t0, det_postprocess_t1);
  _last_timing.rec_box_count = static_cast<int>(boxes.size());

  std::vector<OcrTextRegion> regions;
  regions.reserve(boxes.size());
  const auto rec_total_t0 = SteadyClock::now();

  for (const DetectionBox &box : boxes) {
    const auto rec_crop_t0 = SteadyClock::now();
    const cv::Mat crop = cropAndRectify(bgr_image, box);
    const auto rec_crop_t1 = SteadyClock::now();
    _last_timing.rec_crop_ms += elapsedMs(rec_crop_t0, rec_crop_t1);

    if (crop.empty()) {
      continue;
    }

    const RecognitionResult result = recognizeText(crop);

    if (result.text.empty()) {
      continue;
    }

    regions.push_back({box.points, result.text, result.score});
  }

  const auto rec_total_t1 = SteadyClock::now();
  _last_timing.rec_total_ms = elapsedMs(rec_total_t0, rec_total_t1);

  return regions;
}

std::vector<OcrTextRegion> BpuOcrPipeline::recognizeTextBoxes(
    const cv::Mat &bgr_image,
    const std::vector<std::array<cv::Point2f, 4>> &boxes)
{
  _last_timing = {};

  if (bgr_image.empty() || boxes.empty()) {
    return {};
  }

  _last_timing.rec_box_count = static_cast<int>(boxes.size());

  std::vector<OcrTextRegion> regions;
  regions.reserve(boxes.size());
  const auto rec_total_t0 = SteadyClock::now();

  for (const std::array<cv::Point2f, 4> &points : boxes) {
    const DetectionBox box{points};

    const auto rec_crop_t0 = SteadyClock::now();
    const cv::Mat crop = cropAndRectify(bgr_image, box);
    const auto rec_crop_t1 = SteadyClock::now();
    _last_timing.rec_crop_ms += elapsedMs(rec_crop_t0, rec_crop_t1);

    if (crop.empty()) {
      continue;
    }

    const RecognitionResult result = recognizeText(crop);

    if (result.text.empty()) {
      continue;
    }

    regions.push_back({box.points, result.text, result.score});
  }

  const auto rec_total_t1 = SteadyClock::now();
  _last_timing.rec_total_ms = elapsedMs(rec_total_t0, rec_total_t1);

  return regions;
}

void BpuOcrPipeline::printModelInfo() const
{
  if (_det_model) {
    _det_model->printModelInfo();
  }

  _rec_model->printModelInfo();
}

const OcrTimingStats &BpuOcrPipeline::lastTiming() const
{
  return _last_timing;
}

BpuOcrPipeline::DetectionInput BpuOcrPipeline::prepareDetectionInput(const cv::Mat &bgr_image) const
{
  const hbDNNTensorShape &shape = _det_model->inputProperties().validShape;

  if (shape.numDimensions < 4) {
    throw std::runtime_error("detection model input shape is invalid");
  }

  const int det_input_height_px = shape.dimensionSize[kDetInputHeightIndex];
  const int det_input_width_px = shape.dimensionSize[kDetInputWidthIndex];

  const float scale = std::min(
      static_cast<float>(det_input_width_px) / static_cast<float>(bgr_image.cols),
      static_cast<float>(det_input_height_px) / static_cast<float>(bgr_image.rows));
  const int resized_width_px =
      std::max(1, static_cast<int>(std::round(static_cast<float>(bgr_image.cols) * scale)));
  const int resized_height_px =
      std::max(1, static_cast<int>(std::round(static_cast<float>(bgr_image.rows) * scale)));
  const int pad_x = (det_input_width_px - resized_width_px) / 2;
  const int pad_y = (det_input_height_px - resized_height_px) / 2;

  cv::Mat resized_image;
  cv::resize(
      bgr_image,
      resized_image,
      cv::Size(resized_width_px, resized_height_px),
      0.0,
      0.0,
      cv::INTER_LINEAR);

  cv::Mat letterbox_image(
      det_input_height_px,
      det_input_width_px,
      bgr_image.type(),
      cv::Scalar(114, 114, 114));
  resized_image.copyTo(
      letterbox_image(cv::Rect(pad_x, pad_y, resized_width_px, resized_height_px)));

  DetectionInput input{};
  input.nv12 = bgrToNv12(letterbox_image);
  input.scale = scale;
  input.pad_x = pad_x;
  input.pad_y = pad_y;
  input.model_width_px = det_input_width_px;
  input.model_height_px = det_input_height_px;
  return input;
}

std::vector<BpuOcrPipeline::DetectionBox> BpuOcrPipeline::detectTextBoxes(
    const std::vector<std::vector<float>> &outputs,
    const cv::Size &image_size,
    const DetectionInput &detection_input) const
{
  if (outputs.empty()) {
    return {};
  }

  const hbDNNTensorShape &shape = _det_model->outputProperties()[0].validShape;
  const auto [output_height_px, output_width_px] = resolveSingleChannelMapShape(shape);

  cv::Mat score_map(
      output_height_px,
      output_width_px,
      CV_32FC1,
      const_cast<float *>(outputs[0].data()));

  cv::Mat binary_mask;
  cv::threshold(score_map, binary_mask, _config.det_threshold, 255.0, cv::THRESH_BINARY);
  binary_mask.convertTo(binary_mask, CV_8UC1);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<DetectionBox> boxes;
  boxes.reserve(contours.size());
  const float output_to_model_x =
      static_cast<float>(detection_input.model_width_px) / static_cast<float>(output_width_px);
  const float output_to_model_y =
      static_cast<float>(detection_input.model_height_px) / static_cast<float>(output_height_px);
  const float output_to_model_scale = (output_to_model_x + output_to_model_y) * 0.5F;

  for (const std::vector<cv::Point> &contour : contours) {
    if (cv::contourArea(contour) < static_cast<double>(_config.det_min_area_px)) {
      continue;
    }

    const float unclip_distance_output =
        contourUnclipDistance(contour, _config.det_unclip_ratio);
    const float unclip_distance_image =
        unclip_distance_output * output_to_model_scale / detection_input.scale;

    std::vector<cv::Point2f> mapped_contour;
    mapped_contour.reserve(contour.size());

    for (const cv::Point &point : contour) {
      const float x =
          (static_cast<float>(point.x) * output_to_model_x - static_cast<float>(detection_input.pad_x)) /
          detection_input.scale;
      const float y =
          (static_cast<float>(point.y) * output_to_model_y - static_cast<float>(detection_input.pad_y)) /
          detection_input.scale;
      mapped_contour.emplace_back(
          clampFloat(x, 0.0F, static_cast<float>(image_size.width - 1)),
          clampFloat(y, 0.0F, static_cast<float>(image_size.height - 1)));
    }

    const cv::RotatedRect rect = cv::minAreaRect(mapped_contour);
    cv::RotatedRect unclipped_rect = rect;
    unclipped_rect.size.width += unclip_distance_image * 2.0F;
    unclipped_rect.size.height += unclip_distance_image * 2.0F;

    if (unclipped_rect.size.width <= 2.0F || unclipped_rect.size.height <= 2.0F) {
      continue;
    }

    boxes.push_back({rectToOrderedPoints(unclipped_rect)});
  }

  std::sort(
      boxes.begin(),
      boxes.end(),
      [](const DetectionBox &a, const DetectionBox &b) {
        const float ay = a.points[0].y + a.points[1].y + a.points[2].y + a.points[3].y;
        const float by = b.points[0].y + b.points[1].y + b.points[2].y + b.points[3].y;
        return ay < by;
      });

  return boxes;
}

BpuOcrPipeline::RecognitionResult BpuOcrPipeline::recognizeText(const cv::Mat &crop_bgr)
{
  const auto rec_preprocess_t0 = SteadyClock::now();
  const float height_scale =
      static_cast<float>(_config.rec_input_height_px) / static_cast<float>(crop_bgr.rows);
  const int resized_width_px = std::clamp(
      static_cast<int>(std::ceil(static_cast<float>(crop_bgr.cols) * height_scale)),
      1,
      _config.rec_input_width_px);

  cv::Mat resized_image;
  cv::resize(
      crop_bgr,
      resized_image,
      cv::Size(resized_width_px, _config.rec_input_height_px),
      0.0,
      0.0,
      cv::INTER_LINEAR);

  cv::Mat padded_image(
      _config.rec_input_height_px,
      _config.rec_input_width_px,
      crop_bgr.type(),
      cv::Scalar(0, 0, 0));
  resized_image.copyTo(
      padded_image(cv::Rect(0, 0, resized_width_px, _config.rec_input_height_px)));

  padded_image.convertTo(padded_image, CV_32FC3, 1.0 / 255.0);
  cv::cvtColor(padded_image, padded_image, cv::COLOR_BGR2RGB);

  std::vector<float> nchw_input(
      static_cast<std::size_t>(_config.rec_input_height_px) *
      static_cast<std::size_t>(_config.rec_input_width_px) * 3U);

  for (int y = 0; y < _config.rec_input_height_px; ++y) {
    for (int x = 0; x < _config.rec_input_width_px; ++x) {
      const cv::Vec3f pixel = padded_image.at<cv::Vec3f>(y, x);
      const std::size_t plane_offset =
          static_cast<std::size_t>(y) * static_cast<std::size_t>(_config.rec_input_width_px) +
          static_cast<std::size_t>(x);
      const std::size_t plane_size =
          static_cast<std::size_t>(_config.rec_input_height_px) *
          static_cast<std::size_t>(_config.rec_input_width_px);
      nchw_input[plane_offset] = pixel[0];
      nchw_input[plane_size + plane_offset] = pixel[1];
      nchw_input[plane_size * 2U + plane_offset] = pixel[2];
    }
  }
  const auto rec_preprocess_t1 = SteadyClock::now();
  _last_timing.rec_preprocess_ms += elapsedMs(rec_preprocess_t0, rec_preprocess_t1);

  const auto rec_infer_t0 = SteadyClock::now();
  const std::vector<std::vector<float>> rec_outputs =
      _rec_model->infer(
      nchw_input.data(),
      nchw_input.size() * sizeof(float));
  const auto rec_infer_t1 = SteadyClock::now();
  _last_timing.rec_infer_ms += elapsedMs(rec_infer_t0, rec_infer_t1);

  if (rec_outputs.empty()) {
    return {};
  }

  const auto rec_decode_t0 = SteadyClock::now();
  const hbDNNTensorShape &shape = _rec_model->outputProperties()[0].validShape;
  const auto [time_steps, class_count] = resolveSequenceShape(shape);
  float mean_score = 0.0F;
  const std::string text = normalizeShelfCode(decodeCtcGreedy(
      rec_outputs[0].data(),
      time_steps,
      class_count,
      &mean_score));
  const auto rec_decode_t1 = SteadyClock::now();
  _last_timing.rec_decode_ms += elapsedMs(rec_decode_t0, rec_decode_t1);

  if (!isValidShelfCode(text)) {
    return {};
  }

  return {text, mean_score};
}

cv::Mat BpuOcrPipeline::cropAndRectify(const cv::Mat &image, const DetectionBox &box) const
{
  const float top_width = cv::norm(box.points[1] - box.points[0]);
  const float bottom_width = cv::norm(box.points[2] - box.points[3]);
  const float left_height = cv::norm(box.points[3] - box.points[0]);
  const float right_height = cv::norm(box.points[2] - box.points[1]);

  const int crop_width_px = std::max(1, static_cast<int>(std::round(std::max(top_width, bottom_width))));
  const int crop_height_px = std::max(1, static_cast<int>(std::round(std::max(left_height, right_height))));

  const std::array<cv::Point2f, 4> source_points = box.points;
  const std::array<cv::Point2f, 4> target_points = {
      cv::Point2f(0.0F, 0.0F),
      cv::Point2f(static_cast<float>(crop_width_px - 1), 0.0F),
      cv::Point2f(static_cast<float>(crop_width_px - 1), static_cast<float>(crop_height_px - 1)),
      cv::Point2f(0.0F, static_cast<float>(crop_height_px - 1))};

  const std::vector<cv::Point2f> source_points_vec(source_points.begin(), source_points.end());
  const std::vector<cv::Point2f> target_points_vec(target_points.begin(), target_points.end());
  const cv::Mat transform = cv::getPerspectiveTransform(source_points_vec, target_points_vec);

  cv::Mat warped_image;
  cv::warpPerspective(
      image,
      warped_image,
      transform,
      cv::Size(crop_width_px, crop_height_px),
      cv::INTER_LINEAR,
      cv::BORDER_REPLICATE);

  if (warped_image.cols < warped_image.rows) {
    cv::rotate(warped_image, warped_image, cv::ROTATE_90_CLOCKWISE);
  }

  return warped_image;
}

std::string BpuOcrPipeline::decodeCtcGreedy(
    const float *logits,
    int time_steps,
    int class_count,
    float *mean_score)
{
  if (logits == nullptr || time_steps <= 0 || class_count <= 1) {
    if (mean_score != nullptr) {
      *mean_score = 0.0F;
    }

    return {};
  }

  std::string decoded_text;
  float score_sum = 0.0F;
  int score_count = 0;
  int last_index = -1;

  for (int t = 0; t < time_steps; ++t) {
    const float *step_logits = logits + static_cast<std::size_t>(t) * static_cast<std::size_t>(class_count);
    int best_index = 0;
    float best_value = step_logits[0];

    for (int c = 1; c < class_count; ++c) {
      if (step_logits[c] > best_value) {
        best_value = step_logits[c];
        best_index = c;
      }
    }

    if (best_index == 0 || best_index == last_index) {
      last_index = best_index;
      continue;
    }

    const int alphabet_index = best_index - 1;

    if (alphabet_index >= 0 &&
        alphabet_index < static_cast<int>(sizeof(kCtcAlphabet)) - 1) {
      decoded_text.push_back(kCtcAlphabet[alphabet_index]);
      score_sum += best_value;
      ++score_count;
    }

    last_index = best_index;
  }

  if (mean_score != nullptr) {
    *mean_score = score_count > 0 ? score_sum / static_cast<float>(score_count) : 0.0F;
  }

  return decoded_text;
}

std::string BpuOcrPipeline::normalizeShelfCode(const std::string &text)
{
  std::string normalized;
  normalized.reserve(text.size());

  for (char value : text) {
    if (std::isspace(static_cast<unsigned char>(value)) != 0) {
      continue;
    }

    if (value == '_') {
      normalized.push_back('-');
      continue;
    }

    normalized.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(value))));
  }

  if (normalized.size() >= 2U && (normalized[0] == '4' || normalized[0] == '8')) {
    normalized[0] = normalized[0] == '4' ? 'A' : 'B';
  }

  for (std::size_t i = 0; i < normalized.size(); ++i) {
    if (normalized[i] == '=') {
      normalized[i] = '-';
      continue;
    }

    if (i == 0U) {
      continue;
    }

    if (normalized[i] == 'O') {
      normalized[i] = '0';
    } else if (normalized[i] == 'I' || normalized[i] == 'L') {
      normalized[i] = '1';
    }
  }

  return normalized;
}

bool BpuOcrPipeline::isValidShelfCode(const std::string &text)
{
  if (text.size() < 5U) {
    return false;
  }

  if (text[0] != 'A' && text[0] != 'B') {
    return false;
  }

  const std::size_t first_dash = text.find('-');
  const std::size_t second_dash = text.find('-', first_dash == std::string::npos ? 0U : first_dash + 1U);

  if (first_dash != 1U || second_dash == std::string::npos) {
    return false;
  }

  if (text.find('-', second_dash + 1U) != std::string::npos) {
    return false;
  }

  const std::string middle = text.substr(first_dash + 1U, second_dash - first_dash - 1U);
  const std::string tail = text.substr(second_dash + 1U);

  if (middle.empty() || tail.empty()) {
    return false;
  }

  return std::all_of(middle.begin(), middle.end(), isDigitChar) &&
      std::all_of(tail.begin(), tail.end(), isDigitChar);
}

std::string BpuOcrPipeline::defaultDetectionModelPath()
{
  static constexpr const char *kCandidatePaths[] = {
      "/home/sunrise/drone_ws/src/drone_perception/en_PP-OCRv3_det_640x640_nv12.bin",
      "/home/gjl/drone_ws/src/drone_perception/en_PP-OCRv3_det_640x640_nv12.bin",
      "/home/sunrise/rdk_model_zoo/samples/vision/PaddleOCR/model/en_PP-OCRv3_det_640x640_nv12.bin",
      "/home/gjl/rdk_model_zoo/samples/vision/PaddleOCR/model/en_PP-OCRv3_det_640x640_nv12.bin"};

  for (const char *path : kCandidatePaths) {
    if (std::filesystem::exists(path)) {
      return path;
    }
  }

  return kCandidatePaths[0];
}

std::string BpuOcrPipeline::defaultRecognitionModelPath()
{
  static constexpr const char *kCandidatePaths[] = {
      "/home/sunrise/drone_ws/src/drone_perception/en_PP-OCRv3_rec_48x320_rgb.bin",
      "/home/gjl/drone_ws/src/drone_perception/en_PP-OCRv3_rec_48x320_rgb.bin",
      "/home/sunrise/rdk_model_zoo/samples/vision/PaddleOCR/model/en_PP-OCRv3_rec_48x320_rgb.bin",
      "/home/gjl/rdk_model_zoo/samples/vision/PaddleOCR/model/en_PP-OCRv3_rec_48x320_rgb.bin"};

  for (const char *path : kCandidatePaths) {
    if (std::filesystem::exists(path)) {
      return path;
    }
  }

  return kCandidatePaths[0];
}
