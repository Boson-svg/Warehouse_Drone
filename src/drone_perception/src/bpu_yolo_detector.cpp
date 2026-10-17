#include "drone_perception/bpu_yolo_detector.hpp"

#include <dnn/hb_sys.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace
{
using SteadyClock = std::chrono::steady_clock;

double elapsedMs(
    const SteadyClock::time_point &start,
    const SteadyClock::time_point &end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

static constexpr int32_t kSingleModelFileCount = 1;
static constexpr int32_t kMinModelCount = 1;

static constexpr int32_t kExpectedOutputCount = 6;
static constexpr int32_t kClassCount = 3;
static constexpr int32_t kDflBinCount = 16;
static constexpr int32_t kBoxSideCount = 4;
static constexpr int32_t kModelInputSizePx = 640;
static constexpr float kModelInputSizePxFloat = 640.0F;
static constexpr const char *kClassNames[] = {"qrcode", "package", "shelf_tag"};

struct BpuYoloOutputGroup
{
  int32_t cls_output_index;
  int32_t box_output_index;
  int32_t stride_px;
  int32_t grid_size;
};

static constexpr BpuYoloOutputGroup kOutputGroups[] = {
  {0, 1, 8, 80},
  {2, 3, 16, 40},
  {4, 5, 32, 20},
};

static constexpr float kScoreThreshold = 0.25F;
static constexpr float kNmsThreshold = 0.45F;

struct BpuYoloCandidate
{
  int32_t box_output_index;
  int32_t stride_px;
  int32_t grid_x;
  int32_t grid_y;
  int32_t class_id;
  float score;
};

float sigmoid(float value)
{
  return 1.0F / (1.0F + std::exp(-value));
}

float decodeDflDistance(const float *dfl_data)
{
  float max_value = dfl_data[0];

  for (int32_t i = 1; i < kDflBinCount; ++i) {
    if (dfl_data[i] > max_value) {
      max_value = dfl_data[i];
    }
  }

  float exp_sum = 0.0F;
  float weighted_sum = 0.0F;

  for (int32_t i = 0; i < kDflBinCount; ++i) {
    const float probability = std::exp(dfl_data[i] - max_value);

    exp_sum += probability;
    weighted_sum += static_cast<float>(i) * probability;
  }

  if (exp_sum <= 0.0F) {
    return 0.0F;
  }

  return weighted_sum / exp_sum;
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

BpuYoloDetection decodeCandidateBox(
    const BpuYoloCandidate &candidate,
    const std::vector<hbDNNTensor> &output_tensors)
{
  const hbDNNTensor &box_tensor =
      output_tensors[static_cast<std::size_t>(candidate.box_output_index)];
  const float *box_data = static_cast<const float *>(box_tensor.sysMem[0].virAddr);

  const int32_t grid_size = kModelInputSizePx / candidate.stride_px;
  const std::size_t grid_index =
      static_cast<std::size_t>(candidate.grid_y) *
          static_cast<std::size_t>(grid_size) +
      static_cast<std::size_t>(candidate.grid_x);

  const std::size_t cell_offset =
      grid_index * static_cast<std::size_t>(kBoxSideCount * kDflBinCount);

  const float left = decodeDflDistance(&box_data[cell_offset + 0 * kDflBinCount]);
  const float top = decodeDflDistance(&box_data[cell_offset + 1 * kDflBinCount]);
  const float right = decodeDflDistance(&box_data[cell_offset + 2 * kDflBinCount]);
  const float bottom = decodeDflDistance(&box_data[cell_offset + 3 * kDflBinCount]);

  const float stride_px = static_cast<float>(candidate.stride_px);
  const float center_x = (static_cast<float>(candidate.grid_x) + 0.5F) * stride_px;
  const float center_y = (static_cast<float>(candidate.grid_y) + 0.5F) * stride_px;

  BpuYoloDetection detection{};
  detection.x_min_px =
      clampFloat(center_x - left * stride_px, 0.0F, kModelInputSizePxFloat);
  detection.y_min_px =
      clampFloat(center_y - top * stride_px, 0.0F, kModelInputSizePxFloat);
  detection.x_max_px =
      clampFloat(center_x + right * stride_px, 0.0F, kModelInputSizePxFloat);
  detection.y_max_px =
      clampFloat(center_y + bottom * stride_px, 0.0F, kModelInputSizePxFloat);

  detection.score = candidate.score;
  detection.class_id = candidate.class_id;

  return detection;
}

std::vector<BpuYoloDetection> decodeCandidates(
    const std::vector<BpuYoloCandidate> &candidates,
    const std::vector<hbDNNTensor> &output_tensors)
{
  std::vector<BpuYoloDetection> detections;
  detections.reserve(candidates.size());

  for (const BpuYoloCandidate &candidate : candidates) {
    detections.push_back(decodeCandidateBox(candidate, output_tensors));
  }

  return detections;
}

float calculateBoxArea(const BpuYoloDetection &detection)
{
  const float width = detection.x_max_px - detection.x_min_px;
  const float height = detection.y_max_px - detection.y_min_px;

  if (width <= 0.0F || height <= 0.0F) {
    return 0.0F;
  }

  return width * height;
}

float calculateIou(const BpuYoloDetection &a, const BpuYoloDetection &b)
{
  const float inter_x_min = a.x_min_px > b.x_min_px ? a.x_min_px : b.x_min_px;
  const float inter_y_min = a.y_min_px > b.y_min_px ? a.y_min_px : b.y_min_px;
  const float inter_x_max = a.x_max_px < b.x_max_px ? a.x_max_px : b.x_max_px;
  const float inter_y_max = a.y_max_px < b.y_max_px ? a.y_max_px : b.y_max_px;

  const float inter_width = inter_x_max - inter_x_min;
  const float inter_height = inter_y_max - inter_y_min;

  if (inter_width <= 0.0F || inter_height <= 0.0F) {
    return 0.0F;
  }

  const float inter_area = inter_width * inter_height;
  const float union_area = calculateBoxArea(a) + calculateBoxArea(b) - inter_area;

  if (union_area <= 0.0F) {
    return 0.0F;
  }

  return inter_area / union_area;
}

std::vector<BpuYoloDetection> runNms(
    const std::vector<BpuYoloDetection> &detections)
{
  std::vector<BpuYoloDetection> sorted_detections = detections;

  std::sort(
      sorted_detections.begin(),
      sorted_detections.end(),
      [](const BpuYoloDetection &a, const BpuYoloDetection &b) {
        return a.score > b.score;
      });

  std::vector<BpuYoloDetection> kept_detections;
  std::vector<uint8_t> removed(sorted_detections.size(), 0U);

  for (std::size_t i = 0; i < sorted_detections.size(); ++i) {
    if (removed[i] != 0U) {
      continue;
    }

    kept_detections.push_back(sorted_detections[i]);

    for (std::size_t j = i + 1; j < sorted_detections.size(); ++j) {
      if (removed[j] != 0U) {
        continue;
      }

      if (sorted_detections[i].class_id != sorted_detections[j].class_id) {
        continue;
      }

      if (calculateIou(sorted_detections[i], sorted_detections[j]) > kNmsThreshold) {
        removed[j] = 1U;
      }
    }
  }

  return kept_detections;
}

std::vector<BpuYoloCandidate> collectCandidates(const std::vector<hbDNNTensor> &output_tensors)
{
  std::vector<BpuYoloCandidate> candidates;

  for (const BpuYoloOutputGroup &group : kOutputGroups) {
    const hbDNNTensor &cls_tensor =
        output_tensors[static_cast<std::size_t>(group.cls_output_index)];
    const float *cls_data = static_cast<const float *>(cls_tensor.sysMem[0].virAddr);

    for (int32_t grid_y = 0; grid_y < group.grid_size; ++grid_y) {
      for (int32_t grid_x = 0; grid_x < group.grid_size; ++grid_x) {
        const std::size_t grid_index =
            static_cast<std::size_t>(grid_y) *
                static_cast<std::size_t>(group.grid_size) +
            static_cast<std::size_t>(grid_x);
        const std::size_t cell_offset =
            grid_index * static_cast<std::size_t>(kClassCount);

        for (int32_t class_id = 0; class_id < kClassCount; ++class_id) {
          const float score =
              sigmoid(cls_data[cell_offset + static_cast<std::size_t>(class_id)]);

          if (score < kScoreThreshold) {
            continue;
          }

          candidates.push_back({
              group.box_output_index,
              group.stride_px,
              grid_x,
              grid_y,
              class_id,
              score});
        }
      }
    }
  }

  return candidates;
}

}  // namespace

BpuYoloDetector::BpuYoloDetector(const std::string &model_path)
{
  loadModel(model_path);
}

BpuYoloDetector::~BpuYoloDetector()
{
  releaseModel();
}

const char *BpuYoloDetector::className(int32_t class_id)
{
  if (class_id < 0 || class_id >= kClassCount) {
    return "unknown";
  }

  return kClassNames[static_cast<std::size_t>(class_id)];
}

const BpuYoloDetector::InferenceTimingStats &BpuYoloDetector::lastTiming() const
{
  return _last_timing;
}

void BpuYoloDetector::releaseModel()
{
  releaseTensors();

  if (_packed_dnn_handle == nullptr) {
    return;
  }

  const int ret = hbDNNRelease(_packed_dnn_handle);

  if (ret != 0) {
    std::cerr << "hbDNNRelease failed, ret=" << ret << std::endl;
  }

  _packed_dnn_handle = nullptr;
  _dnn_handle = nullptr;
  _input_count = 0;
  _output_count = 0;
  _model_name.clear();
  _input_properties = {};
  _output_properties.clear();
}

void BpuYoloDetector::releaseTensors()
{
  if (_input_tensor_allocated) {
    const int ret = hbSysFreeMem(&_input_tensor.sysMem[0]);

    if (ret != 0) {
      std::cerr << "hbSysFreeMem input failed, ret=" << ret << std::endl;
    }

    _input_tensor = {};
    _input_tensor_allocated = false;
  }

  for (std::size_t i = 0; i < _output_tensors.size(); ++i) {
    if (i >= _output_tensor_allocated.size() || _output_tensor_allocated[i] == 0U) {
      continue;
    }

    const int ret = hbSysFreeMem(&_output_tensors[i].sysMem[0]);

    if (ret != 0) {
      std::cerr << "hbSysFreeMem output[" << i << "] failed, ret=" << ret << std::endl;
    }
  }

  _output_tensors.clear();
  _output_tensor_allocated.clear();
}

void BpuYoloDetector::allocateTensors()
{
  releaseTensors();

  try {
    _input_tensor.properties = _input_properties;

    checkRet(
        hbSysAllocCachedMem(&_input_tensor.sysMem[0], _input_properties.alignedByteSize),
        "hbSysAllocCachedMem input");

    _input_tensor_allocated = true;

    _output_tensors.clear();
    _output_tensor_allocated.clear();

    _output_tensors.resize(static_cast<std::size_t>(_output_count));
    _output_tensor_allocated.resize(static_cast<std::size_t>(_output_count), 0U);

    for (int32_t i = 0; i < _output_count; ++i) {
      const std::size_t index = static_cast<std::size_t>(i);
      hbDNNTensor &tensor = _output_tensors[index];
      const hbDNNTensorProperties &properties = _output_properties[index];

      tensor.properties = properties;

      checkRet(
          hbSysAllocCachedMem(&tensor.sysMem[0], properties.alignedByteSize),
          "hbSysAllocCachedMem output[" + std::to_string(i) + "]");

      _output_tensor_allocated[index] = 1U;
    }
  } catch (...) {
    releaseTensors();
    throw;
  }
}

std::vector<BpuYoloDetection> BpuYoloDetector::inferNv12(
    const uint8_t *nv12_data,
    std::size_t nv12_size)
{
  _last_timing = {};
  InferenceTimingStats timing{};
  const auto total_t0 = SteadyClock::now();

  if (_dnn_handle == nullptr) {
    throw std::runtime_error("BPU model is not loaded");
  }

  if (!_input_tensor_allocated || _output_tensors.empty()) {
    throw std::runtime_error("BPU tensors are not allocated");
  }

  if (nv12_data == nullptr) {
    throw std::runtime_error("NV12 input data is null");
  }

  const std::size_t expected_size =
      static_cast<std::size_t>(_input_properties.alignedByteSize);

  if (nv12_size != expected_size) {
    throw std::runtime_error(
        "unexpected NV12 input size, expected=" + std::to_string(expected_size) +
        ", actual=" + std::to_string(nv12_size));
  }

  const auto input_memcpy_t0 = SteadyClock::now();
  std::memcpy(_input_tensor.sysMem[0].virAddr, nv12_data, nv12_size);
  const auto input_memcpy_t1 = SteadyClock::now();
  timing.input_memcpy_ms = elapsedMs(input_memcpy_t0, input_memcpy_t1);

  const auto input_cache_t0 = SteadyClock::now();
  checkRet(
      hbSysFlushMem(&_input_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN),
      "hbSysFlushMem input clean");
  const auto input_cache_t1 = SteadyClock::now();
  timing.input_cache_clean_ms = elapsedMs(input_cache_t0, input_cache_t1);

  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

  hbDNNTensor *input_tensor = &_input_tensor;
  hbDNNTensor *output_tensors = _output_tensors.data();

  const auto submit_t0 = SteadyClock::now();
  checkRet(
      hbDNNInfer(&task_handle, &output_tensors, input_tensor, _dnn_handle, &infer_ctrl_param),
      "hbDNNInfer");
  const auto submit_t1 = SteadyClock::now();
  timing.submit_ms = elapsedMs(submit_t0, submit_t1);

  const auto wait_t0 = SteadyClock::now();
  const int wait_ret = hbDNNWaitTaskDone(task_handle, 0);
  const auto wait_t1 = SteadyClock::now();
  timing.wait_ms = elapsedMs(wait_t0, wait_t1);

  const auto release_t0 = SteadyClock::now();
  const int release_ret = hbDNNReleaseTask(task_handle);
  const auto release_t1 = SteadyClock::now();
  timing.release_ms = elapsedMs(release_t0, release_t1);

  checkRet(wait_ret, "hbDNNWaitTaskDone");
  checkRet(release_ret, "hbDNNReleaseTask");

  const auto output_invalidate_t0 = SteadyClock::now();
  for (std::size_t i = 0; i < _output_tensors.size(); ++i) {
    checkRet(
        hbSysFlushMem(&_output_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE),
        "hbSysFlushMem output[" + std::to_string(i) + "] invalidate");
  }
  const auto output_invalidate_t1 = SteadyClock::now();
  timing.output_invalidate_ms = elapsedMs(output_invalidate_t0, output_invalidate_t1);

  const auto candidate_t0 = SteadyClock::now();
  const std::vector<BpuYoloCandidate> candidates = collectCandidates(_output_tensors);
  const auto candidate_t1 = SteadyClock::now();
  timing.candidate_ms = elapsedMs(candidate_t0, candidate_t1);
  timing.raw_candidate_count = candidates.size();

  const auto dfl_t0 = SteadyClock::now();
  std::vector<BpuYoloDetection> detections =
      decodeCandidates(candidates, _output_tensors);
  const auto dfl_t1 = SteadyClock::now();
  timing.dfl_decode_ms = elapsedMs(dfl_t0, dfl_t1);
  timing.decoded_detection_count = detections.size();

  const auto nms_t0 = SteadyClock::now();
  std::vector<BpuYoloDetection> final_detections = runNms(detections);
  const auto nms_t1 = SteadyClock::now();
  timing.nms_ms = elapsedMs(nms_t0, nms_t1);
  timing.final_detection_count = final_detections.size();
  timing.total_ms = elapsedMs(total_t0, nms_t1);
  _last_timing = timing;

  return final_detections;
}

void BpuYoloDetector::checkRet(int ret, const std::string &step)
{
  if (ret == 0) {
    return;
  }

  throw std::runtime_error(step + " failed, ret=" + std::to_string(ret));
}

void BpuYoloDetector::printShape(const hbDNNTensorShape &shape)
{
  std::cout << "[";

  for (int32_t i = 0; i < shape.numDimensions; ++i) {
    std::cout << shape.dimensionSize[i];

    if (i + 1 < shape.numDimensions) {
      std::cout << ", ";
    }
  }

  std::cout << "]";
}

void BpuYoloDetector::loadModel(const std::string &model_path)
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

void BpuYoloDetector::queryTensorProperties()
{
  if (_dnn_handle == nullptr) {
    throw std::runtime_error("BPU model is not loaded");
  }

  if (_input_count != 1) {
    throw std::runtime_error("unsupported input count: " + std::to_string(_input_count));
  }

  checkRet(
      hbDNNGetInputTensorProperties(&_input_properties, _dnn_handle, 0),
      "hbDNNGetInputTensorProperties");

  _output_properties.clear();
  _output_properties.resize(static_cast<std::size_t>(_output_count));

  for (int32_t i = 0; i < _output_count; ++i) {
    hbDNNTensorProperties &properties = _output_properties[static_cast<std::size_t>(i)];

    checkRet(
        hbDNNGetOutputTensorProperties(&properties, _dnn_handle, i),
        "hbDNNGetOutputTensorProperties");
  }
  validateOutputTensors();
}

void BpuYoloDetector::validateOutputTensors() const
{
  if (_output_count != kExpectedOutputCount) {
    throw std::runtime_error(
        "unsupported output count: " + std::to_string(_output_count));
  }

  if (_output_properties.size() != static_cast<std::size_t>(kExpectedOutputCount)) {
    throw std::runtime_error("output properties are not ready");
  }

  const auto validate_shape =
      [](const hbDNNTensorProperties &properties,
         int32_t grid_size,
         int32_t channel_count,
         const std::string &name) {
        const hbDNNTensorShape &shape = properties.validShape;

        if (shape.numDimensions != 4 ||
            shape.dimensionSize[0] != 1 ||
            shape.dimensionSize[1] != grid_size ||
            shape.dimensionSize[2] != grid_size ||
            shape.dimensionSize[3] != channel_count) {
          throw std::runtime_error(name + " has unexpected shape");
        }
      };

  for (const BpuYoloOutputGroup &group : kOutputGroups) {
    validate_shape(
        _output_properties[static_cast<std::size_t>(group.cls_output_index)],
        group.grid_size,
        kClassCount,
        "cls output[" + std::to_string(group.cls_output_index) + "]");

    validate_shape(
        _output_properties[static_cast<std::size_t>(group.box_output_index)],
        group.grid_size,
        kDflBinCount * 4,
        "box output[" + std::to_string(group.box_output_index) + "]");
  }
}

void BpuYoloDetector::printModelInfo() const
{
  if (_dnn_handle == nullptr) {
    throw std::runtime_error("BPU model is not loaded");
  }

  std::cout << "model name: " << _model_name << std::endl;
  std::cout << "input count: " << _input_count << std::endl;
  std::cout << "output count: " << _output_count << std::endl;

  std::cout << "input[0] valid_shape=";
  printShape(_input_properties.validShape);
  std::cout << " aligned_shape=";
  printShape(_input_properties.alignedShape);
  std::cout << std::endl;

  for (int32_t i = 0; i < _output_count; ++i) {
    const hbDNNTensorProperties &properties =
        _output_properties[static_cast<std::size_t>(i)];

    std::cout << "output[" << i << "] valid_shape=";
    printShape(properties.validShape);
    std::cout << " aligned_shape=";
    printShape(properties.alignedShape);
    std::cout << std::endl;
  }
}
