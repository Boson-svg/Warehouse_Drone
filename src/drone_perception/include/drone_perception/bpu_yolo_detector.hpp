#pragma once

#include <dnn/hb_dnn.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct BpuYoloDetection
{
  float x_min_px{0.0F};
  float y_min_px{0.0F};
  float x_max_px{0.0F};
  float y_max_px{0.0F};
  float score{0.0F};
  int32_t class_id{0};
};

class BpuYoloDetector
{
public:
  struct InferenceTimingStats
  {
    double input_memcpy_ms{0.0};
    double input_cache_clean_ms{0.0};
    double submit_ms{0.0};
    double wait_ms{0.0};
    double release_ms{0.0};
    double output_invalidate_ms{0.0};
    double candidate_ms{0.0};
    double dfl_decode_ms{0.0};
    double nms_ms{0.0};
    double total_ms{0.0};
    std::size_t raw_candidate_count{0U};
    std::size_t decoded_detection_count{0U};
    std::size_t final_detection_count{0U};
  };

  explicit BpuYoloDetector(const std::string &model_path);
  ~BpuYoloDetector();

  BpuYoloDetector(const BpuYoloDetector &) = delete;
  BpuYoloDetector &operator=(const BpuYoloDetector &) = delete;

  static const char *className(int32_t class_id);

  void printModelInfo() const;
  const InferenceTimingStats &lastTiming() const;
  std::vector<BpuYoloDetection> inferNv12(
      const uint8_t *nv12_data,
      std::size_t nv12_size);

private:
  void loadModel(const std::string &model_path);
  void releaseModel();
  void queryTensorProperties();
  void validateOutputTensors() const;
  void allocateTensors();
  void releaseTensors();

  static void checkRet(int ret, const std::string &step);
  static void printShape(const hbDNNTensorShape &shape);

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
  InferenceTimingStats _last_timing{};
};
