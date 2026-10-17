#include "drone_perception/bpu_yolo_detector.hpp"

#include <exception>
#include <iostream>
#include <string>

int main()
{
  const std::string model_path =
      "/home/sunrise/drone_ws/src/drone_perception/new_640x640_nv12.bin";

  try {
    BpuYoloDetector detector(model_path);
    detector.printModelInfo();
  } catch (const std::exception &e) {
    std::cerr << "bpu_model_probe failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
