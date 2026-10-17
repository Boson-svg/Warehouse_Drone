cmake_minimum_required(VERSION 3.8)
project(drone_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)

pkg_check_modules(ZBAR REQUIRED zbar)

# ── k230_tcp_bridge ──────────────────────────────────────
add_executable(k230_tcp_bridge
  src/k230_tcp_bridge.cpp
)

target_compile_features(k230_tcp_bridge PUBLIC cxx_std_17)

target_include_directories(k230_tcp_bridge
  PRIVATE
  include
)

ament_target_dependencies(k230_tcp_bridge
  rclcpp
  std_msgs
  drone_msgs
)

target_link_libraries(k230_tcp_bridge Threads::Threads)

install(TARGETS
  k230_tcp_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# ── k230_animals_uart_ros2_node ──────────────────────────
add_executable(k230_animals_uart_ros2_node
  src/k230_animals_uart_ros2_node.cpp
)

target_compile_features(k230_animals_uart_ros2_node PUBLIC cxx_std_17)

target_include_directories(k230_animals_uart_ros2_node
  PRIVATE
  include
)

ament_target_dependencies(k230_animals_uart_ros2_node
  rclcpp
  std_msgs
  drone_msgs
  geometry_msgs
)

install(TARGETS
  k230_animals_uart_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/k230_animals_image_saver.sh
  scripts/k230_snapshot_decode.sh
  DESTINATION lib/${PROJECT_NAME}
)

# ── qr_vision_node ───────────────────────────────────────
add_executable(qr_vision_node
  src/main.cpp
  src/qr_vision_node.cpp
)

target_compile_features(qr_vision_node PUBLIC cxx_std_17)

target_include_directories(qr_vision_node
  PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

target_link_libraries(qr_vision_node
  ${OpenCV_LIBS}
  ${ZBAR_LIBRARIES}
  Threads::Threads
)

target_link_directories(qr_vision_node
  PRIVATE
  ${ZBAR_LIBRARY_DIRS}
)

target_compile_options(qr_vision_node
  PRIVATE
  ${ZBAR_CFLAGS_OTHER}
)

ament_target_dependencies(qr_vision_node
  cv_bridge
  drone_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  qr_vision_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── optional D435i direct capture (librealsense2) ────────
# Prefer the CMake package; fall back to explicit include/lib search because
# realsense2.pc on the target has been observed pointing at the wrong arch.
set(REALSENSE2_FOUND_ANY FALSE)
find_package(realsense2 QUIET)
if(realsense2_FOUND)
  set(REALSENSE2_FOUND_ANY TRUE)
else()
  find_path(REALSENSE2_INCLUDE_DIR
    NAMES librealsense2/rs.hpp
    PATHS /opt/ros/humble/include
  )
  find_library(REALSENSE2_LIBRARY
    NAMES realsense2
    PATHS /opt/ros/humble/lib/aarch64-linux-gnu /opt/ros/humble/lib
  )
  if(REALSENSE2_INCLUDE_DIR AND REALSENSE2_LIBRARY)
    set(REALSENSE2_FOUND_ANY TRUE)
  endif()
endif()

if(REALSENSE2_FOUND_ANY)
  target_sources(qr_vision_node
    PRIVATE
    src/d435_color_capture.cpp
  )

  target_compile_definitions(qr_vision_node
    PRIVATE
    DRONE_PERCEPTION_HAS_REALSENSE=1
  )

  if(realsense2_FOUND)
    target_link_libraries(qr_vision_node realsense2::realsense2)
  else()
    target_include_directories(qr_vision_node
      PRIVATE
      ${REALSENSE2_INCLUDE_DIR}
    )
    target_link_libraries(qr_vision_node ${REALSENSE2_LIBRARY})
  endif()

  message(STATUS
    "drone_perception: librealsense2 found, d435_direct mode enabled")
else()
  message(STATUS
    "drone_perception: librealsense2 NOT found, d435_direct mode disabled "
    "(camera_input_mode=ros only)")
endif()

# ── optional RDK X5 Nano2D hardware preprocessor ────────
# Board-only vendor system libraries (/usr/hobot/lib); no rosdep key.
find_path(NANO2D_INCLUDE_DIR
  NAMES GC820/nano2D.h
  PATHS /usr/include
)

find_library(NANO2D_LIBRARY
  NAMES Nano2D
  PATHS /usr/hobot/lib
)

find_library(NANO2D_UTIL_LIBRARY
  NAMES Nano2Dutil
  PATHS /usr/hobot/lib
)

if(NANO2D_INCLUDE_DIR AND NANO2D_LIBRARY AND NANO2D_UTIL_LIBRARY)
  target_sources(qr_vision_node
    PRIVATE
    src/nano2d_preprocessor.cpp
  )

  target_include_directories(qr_vision_node
    PRIVATE
    ${NANO2D_INCLUDE_DIR}
  )

  target_link_libraries(qr_vision_node
    ${NANO2D_LIBRARY}
    ${NANO2D_UTIL_LIBRARY}
  )

  target_compile_definitions(qr_vision_node
    PRIVATE
    DRONE_PERCEPTION_HAS_NANO2D=1
  )

  add_executable(nano2d_preprocess_probe
    src/nano2d_preprocess_probe.cpp
    src/nano2d_preprocessor.cpp
  )

  target_compile_features(nano2d_preprocess_probe PUBLIC cxx_std_17)

  target_include_directories(nano2d_preprocess_probe
    PRIVATE
    include
    ${NANO2D_INCLUDE_DIR}
  )

  target_link_libraries(nano2d_preprocess_probe
    ${NANO2D_LIBRARY}
    ${NANO2D_UTIL_LIBRARY}
  )

  install(TARGETS
    nano2d_preprocess_probe
    DESTINATION lib/${PROJECT_NAME}
  )

  message(STATUS
    "drone_perception: Nano2D found, hardware preprocess enabled")
else()
  message(STATUS
    "drone_perception: Nano2D NOT found, hardware preprocess disabled "
    "(camera_input_mode=ros only)")
endif()

# ── optional RDK X5 BPU model probe ──────────────────────
find_path(RDK_DNN_INCLUDE_DIR
  NAMES dnn/hb_dnn.h
  PATHS /usr/include
)

find_library(RDK_DNN_LIBRARY
  NAMES dnn
  PATHS /usr/lib /usr/lib/aarch64-linux-gnu
)

if(RDK_DNN_INCLUDE_DIR AND RDK_DNN_LIBRARY)
  target_sources(qr_vision_node
    PRIVATE
    src/bpu_yolo_detector.cpp
    src/bpu_ocr_pipeline.cpp
  )

  target_include_directories(qr_vision_node
    PRIVATE
    ${RDK_DNN_INCLUDE_DIR}
  )

  target_link_libraries(qr_vision_node
    ${RDK_DNN_LIBRARY}
  )

  target_compile_definitions(qr_vision_node
    PRIVATE
    DRONE_PERCEPTION_HAS_BPU=1
  )

  add_executable(bpu_model_probe
    src/bpu_model_probe.cpp
    src/bpu_yolo_detector.cpp
  )

  target_compile_features(bpu_model_probe PUBLIC cxx_std_17)

  target_include_directories(bpu_model_probe
    PRIVATE
    include
    ${RDK_DNN_INCLUDE_DIR}
  )

  target_link_libraries(bpu_model_probe
    ${RDK_DNN_LIBRARY}
  )

  install(TARGETS
    bpu_model_probe
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  target_compile_definitions(qr_vision_node
    PRIVATE
    DRONE_PERCEPTION_HAS_BPU=0
  )

  message(STATUS
    "Skipping bpu_model_probe because RDK BPU SDK is unavailable. "
    "Need dnn/hb_dnn.h and libdnn.so.")
endif()

# ── optional RDK X5 D435i H.264 sender ───────────────────
# Standalone V4L2 YUYV -> hb_media_codec H.264 -> MediaMTX/RTSP sender.
# Conditional so non-RDK workstations and the RK3588 target can still build
# the package. The sender is only the T0-T3 link-test entry; the formal
# deployment keeps qr_vision_node as the single D435i opener (see
# rdk_x5_orangepi5ultra_send_receive_implementation_plan.md).
find_path(RDK_MEDIA_CODEC_INCLUDE_DIR NAMES hb_media_codec.h PATHS /usr/include)
find_library(RDK_MULTIMEDIA_LIBRARY NAMES multimedia PATHS /usr/hobot/lib /usr/lib)
pkg_check_modules(RDK_FFMPEG QUIET IMPORTED_TARGET libswscale libavutil)

if(RDK_MEDIA_CODEC_INCLUDE_DIR AND RDK_MULTIMEDIA_LIBRARY AND RDK_FFMPEG_FOUND)
  add_executable(rdk_d435i_h264_sender src/rdk_d435i_h264_sender.cpp)
  target_compile_features(rdk_d435i_h264_sender PUBLIC cxx_std_17)
  target_include_directories(rdk_d435i_h264_sender PRIVATE
    ${RDK_MEDIA_CODEC_INCLUDE_DIR}
  )
  target_link_libraries(rdk_d435i_h264_sender
    ${RDK_MULTIMEDIA_LIBRARY}
    PkgConfig::RDK_FFMPEG
  )
  install(TARGETS rdk_d435i_h264_sender DESTINATION lib/${PROJECT_NAME})

  # qr_vision_node reuses the same hb_media_codec encoder for its video-stream
  # worker. Input comes from the D435ColorFrame already captured by the node
  # (single camera opener), never from a second camera open.
  target_sources(qr_vision_node
    PRIVATE
    src/rdk_h264_encoder.cpp
  )
  target_compile_definitions(qr_vision_node
    PRIVATE
    DRONE_PERCEPTION_HAS_RDK_MEDIA_CODEC=1
  )
  target_include_directories(qr_vision_node
    PRIVATE
    ${RDK_MEDIA_CODEC_INCLUDE_DIR}
  )
  target_link_libraries(qr_vision_node
    ${RDK_MULTIMEDIA_LIBRARY}
    PkgConfig::RDK_FFMPEG
  )

  message(STATUS
    "drone_perception: RDK hb_media_codec found, H.264 sender + "
    "qr_vision_node video-stream worker enabled")
else()
  message(STATUS
    "Skipping rdk_d435i_h264_sender: need RDK hb_media_codec/libmultimedia "
    "and libswscale/libavutil development files.")
endif()

# ── optional OrangePi RTSP MPP probe ─────────────────────
# Non-Qt RTSP -> mppvideodec -> appsink probe for OrangePi 5 Ultra
# (T4-T7 receive-path validation). Built only where GStreamer dev modules are
# present (OrangePi); skipped elsewhere so RDK/PC builds are unaffected.
pkg_check_modules(GSTREAMER_PROBE QUIET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

if(GSTREAMER_PROBE_FOUND)
  add_executable(orangepi_rtsp_mpp_probe src/orangepi_rtsp_mpp_probe.cpp)
  target_compile_features(orangepi_rtsp_mpp_probe PUBLIC cxx_std_17)
  target_include_directories(orangepi_rtsp_mpp_probe PRIVATE
    ${GSTREAMER_PROBE_INCLUDE_DIRS}
  )
  target_compile_options(orangepi_rtsp_mpp_probe PRIVATE
    ${GSTREAMER_PROBE_CFLAGS_OTHER}
  )
  target_link_libraries(orangepi_rtsp_mpp_probe
    ${GSTREAMER_PROBE_LIBRARIES}
  )
  install(TARGETS orangepi_rtsp_mpp_probe DESTINATION lib/${PROJECT_NAME})
  message(STATUS
    "drone_perception: GStreamer found, orangepi_rtsp_mpp_probe enabled")
else()
  message(STATUS
    "Skipping orangepi_rtsp_mpp_probe: GStreamer dev modules not found.")
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/rdk_d435i_mediamtx_sender.sh
  scripts/start_rdk_d435i_sender.sh
  scripts/rdk_rtsp_link_sender_test.sh
  scripts/orangepi_rtsp_mpp_probe_run.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
