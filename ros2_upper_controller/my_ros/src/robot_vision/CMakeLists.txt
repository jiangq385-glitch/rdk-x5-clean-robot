cmake_minimum_required(VERSION 3.8)
project(robot_vision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_path(HB_DNN_INCLUDE_DIR
  NAMES dnn/hb_dnn.h
  PATHS /usr/include /opt/tros/humble/include
  NO_DEFAULT_PATH)

find_library(HB_DNN_LIBRARY
  NAMES dnn
  PATHS /lib /usr/lib /usr/lib/aarch64-linux-gnu /opt/tros/humble/lib
  NO_DEFAULT_PATH)

if(HB_DNN_INCLUDE_DIR AND HB_DNN_LIBRARY)
  add_compile_definitions(X5_VISION_HAS_RDK_DNN)
  include_directories(${HB_DNN_INCLUDE_DIR})
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(usb_camera_node
  robot_vision/usb_camera_node.cpp
)
ament_target_dependencies(usb_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(usb_camera_node ${OpenCV_LIBS})

add_executable(yolo_detector_node
  robot_vision/yolo_detector_node.cpp
  robot_vision/vision_utils.cpp
  robot_vision/dummy_backend.cpp
  robot_vision/rdk_yolo_backend_stub.cpp
)
ament_target_dependencies(yolo_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)
target_link_libraries(yolo_detector_node ${OpenCV_LIBS})

if(HB_DNN_LIBRARY)
  target_link_libraries(yolo_detector_node ${HB_DNN_LIBRARY})
endif()

install(TARGETS
  usb_camera_node
  yolo_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS robot_vision/camera_info_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_info_publisher
)

install(PROGRAMS robot_vision/laser_range_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME laser_range_node
)

install(PROGRAMS robot_vision/object_pose_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_pose_node
)

install(PROGRAMS robot_vision/object_pose_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_on_plane_node
)

install(PROGRAMS robot_vision/plane_calibration_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plane_calibration_node
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
