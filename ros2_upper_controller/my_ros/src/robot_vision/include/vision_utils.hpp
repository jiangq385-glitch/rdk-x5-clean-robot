#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace x5_vision
{

struct Detection
{
  int class_id{0};
  std::string class_name;
  float score{0.0F};
  cv::Rect2f bbox;
};

struct PreprocessContext
{
  float scale_x{1.0F};
  float scale_y{1.0F};
  int pad_left{0};
  int pad_top{0};
  int input_width{0};
  int input_height{0};
  int source_width{0};
  int source_height{0};
  bool letterbox{true};
};

cv::Mat preprocess_for_yolo(
  const cv::Mat & image,
  int input_width,
  int input_height,
  bool letterbox,
  int pad_value,
  PreprocessContext & context);

cv::Mat bgr_to_nv12(const cv::Mat & bgr_image);

cv::Rect2f restore_bbox_to_source(
  const cv::Rect2f & bbox,
  const PreprocessContext & context);

std::string detections_to_json(
  const std::vector<Detection> & detections,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & backend_name,
  bool backend_ready,
  double latency_ms = -1.0);

void draw_detections(
  cv::Mat & image,
  const std::vector<Detection> & detections);

}  // namespace x5_vision
