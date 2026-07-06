#include "vision_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace x5_vision
{

namespace
{

std::string escape_json(const std::string & value)
{
  std::ostringstream oss;
  for (const char c : value) {
    switch (c) {
      case '"': oss << "\\\""; break;
      case '\\': oss << "\\\\"; break;
      case '\n': oss << "\\n"; break;
      case '\r': oss << "\\r"; break;
      case '\t': oss << "\\t"; break;
      default: oss << c; break;
    }
  }
  return oss.str();
}

}  // namespace

cv::Mat preprocess_for_yolo(
  const cv::Mat & image,
  int input_width,
  int input_height,
  bool letterbox,
  int pad_value,
  PreprocessContext & context)
{
  if (image.empty()) {
    throw std::runtime_error("input image is empty");
  }

  context.source_width = image.cols;
  context.source_height = image.rows;
  context.input_width = input_width;
  context.input_height = input_height;
  context.letterbox = letterbox;

  cv::Mat output;

  if (letterbox) {
    const float scale = std::min(
      static_cast<float>(input_width) / static_cast<float>(image.cols),
      static_cast<float>(input_height) / static_cast<float>(image.rows));
    const int resized_width = std::max(1, static_cast<int>(std::round(image.cols * scale)));
    const int resized_height = std::max(1, static_cast<int>(std::round(image.rows * scale)));

    context.scale_x = scale;
    context.scale_y = scale;
    context.pad_left = (input_width - resized_width) / 2;
    context.pad_top = (input_height - resized_height) / 2;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(resized_width, resized_height));

    const int pad_right = input_width - resized_width - context.pad_left;
    const int pad_bottom = input_height - resized_height - context.pad_top;
    cv::copyMakeBorder(
      resized,
      output,
      context.pad_top,
      pad_bottom,
      context.pad_left,
      pad_right,
      cv::BORDER_CONSTANT,
      cv::Scalar(pad_value, pad_value, pad_value));
  } else {
    context.scale_x = static_cast<float>(input_width) / static_cast<float>(image.cols);
    context.scale_y = static_cast<float>(input_height) / static_cast<float>(image.rows);
    context.pad_left = 0;
    context.pad_top = 0;
    cv::resize(image, output, cv::Size(input_width, input_height));
  }

  return output;
}

cv::Mat bgr_to_nv12(const cv::Mat & bgr_image)
{
  if (bgr_image.empty()) {
    throw std::runtime_error("input image is empty");
  }

  if (bgr_image.cols % 2 != 0 || bgr_image.rows % 2 != 0) {
    throw std::runtime_error("NV12 conversion requires even image width and height");
  }

  cv::Mat yuv_i420;
  cv::cvtColor(bgr_image, yuv_i420, cv::COLOR_BGR2YUV_I420);

  const int y_size = bgr_image.cols * bgr_image.rows;
  const int uv_plane_size = y_size / 4;

  cv::Mat nv12(bgr_image.rows * 3 / 2, bgr_image.cols, CV_8UC1);
  std::memcpy(nv12.data, yuv_i420.data, y_size);

  const uint8_t * u_plane = yuv_i420.data + y_size;
  const uint8_t * v_plane = yuv_i420.data + y_size + uv_plane_size;
  uint8_t * uv_interleaved = nv12.data + y_size;

  for (int i = 0; i < uv_plane_size; ++i) {
    uv_interleaved[2 * i] = u_plane[i];
    uv_interleaved[2 * i + 1] = v_plane[i];
  }

  return nv12;
}

cv::Rect2f restore_bbox_to_source(
  const cv::Rect2f & bbox,
  const PreprocessContext & context)
{
  if (context.letterbox) {
    const float x = (bbox.x - static_cast<float>(context.pad_left)) / context.scale_x;
    const float y = (bbox.y - static_cast<float>(context.pad_top)) / context.scale_y;
    const float w = bbox.width / context.scale_x;
    const float h = bbox.height / context.scale_y;
    return cv::Rect2f(x, y, w, h);
  }

  const float x = bbox.x / context.scale_x;
  const float y = bbox.y / context.scale_y;
  const float w = bbox.width / context.scale_x;
  const float h = bbox.height / context.scale_y;
  return cv::Rect2f(x, y, w, h);
}

std::string detections_to_json(
  const std::vector<Detection> & detections,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const std::string & backend_name,
  bool backend_ready,
  double latency_ms)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  oss << "{";
  oss << "\"frame_id\":\"" << escape_json(frame_id) << "\",";
  oss << "\"stamp_ns\":" << stamp.nanoseconds() << ",";
  oss << "\"latency_ms\":" << latency_ms << ",";
  oss << "\"backend\":\"" << escape_json(backend_name) << "\",";
  oss << "\"backend_ready\":" << (backend_ready ? "true" : "false") << ",";
  oss << "\"detections\":[";

  for (std::size_t i = 0; i < detections.size(); ++i) {
    const auto & det = detections[i];
    oss << "{";
    oss << "\"class_id\":" << det.class_id << ",";
    oss << "\"class_name\":\"" << escape_json(det.class_name) << "\",";
    oss << "\"score\":" << det.score << ",";
    oss << "\"bbox\":{";
    oss << "\"x\":" << det.bbox.x << ",";
    oss << "\"y\":" << det.bbox.y << ",";
    oss << "\"w\":" << det.bbox.width << ",";
    oss << "\"h\":" << det.bbox.height;
    oss << "}";
    oss << "}";
    if (i + 1 < detections.size()) {
      oss << ",";
    }
  }

  oss << "]}";
  return oss.str();
}

void draw_detections(
  cv::Mat & image,
  const std::vector<Detection> & detections)
{
  for (const auto & det : detections) {
    const auto color = cv::Scalar(40, 180, 240);
    const cv::Rect rect(
      static_cast<int>(std::round(det.bbox.x)),
      static_cast<int>(std::round(det.bbox.y)),
      static_cast<int>(std::round(det.bbox.width)),
      static_cast<int>(std::round(det.bbox.height)));

    cv::rectangle(image, rect, color, 2);

    std::ostringstream label;
    label << det.class_name << " " << std::fixed << std::setprecision(2) << det.score;
    cv::putText(
      image,
      label.str(),
      cv::Point(rect.x, std::max(20, rect.y - 5)),
      cv::FONT_HERSHEY_SIMPLEX,
      0.6,
      cv::Scalar(0, 255, 0),
      2,
      cv::LINE_AA);
  }
}

}  // namespace x5_vision
