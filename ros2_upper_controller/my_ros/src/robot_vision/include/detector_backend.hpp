#pragma once

#include "vision_utils.hpp"

#include <memory>
#include <string>
#include <vector>

namespace x5_vision
{

class DetectorBackend
{
public:
  virtual ~DetectorBackend() = default;
  virtual bool initialize(const std::string & model_path) = 0;
  virtual bool is_ready() const = 0;
  virtual std::string name() const = 0;
  virtual std::vector<Detection> infer(
    const cv::Mat & original_bgr,
    const cv::Mat & preprocessed_bgr,
    const cv::Mat & preprocessed_nv12,
    const PreprocessContext & context,
    const std::vector<std::string> & class_names,
    float score_threshold,
    float nms_threshold) = 0;
};

std::unique_ptr<DetectorBackend> make_dummy_backend();
std::unique_ptr<DetectorBackend> make_rdk_backend(int num_classes);

}  // namespace x5_vision
