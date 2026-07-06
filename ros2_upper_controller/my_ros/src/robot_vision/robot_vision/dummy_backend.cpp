#include "detector_backend.hpp"

#include <memory>
#include <string>
#include <vector>

namespace x5_vision
{

class DummyBackend final : public DetectorBackend
{
public:
  bool initialize(const std::string & model_path) override
  {
    model_path_ = model_path;
    ready_ = false;
    return true;
  }

  bool is_ready() const override
  {
    return ready_;
  }

  std::string name() const override
  {
    return "dummy_backend";
  }

  std::vector<Detection> infer(
    const cv::Mat &,
    const cv::Mat &,
    const cv::Mat &,
    const PreprocessContext &,
    const std::vector<std::string> &,
    float,
    float) override
  {
    return {};
  }

private:
  std::string model_path_;
  bool ready_{false};
};

std::unique_ptr<DetectorBackend> make_dummy_backend()
{
  return std::make_unique<DummyBackend>();
}

}  // namespace x5_vision
