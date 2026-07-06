#include "detector_backend.hpp"
#include "vision_utils.hpp"

#include <atomic>
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/opencv.hpp>

#include <condition_variable>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace x5_vision
{

class YoloDetectorNode : public rclcpp::Node
{
public:
  YoloDetectorNode()
  : Node("yolo_detector_node")
  {
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    detections_topic_ = this->declare_parameter<std::string>("detections_topic", "/vision/detections_json");
    annotated_topic_ = this->declare_parameter<std::string>("annotated_topic", "/vision/image_annotated");
    backend_type_ = this->declare_parameter<std::string>("backend_type", "dummy");
    model_path_ = this->declare_parameter<std::string>("model_path", "");
    input_width_ = this->declare_parameter<int>("input_width", 640);
    input_height_ = this->declare_parameter<int>("input_height", 640);
    pad_value_ = this->declare_parameter<int>("pad_value", 114);
    use_letterbox_ = this->declare_parameter<bool>("use_letterbox", true);
    score_threshold_ = this->declare_parameter<double>("score_threshold", 0.25);
    nms_threshold_ = this->declare_parameter<double>("nms_threshold", 0.45);
    publish_annotated_ = this->declare_parameter<bool>("publish_annotated", true);
    inference_period_ms_ = this->declare_parameter<int>("inference_period_ms", 60);
    class_names_ = this->declare_parameter<std::vector<std::string>>(
      "class_names", std::vector<std::string>{"target"});

    if (inference_period_ms_ <= 0) {
      inference_period_ms_ = 60;
    }

    backend_ = create_backend(backend_type_);
    const bool backend_initialized = backend_->initialize(model_path_);

    detections_publisher_ = this->create_publisher<std_msgs::msg::String>(detections_topic_, 10);

    auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    annotated_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(annotated_topic_, image_qos);

    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      image_qos,
      std::bind(&YoloDetectorNode::image_callback, this, std::placeholders::_1));

    inference_thread_ = std::thread(&YoloDetectorNode::inference_worker_loop, this);

    RCLCPP_INFO(
      this->get_logger(),
      "Detector node ready: backend=%s, model=%s, image_topic=%s, inference_period_ms=%d",
      backend_->name().c_str(), model_path_.c_str(), image_topic_.c_str(), inference_period_ms_);
    if (!backend_initialized || !backend_->is_ready()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Detector backend is not ready yet. Set backend_type=rdk and provide a valid .bin model path on the RDK X5 board.");
    }
  }

  ~YoloDetectorNode() override
  {
    stop_worker_.store(true);
    worker_cv_.notify_all();
    if (inference_thread_.joinable()) {
      inference_thread_.join();
    }
  }

private:
  std::unique_ptr<DetectorBackend> create_backend(const std::string & backend_type)
  {
    if (backend_type == "rdk" || backend_type == "rdk_stub") {
      return make_rdk_backend(static_cast<int>(class_names_.size()));
    }
    return make_dummy_backend();
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (publish_annotated_) {
      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      } catch (const cv_bridge::Exception & ex) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", ex.what());
      }

      if (cv_ptr) {
        cv::Mat annotated = cv_ptr->image.clone();
        double detection_age_ms = -1.0;
        {
          std::lock_guard<std::mutex> lock(detections_mutex_);
          draw_detections(annotated, latest_detections_);
          if (latest_detection_stamp_.nanoseconds() > 0) {
            detection_age_ms = (this->now() - latest_detection_stamp_).seconds() * 1000.0;
          }
        }
        if (detection_age_ms >= 0.0) {
          std::ostringstream age_label;
          age_label << "det age " << std::fixed << std::setprecision(0) << detection_age_ms << " ms";
          cv::putText(
            annotated,
            age_label.str(),
            cv::Point(12, 28),
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            cv::Scalar(0, 255, 255),
            2,
            cv::LINE_AA);
        }
        auto annotated_msg = cv_bridge::CvImage(msg->header, "bgr8", annotated).toImageMsg();
        annotated_publisher_->publish(*annotated_msg);
      }
    }

    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      latest_frame_ = msg;
      latest_frame_sequence_++;
      new_frame_available_ = true;
    }

    worker_cv_.notify_one();
  }

  void inference_worker_loop()
  {
    const auto period = std::chrono::milliseconds(inference_period_ms_);
    auto next_run = std::chrono::steady_clock::now() + period;

    const auto advance_next_run = [&]() {
      const auto now = std::chrono::steady_clock::now();
      do {
        next_run += period;
      } while (next_run <= now);
    };

    std::unique_lock<std::mutex> lock(frame_mutex_);
    while (!stop_worker_.load()) {
      worker_cv_.wait_until(lock, next_run, [this]() {
        return stop_worker_.load();
      });

      if (stop_worker_.load()) {
        break;
      }

      if (!latest_frame_ || latest_frame_sequence_ == last_processed_sequence_) {
        new_frame_available_ = false;
        advance_next_run();
        continue;
      }

      auto frame_msg = latest_frame_;
      const std::uint64_t frame_sequence = latest_frame_sequence_;
      new_frame_available_ = false;
      lock.unlock();

      process_frame(frame_msg);

      lock.lock();
      last_processed_sequence_ = frame_sequence;
      advance_next_run();
    }
  }

  void process_frame(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", ex.what());
      return;
    }

    PreprocessContext preprocess_context;
    cv::Mat preprocessed_bgr;
    cv::Mat preprocessed_nv12;

    try {
      preprocessed_bgr = preprocess_for_yolo(
        cv_ptr->image,
        input_width_,
        input_height_,
        use_letterbox_,
        pad_value_,
        preprocess_context);
      preprocessed_nv12 = bgr_to_nv12(preprocessed_bgr);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Preprocess failed: %s", ex.what());
      return;
    }

    auto detections = backend_->infer(
      cv_ptr->image,
      preprocessed_bgr,
      preprocessed_nv12,
      preprocess_context,
      class_names_,
      static_cast<float>(score_threshold_),
      static_cast<float>(nms_threshold_));

    std_msgs::msg::String detections_msg;
    const auto publish_time = this->now();
    const double latency_ms = (publish_time - rclcpp::Time(msg->header.stamp)).seconds() * 1000.0;
    detections_msg.data = detections_to_json(
      detections,
      msg->header.frame_id,
      rclcpp::Time(msg->header.stamp),
      backend_->name(),
      backend_->is_ready(),
      latency_ms);
    detections_publisher_->publish(detections_msg);

    {
      std::lock_guard<std::mutex> lock(detections_mutex_);
      latest_detections_ = std::move(detections);
      latest_frame_id_ = msg->header.frame_id;
      latest_detection_stamp_ = publish_time;
      latest_detection_latency_ms_ = latency_ms;
    }
  }

  std::string image_topic_;
  std::string detections_topic_;
  std::string annotated_topic_;
  std::string backend_type_;
  std::string model_path_;
  int input_width_{640};
  int input_height_{640};
  int pad_value_{114};
  bool use_letterbox_{true};
  double score_threshold_{0.25};
  double nms_threshold_{0.45};
  bool publish_annotated_{true};
  int inference_period_ms_{60};
  std::vector<std::string> class_names_;

  std::unique_ptr<DetectorBackend> backend_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_publisher_;

  std::mutex frame_mutex_;
  std::mutex detections_mutex_;
  std::condition_variable worker_cv_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_frame_;
  std::vector<Detection> latest_detections_;
  std::string latest_frame_id_;
  rclcpp::Time latest_detection_stamp_{0, 0, RCL_ROS_TIME};
  double latest_detection_latency_ms_{-1.0};
  std::uint64_t latest_frame_sequence_{0};
  std::uint64_t last_processed_sequence_{0};
  bool new_frame_available_{false};
  std::atomic<bool> stop_worker_{false};
  std::thread inference_thread_;
};

}  // namespace x5_vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<x5_vision::YoloDetectorNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("yolo_detector_node"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
