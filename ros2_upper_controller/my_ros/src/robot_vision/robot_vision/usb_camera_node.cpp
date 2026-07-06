#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

class UsbCameraNode : public rclcpp::Node
{
public:
  UsbCameraNode()
  : Node("usb_camera_node")
  {
    device_path_ = this->declare_parameter<std::string>("device_path", "");
    camera_index_ = this->declare_parameter<int>("camera_index", 0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_link");
    output_topic_ = this->declare_parameter<std::string>("output_topic", "/camera/image_raw");
    width_ = this->declare_parameter<int>("width", 640);
    height_ = this->declare_parameter<int>("height", 480);
    fps_ = this->declare_parameter<int>("fps", 30);
    use_mjpeg_ = this->declare_parameter<bool>("use_mjpeg", true);
    reopen_on_failure_ = this->declare_parameter<bool>("reopen_on_failure", true);
    reopen_delay_ms_ = this->declare_parameter<int>("reopen_delay_ms", 1000);

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);

    if (!open_camera()) {
      throw std::runtime_error("failed to open USB camera");
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(1, fps_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&UsbCameraNode::capture_once, this));

    RCLCPP_INFO(
      this->get_logger(),
      "USB camera started: source=%s, %dx%d @ %d FPS, topic=%s",
      camera_source_description().c_str(), width_, height_, fps_, output_topic_.c_str());
  }

private:
  bool open_camera()
  {
    if (cap_.isOpened()) {
      cap_.release();
    }

    const bool opened = device_path_.empty() ?
      cap_.open(camera_index_, cv::CAP_V4L2) :
      cap_.open(device_path_, cv::CAP_V4L2);

    if (!opened || !cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open camera source: %s", camera_source_description().c_str());
      return false;
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    if (use_mjpeg_) {
      cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    }

    return true;
  }

  void capture_once()
  {
    cv::Mat frame;
    if (!cap_.isOpened() || !cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to read one frame from %s",
        camera_source_description().c_str());
      try_reopen_camera();
      return;
    }

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = frame_id_;
    publisher_->publish(*msg);
  }

  std::string camera_source_description() const
  {
    if (!device_path_.empty()) {
      return device_path_;
    }
    return "/dev/video" + std::to_string(camera_index_);
  }

  void try_reopen_camera()
  {
    if (!reopen_on_failure_) {
      return;
    }

    const auto now = this->now();
    if (last_reopen_attempt_.nanoseconds() != 0 &&
      (now - last_reopen_attempt_).nanoseconds() < static_cast<int64_t>(reopen_delay_ms_) * 1000000LL)
    {
      return;
    }

    last_reopen_attempt_ = now;
    RCLCPP_WARN(this->get_logger(), "Trying to reopen camera source: %s", camera_source_description().c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(std::max(0, reopen_delay_ms_)));
    open_camera();
  }

  std::string device_path_;
  int camera_index_{0};
  int width_{640};
  int height_{480};
  int fps_{30};
  bool use_mjpeg_{true};
  bool reopen_on_failure_{true};
  int reopen_delay_ms_{1000};
  std::string frame_id_;
  std::string output_topic_;

  cv::VideoCapture cap_;
  rclcpp::Time last_reopen_attempt_{0};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UsbCameraNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("usb_camera_node"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
