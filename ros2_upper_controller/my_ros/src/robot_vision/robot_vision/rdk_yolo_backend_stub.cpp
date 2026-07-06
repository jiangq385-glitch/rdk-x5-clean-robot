#include "detector_backend.hpp"
#include "vision_utils.hpp"

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef X5_VISION_HAS_RDK_DNN
#include "dnn/hb_dnn.h"
#include "dnn/hb_dnn_ext.h"
#include "dnn/hb_sys.h"
#endif

namespace x5_vision
{

namespace
{

constexpr int kOutputCount = 6;
constexpr int kReg = 16;
constexpr int kNmsTopK = 300;

cv::Rect2f clamp_bbox(const cv::Rect2f & bbox, int image_width, int image_height)
{
  const float x1 = std::max(0.0f, std::min(bbox.x, static_cast<float>(image_width)));
  const float y1 = std::max(0.0f, std::min(bbox.y, static_cast<float>(image_height)));
  const float x2 = std::max(0.0f, std::min(bbox.x + bbox.width, static_cast<float>(image_width)));
  const float y2 = std::max(0.0f, std::min(bbox.y + bbox.height, static_cast<float>(image_height)));

  return cv::Rect2f(x1, y1, std::max(0.0f, x2 - x1), std::max(0.0f, y2 - y1));
}

std::string class_name_for(const std::vector<std::string> & class_names, int class_id)
{
  if (class_id >= 0 && class_id < static_cast<int>(class_names.size())) {
    return class_names[static_cast<std::size_t>(class_id)];
  }
  return "class_" + std::to_string(class_id);
}

float sigmoid(float value)
{
  return 1.0F / (1.0F + std::exp(-value));
}

float dfl_expectation(const float * logits, int bins)
{
  float max_logit = logits[0];
  for (int i = 1; i < bins; ++i) {
    max_logit = std::max(max_logit, logits[i]);
  }

  float sum = 0.0F;
  float weighted_sum = 0.0F;
  for (int i = 0; i < bins; ++i) {
    const float weight = std::exp(logits[i] - max_logit);
    sum += weight;
    weighted_sum += weight * static_cast<float>(i);
  }

  return (sum > 0.0F) ? (weighted_sum / sum) : 0.0F;
}

}  // namespace

#ifdef X5_VISION_HAS_RDK_DNN

namespace rdk_runtime
{

inline bool succeeded(int ret)
{
  return ret == 0;
}

void free_output_tensors(std::vector<hbDNNTensor> & outputs)
{
  for (auto & output : outputs) {
    if (output.sysMem[0].virAddr != nullptr) {
      hbSysFreeMem(&output.sysMem[0]);
      output.sysMem[0].virAddr = nullptr;
      output.sysMem[0].phyAddr = 0;
    }
  }
}

}  // namespace rdk_runtime

class RdkYoloBackend final : public DetectorBackend
{
public:
  explicit RdkYoloBackend(int num_classes)
  : num_classes_hint_(num_classes)
  {}

  ~RdkYoloBackend() override
  {
    cleanup();
  }

  bool initialize(const std::string & model_path) override
  {
    cleanup();
    model_path_ = model_path;

    if (model_path_.empty()) {
      RCLCPP_WARN(
        rclcpp::get_logger("rdk_yolo_backend"),
        "RDK backend disabled because model_path is empty");
      return false;
    }

    const char * model_file_name = model_path_.c_str();
    if (!rdk_runtime::succeeded(hbDNNInitializeFromFiles(&packed_dnn_handle_, &model_file_name, 1))) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rdk_yolo_backend"),
        "hbDNNInitializeFromFiles failed for model: %s",
        model_path_.c_str());
      cleanup();
      return false;
    }

    const char ** model_name_list = nullptr;
    int model_count = 0;
    if (!rdk_runtime::succeeded(hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle_)) ||
      model_count <= 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Failed to query model names");
      cleanup();
      return false;
    }

    model_name_ = model_name_list[0];
    if (!rdk_runtime::succeeded(hbDNNGetModelHandle(&dnn_handle_, packed_dnn_handle_, model_name_.c_str()))) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "hbDNNGetModelHandle failed");
      cleanup();
      return false;
    }

    int32_t input_count = 0;
    if (!rdk_runtime::succeeded(hbDNNGetInputCount(&input_count, dnn_handle_)) || input_count != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rdk_yolo_backend"),
        "Expected one input tensor but found %d",
        static_cast<int>(input_count));
      cleanup();
      return false;
    }

    if (!rdk_runtime::succeeded(hbDNNGetInputTensorProperties(&input_properties_, dnn_handle_, 0))) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Failed to query input tensor properties");
      cleanup();
      return false;
    }

    if (input_properties_.tensorType != HB_DNN_IMG_TYPE_NV12) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Model input tensor type is not NV12");
      cleanup();
      return false;
    }

    if (input_properties_.validShape.numDimensions != 4) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Model input shape is not 4D NCHW");
      cleanup();
      return false;
    }

    input_height_ = input_properties_.validShape.dimensionSize[2];
    input_width_ = input_properties_.validShape.dimensionSize[3];

    int32_t output_count = 0;
    if (!rdk_runtime::succeeded(hbDNNGetOutputCount(&output_count, dnn_handle_)) || output_count != kOutputCount) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rdk_yolo_backend"),
        "Expected %d outputs but found %d",
        kOutputCount,
        static_cast<int>(output_count));
      cleanup();
      return false;
    }

    if (!build_output_order()) {
      cleanup();
      return false;
    }

    if (num_classes_hint_ > 0 && num_classes_hint_ != num_classes_) {
      RCLCPP_WARN(
        rclcpp::get_logger("rdk_yolo_backend"),
        "Configured class_names count (%d) does not match model class count (%d); unknown labels will use class_N names",
        num_classes_hint_,
        num_classes_);
    }

    ready_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("rdk_yolo_backend"),
      "RDK backend ready: model=%s, input=%dx%d, classes=%d",
      model_path_.c_str(),
      input_width_,
      input_height_,
      num_classes_);
    return true;
  }

  bool is_ready() const override
  {
    return ready_;
  }

  std::string name() const override
  {
    return "rdk_yolo_backend";
  }

  std::vector<Detection> infer(
    const cv::Mat & original_bgr,
    const cv::Mat &,
    const cv::Mat & preprocessed_nv12,
    const PreprocessContext & context,
    const std::vector<std::string> & class_names,
    float score_threshold,
    float nms_threshold) override
  {
    if (!ready_) {
      return {};
    }

    if (preprocessed_nv12.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Preprocessed NV12 input is empty");
      return {};
    }

    const int expected_bytes = input_width_ * input_height_ * 3 / 2;
    if (preprocessed_nv12.total() != static_cast<std::size_t>(expected_bytes)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rdk_yolo_backend"),
        "NV12 input size mismatch: expected %d bytes, got %zu",
        expected_bytes,
        preprocessed_nv12.total());
      return {};
    }

    hbDNNTensor input{};
    input.properties = input_properties_;
    if (!rdk_runtime::succeeded(hbSysAllocCachedMem(&input.sysMem[0], expected_bytes))) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Failed to allocate input tensor memory");
      return {};
    }

    std::memcpy(input.sysMem[0].virAddr, preprocessed_nv12.data, static_cast<std::size_t>(expected_bytes));
    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

    std::vector<hbDNNTensor> outputs(kOutputCount);
    for (int i = 0; i < kOutputCount; ++i) {
      if (!rdk_runtime::succeeded(hbDNNGetOutputTensorProperties(&outputs[static_cast<std::size_t>(i)].properties, dnn_handle_, i))) {
        RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Failed to query output tensor properties");
        hbSysFreeMem(&input.sysMem[0]);
        rdk_runtime::free_output_tensors(outputs);
        return {};
      }
      if (!rdk_runtime::succeeded(
          hbSysAllocCachedMem(
            &outputs[static_cast<std::size_t>(i)].sysMem[0],
            outputs[static_cast<std::size_t>(i)].properties.alignedByteSize)))
      {
        RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Failed to allocate output tensor memory");
        hbSysFreeMem(&input.sysMem[0]);
        rdk_runtime::free_output_tensors(outputs);
        return {};
      }
    }

    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

    hbDNNTensor * output_ptr = outputs.data();
    int infer_ret = hbDNNInfer(&task_handle, &output_ptr, &input, dnn_handle_, &infer_ctrl_param);
    if (!rdk_runtime::succeeded(infer_ret)) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "hbDNNInfer failed with code %d", infer_ret);
      hbSysFreeMem(&input.sysMem[0]);
      rdk_runtime::free_output_tensors(outputs);
      return {};
    }

    int wait_ret = hbDNNWaitTaskDone(task_handle, 0);
    if (!rdk_runtime::succeeded(wait_ret)) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "hbDNNWaitTaskDone failed with code %d", wait_ret);
      hbDNNReleaseTask(task_handle);
      hbSysFreeMem(&input.sysMem[0]);
      rdk_runtime::free_output_tensors(outputs);
      return {};
    }

    std::vector<std::vector<cv::Rect2d>> bboxes(static_cast<std::size_t>(num_classes_));
    std::vector<std::vector<float>> scores(static_cast<std::size_t>(num_classes_));
    const float conf_threshold_raw = -std::log(1.0F / score_threshold - 1.0F);

    const bool small_ok = process_feature_map(outputs, order_[0], order_[1], 8, conf_threshold_raw, bboxes, scores);
    const bool medium_ok = process_feature_map(outputs, order_[2], order_[3], 16, conf_threshold_raw, bboxes, scores);
    const bool large_ok = process_feature_map(outputs, order_[4], order_[5], 32, conf_threshold_raw, bboxes, scores);

    hbDNNReleaseTask(task_handle);
    hbSysFreeMem(&input.sysMem[0]);
    rdk_runtime::free_output_tensors(outputs);

    if (!(small_ok && medium_ok && large_ok)) {
      return {};
    }

    std::vector<Detection> detections;
    for (int class_id = 0; class_id < num_classes_; ++class_id) {
      std::vector<int> indices;
      cv::dnn::NMSBoxes(
        bboxes[static_cast<std::size_t>(class_id)],
        scores[static_cast<std::size_t>(class_id)],
        score_threshold,
        nms_threshold,
        indices,
        1.0F,
        kNmsTopK);

      for (const int index : indices) {
        Detection detection;
        detection.class_id = class_id;
        detection.class_name = class_name_for(class_names, class_id);
        detection.score = scores[static_cast<std::size_t>(class_id)][static_cast<std::size_t>(index)];
        detection.bbox = clamp_bbox(
          restore_bbox_to_source(
            cv::Rect2f(
              static_cast<float>(bboxes[static_cast<std::size_t>(class_id)][static_cast<std::size_t>(index)].x),
              static_cast<float>(bboxes[static_cast<std::size_t>(class_id)][static_cast<std::size_t>(index)].y),
              static_cast<float>(bboxes[static_cast<std::size_t>(class_id)][static_cast<std::size_t>(index)].width),
              static_cast<float>(bboxes[static_cast<std::size_t>(class_id)][static_cast<std::size_t>(index)].height)),
            context),
          original_bgr.cols,
          original_bgr.rows);
        detections.push_back(detection);
      }
    }

    std::sort(
      detections.begin(), detections.end(),
      [](const Detection & a, const Detection & b) {
        return a.score > b.score;
      });

    return detections;
  }

  std::vector<Detection> infer_single_output(
    const cv::Mat & original_bgr,
    const cv::Mat & preprocessed_nv12,
    const PreprocessContext & context,
    const std::vector<std::string> & class_names,
    float score_threshold,
    float nms_threshold)
  {
    if (preprocessed_nv12.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Preprocessed NV12 input is empty");
      return {};
    }

    const int expected_bytes = input_width_ * input_height_ * 3 / 2;
    if (preprocessed_nv12.total() != static_cast<std::size_t>(expected_bytes)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rdk_yolo_backend"),
        "NV12 input size mismatch: expected %d bytes, got %zu",
        expected_bytes,
        preprocessed_nv12.total());
      return {};
    }

    hbDNNTensor input{};
    input.properties = input_properties_;
    if (!rdk_runtime::succeeded(hbSysAllocCachedMem(&input.sysMem[0], expected_bytes))) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Failed to allocate input tensor memory");
      return {};
    }

    std::memcpy(input.sysMem[0].virAddr, preprocessed_nv12.data, static_cast<std::size_t>(expected_bytes));
    hbSysFlushMem(&input.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);

    hbDNNTensor output{};
    if (!rdk_runtime::succeeded(hbDNNGetOutputTensorProperties(&output.properties, dnn_handle_, 0))) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Failed to query single output tensor properties");
      hbSysFreeMem(&input.sysMem[0]);
      return {};
    }

    if (!rdk_runtime::succeeded(hbSysAllocCachedMem(&output.sysMem[0], output.properties.alignedByteSize))) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Failed to allocate single output tensor memory");
      hbSysFreeMem(&input.sysMem[0]);
      return {};
    }

    hbDNNTaskHandle_t task_handle = nullptr;
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);

    hbDNNTensor * output_ptr = &output;
    int infer_ret = hbDNNInfer(&task_handle, &output_ptr, &input, dnn_handle_, &infer_ctrl_param);
    if (!rdk_runtime::succeeded(infer_ret)) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "hbDNNInfer failed with code %d", infer_ret);
      hbSysFreeMem(&input.sysMem[0]);
      std::vector<hbDNNTensor> cleanup_outputs{output};
      rdk_runtime::free_output_tensors(cleanup_outputs);
      return {};
    }

    int wait_ret = hbDNNWaitTaskDone(task_handle, 0);
    if (!rdk_runtime::succeeded(wait_ret)) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "hbDNNWaitTaskDone failed with code %d", wait_ret);
      hbDNNReleaseTask(task_handle);
      hbSysFreeMem(&input.sysMem[0]);
      std::vector<hbDNNTensor> cleanup_outputs{output};
      rdk_runtime::free_output_tensors(cleanup_outputs);
      return {};
    }

    const int configured_classes = std::max(1, static_cast<int>(class_names.size()));
    const int dim_count = output.properties.validShape.numDimensions;
    std::vector<int> dims;
    for (int i = 0; i < dim_count; ++i) {
      dims.push_back(output.properties.validShape.dimensionSize[i]);
    }

    int rows = 0;
    int values = 0;
    bool rows_major = true;
    const int candidates[] = {4 + configured_classes, 5 + configured_classes};
    for (const int candidate : candidates) {
      for (std::size_t i = 0; i < dims.size(); ++i) {
        if (dims[i] != candidate) {
          continue;
        }
        for (std::size_t j = 0; j < dims.size(); ++j) {
          if (i == j || dims[j] <= 1) {
            continue;
          }
          rows = dims[j];
          values = candidate;
          rows_major = (j < i);
          break;
        }
        if (rows > 0) {
          break;
        }
      }
      if (rows > 0) {
        break;
      }
    }

    if (rows <= 0 || values <= 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rdk_yolo_backend"),
        "Unsupported single-output shape; dims=%d [%d, %d, %d, %d]",
        dim_count,
        dim_count > 0 ? dims[0] : 0,
        dim_count > 1 ? dims[1] : 0,
        dim_count > 2 ? dims[2] : 0,
        dim_count > 3 ? dims[3] : 0);
      hbDNNReleaseTask(task_handle);
      hbSysFreeMem(&input.sysMem[0]);
      std::vector<hbDNNTensor> cleanup_outputs{output};
      rdk_runtime::free_output_tensors(cleanup_outputs);
      return {};
    }

    hbSysFlushMem(&output.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    auto * raw = reinterpret_cast<float *>(output.sysMem[0].virAddr);

    auto value_at = [&](int row, int col) -> float {
      return rows_major ? raw[row * values + col] : raw[col * rows + row];
    };

    std::vector<std::vector<cv::Rect2d>> bboxes(static_cast<std::size_t>(configured_classes));
    std::vector<std::vector<float>> scores(static_cast<std::size_t>(configured_classes));

    for (int row = 0; row < rows; ++row) {
      const float x = value_at(row, 0);
      const float y = value_at(row, 1);
      const float w = value_at(row, 2);
      const float h = value_at(row, 3);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(w) || !std::isfinite(h) || w <= 0.0F || h <= 0.0F) {
        continue;
      }

      float objectness = 1.0F;
      const int class_start = (values == 5 + configured_classes) ? 5 : 4;
      if (values == 5 + configured_classes) {
        const float raw_objectness = value_at(row, 4);
        objectness = (raw_objectness < 0.0F || raw_objectness > 1.0F) ? sigmoid(raw_objectness) : raw_objectness;
      }

      int class_id = 0;
      float class_score = value_at(row, class_start);
      for (int cls = 1; cls < configured_classes; ++cls) {
        const float candidate = value_at(row, class_start + cls);
        if (candidate > class_score) {
          class_score = candidate;
          class_id = cls;
        }
      }

      class_score = (class_score < 0.0F || class_score > 1.0F) ? sigmoid(class_score) : class_score;
      const float score = objectness * class_score;
      if (score < score_threshold) {
        continue;
      }

      float x1 = x;
      float y1 = y;
      float bw = w;
      float bh = h;
      if (x1 <= 1.5F && y1 <= 1.5F && bw <= 2.0F && bh <= 2.0F) {
        x1 *= static_cast<float>(input_width_);
        y1 *= static_cast<float>(input_height_);
        bw *= static_cast<float>(input_width_);
        bh *= static_cast<float>(input_height_);
      }

      bboxes[static_cast<std::size_t>(class_id)].push_back(cv::Rect2d(x1 - bw * 0.5F, y1 - bh * 0.5F, bw, bh));
      scores[static_cast<std::size_t>(class_id)].push_back(score);
    }

    hbDNNReleaseTask(task_handle);
    hbSysFreeMem(&input.sysMem[0]);
    std::vector<hbDNNTensor> cleanup_outputs{output};
    rdk_runtime::free_output_tensors(cleanup_outputs);

    std::vector<Detection> detections;
    for (int class_id = 0; class_id < configured_classes; ++class_id) {
      std::vector<int> indices;
      cv::dnn::NMSBoxes(
        bboxes[static_cast<std::size_t>(class_id)],
        scores[static_cast<std::size_t>(class_id)],
        score_threshold,
        nms_threshold,
        indices,
        1.0F,
        kNmsTopK);

      for (const int index : indices) {
        Detection detection;
        detection.class_id = class_id;
        detection.class_name = class_name_for(class_names, class_id);
        detection.score = scores[static_cast<std::size_t>(class_id)][static_cast<std::size_t>(index)];
        detection.bbox = clamp_bbox(
          restore_bbox_to_source(
            cv::Rect2f(
              static_cast<float>(bboxes[static_cast<std::size_t>(class_id)][static_cast<std::size_t>(index)].x),
              static_cast<float>(bboxes[static_cast<std::size_t>(class_id)][static_cast<std::size_t>(index)].y),
              static_cast<float>(bboxes[static_cast<std::size_t>(class_id)][static_cast<std::size_t>(index)].width),
              static_cast<float>(bboxes[static_cast<std::size_t>(class_id)][static_cast<std::size_t>(index)].height)),
            context),
          original_bgr.cols,
          original_bgr.rows);
        detections.push_back(detection);
      }
    }

    std::sort(
      detections.begin(), detections.end(),
      [](const Detection & a, const Detection & b) {
        return a.score > b.score;
      });

    return detections;
  }

private:
  bool build_output_order()
  {
    int inferred_class_count = 0;
    for (int actual_index = 0; actual_index < kOutputCount; ++actual_index) {
      hbDNNTensorProperties properties;
      if (!rdk_runtime::succeeded(hbDNNGetOutputTensorProperties(&properties, dnn_handle_, actual_index))) {
        return false;
      }
      const int channels =
        (properties.tensorLayout == HB_DNN_LAYOUT_NHWC) ?
        properties.validShape.dimensionSize[3] :
        properties.validShape.dimensionSize[1];
      if (channels != 4 * kReg) {
        inferred_class_count = channels;
        break;
      }
    }

    if (inferred_class_count <= 0) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Failed to infer model class count from outputs");
      return false;
    }

    num_classes_ = inferred_class_count;
    const int h8 = input_height_ / 8;
    const int h16 = input_height_ / 16;
    const int h32 = input_height_ / 32;
    const int w8 = input_width_ / 8;
    const int w16 = input_width_ / 16;
    const int w32 = input_width_ / 32;

    const int wanted[kOutputCount][3] = {
      {h8, w8, num_classes_},
      {h8, w8, 4 * kReg},
      {h16, w16, num_classes_},
      {h16, w16, 4 * kReg},
      {h32, w32, num_classes_},
      {h32, w32, 4 * kReg}
    };

    std::array<bool, kOutputCount> used{};
    for (int want_index = 0; want_index < kOutputCount; ++want_index) {
      bool found = false;
      for (int actual_index = 0; actual_index < kOutputCount; ++actual_index) {
        if (used[static_cast<std::size_t>(actual_index)]) {
          continue;
        }
        hbDNNTensorProperties properties;
        if (!rdk_runtime::succeeded(hbDNNGetOutputTensorProperties(&properties, dnn_handle_, actual_index))) {
          return false;
        }
        const int c_nchw = properties.validShape.dimensionSize[1];
        const int h_nchw = properties.validShape.dimensionSize[2];
        const int w_nchw = properties.validShape.dimensionSize[3];
        const int h_nhwc = properties.validShape.dimensionSize[1];
        const int w_nhwc = properties.validShape.dimensionSize[2];
        const int c_nhwc = properties.validShape.dimensionSize[3];
        if (
          (h_nchw == wanted[want_index][0] && w_nchw == wanted[want_index][1] && c_nchw == wanted[want_index][2]) ||
          (h_nhwc == wanted[want_index][0] && w_nhwc == wanted[want_index][1] && c_nhwc == wanted[want_index][2]))
        {
          order_[static_cast<std::size_t>(want_index)] = actual_index;
          used[static_cast<std::size_t>(actual_index)] = true;
          found = true;
          break;
        }
      }
      if (!found) {
        RCLCPP_ERROR(
          rclcpp::get_logger("rdk_yolo_backend"),
          "Failed to map output tensor order for wanted slot %d",
          want_index);
        return false;
      }
    }

    return true;
  }

  bool process_feature_map(
    std::vector<hbDNNTensor> & outputs,
    int cls_output_index,
    int bbox_output_index,
    int stride,
    float conf_threshold_raw,
    std::vector<std::vector<cv::Rect2d>> & bboxes,
    std::vector<std::vector<float>> & scores)
  {
    auto & cls_tensor = outputs[static_cast<std::size_t>(cls_output_index)];
    auto & bbox_tensor = outputs[static_cast<std::size_t>(bbox_output_index)];

    if (cls_tensor.properties.quantiType != NONE) {
      RCLCPP_ERROR(rclcpp::get_logger("rdk_yolo_backend"), "Class output quantiType must be NONE");
      return false;
    }
    if (bbox_tensor.properties.quantiType != SCALE && bbox_tensor.properties.quantiType != NONE) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rdk_yolo_backend"),
        "BBox output quantiType must be SCALE or NONE");
      return false;
    }

    hbSysFlushMem(&cls_tensor.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
    hbSysFlushMem(&bbox_tensor.sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);

    auto * cls_raw = reinterpret_cast<float *>(cls_tensor.sysMem[0].virAddr);
    auto * bbox_raw_i32 = reinterpret_cast<int32_t *>(bbox_tensor.sysMem[0].virAddr);
    auto * bbox_raw_f32 = reinterpret_cast<float *>(bbox_tensor.sysMem[0].virAddr);
    const float * bbox_scale =
      (bbox_tensor.properties.quantiType == SCALE) ?
      reinterpret_cast<float *>(bbox_tensor.properties.scale.scaleData) :
      nullptr;

    bool nhwc_output = false;
    int feature_c = cls_tensor.properties.validShape.dimensionSize[1];
    int feature_h = cls_tensor.properties.validShape.dimensionSize[2];
    int feature_w = cls_tensor.properties.validShape.dimensionSize[3];
    if (feature_c != num_classes_ && cls_tensor.properties.validShape.dimensionSize[3] == num_classes_) {
      nhwc_output = true;
      feature_h = cls_tensor.properties.validShape.dimensionSize[1];
      feature_w = cls_tensor.properties.validShape.dimensionSize[2];
      feature_c = cls_tensor.properties.validShape.dimensionSize[3];
    }
    const int feature_spatial = feature_h * feature_w;

    if (feature_c != num_classes_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rdk_yolo_backend"),
        "Class output channel count mismatch: expected %d, got %d",
        num_classes_,
        feature_c);
      return false;
    }

    for (int h = 0; h < feature_h; ++h) {
      for (int w = 0; w < feature_w; ++w) {
        const int spatial_index = h * feature_w + w;
        int cls_id = 0;
        for (int i = 1; i < num_classes_; ++i) {
          const int cls_index = nhwc_output ? (spatial_index * num_classes_ + i) : (i * feature_spatial + spatial_index);
          const int best_index = nhwc_output ? (spatial_index * num_classes_ + cls_id) : (cls_id * feature_spatial + spatial_index);
          if (cls_raw[cls_index] > cls_raw[best_index]) {
            cls_id = i;
          }
        }

        const int cls_index = nhwc_output ? (spatial_index * num_classes_ + cls_id) : (cls_id * feature_spatial + spatial_index);
        if (cls_raw[cls_index] < conf_threshold_raw) {
          continue;
        }

        const float score = sigmoid(cls_raw[cls_index]);
        float ltrb[4] = {0.0F, 0.0F, 0.0F, 0.0F};
        for (int side = 0; side < 4; ++side) {
          float logits[kReg];
          for (int bucket = 0; bucket < kReg; ++bucket) {
            const int offset = side * kReg + bucket;
            const int tensor_index = nhwc_output ? (spatial_index * 4 * kReg + offset) : (offset * feature_spatial + spatial_index);
            logits[bucket] =
              (bbox_tensor.properties.quantiType == SCALE) ?
              static_cast<float>(bbox_raw_i32[tensor_index]) * bbox_scale[offset] :
              bbox_raw_f32[tensor_index];
          }
          ltrb[side] = dfl_expectation(logits, kReg);
        }

        if (ltrb[0] + ltrb[2] <= 0.0F || ltrb[1] + ltrb[3] <= 0.0F) {
          continue;
        }

        const float x1 = (static_cast<float>(w) + 0.5F - ltrb[0]) * static_cast<float>(stride);
        const float y1 = (static_cast<float>(h) + 0.5F - ltrb[1]) * static_cast<float>(stride);
        const float x2 = (static_cast<float>(w) + 0.5F + ltrb[2]) * static_cast<float>(stride);
        const float y2 = (static_cast<float>(h) + 0.5F + ltrb[3]) * static_cast<float>(stride);

        bboxes[static_cast<std::size_t>(cls_id)].push_back(
          cv::Rect2d(
            static_cast<double>(x1),
            static_cast<double>(y1),
            static_cast<double>(x2 - x1),
            static_cast<double>(y2 - y1)));
        scores[static_cast<std::size_t>(cls_id)].push_back(score);
      }
    }

    return true;
  }

  void cleanup()
  {
    ready_ = false;
    dnn_handle_ = nullptr;
    input_width_ = 0;
    input_height_ = 0;
    order_ = {0, 1, 2, 3, 4, 5};
    if (packed_dnn_handle_ != nullptr) {
      hbDNNRelease(packed_dnn_handle_);
      packed_dnn_handle_ = nullptr;
    }
  }

  std::string model_path_;
  std::string model_name_;
  bool ready_{false};
  int num_classes_hint_{1};
  int num_classes_{1};
  int input_width_{0};
  int input_height_{0};
  std::array<int, kOutputCount> order_{0, 1, 2, 3, 4, 5};
  hbPackedDNNHandle_t packed_dnn_handle_{nullptr};
  hbDNNHandle_t dnn_handle_{nullptr};
  hbDNNTensorProperties input_properties_{};
};

#else

class RdkYoloBackend final : public DetectorBackend
{
public:
  explicit RdkYoloBackend(int num_classes)
  : num_classes_hint_(num_classes)
  {}

  bool initialize(const std::string & model_path) override
  {
    model_path_ = model_path;
    ready_ = false;
    RCLCPP_WARN(
      rclcpp::get_logger("rdk_yolo_backend"),
      "RDK DNN headers/libraries were not found when building x5_vision; rebuild on the RDK X5 board to enable real inference");
    return false;
  }

  bool is_ready() const override
  {
    return ready_;
  }

  std::string name() const override
  {
    return "rdk_yolo_backend_unavailable";
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
  int num_classes_hint_{1};
};

#endif

std::unique_ptr<DetectorBackend> make_rdk_backend(int num_classes)
{
  return std::make_unique<RdkYoloBackend>(num_classes);
}

}  // namespace x5_vision
