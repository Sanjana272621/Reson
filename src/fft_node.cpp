// fft_node
//
// Subscribes to /reson/vibration_raw (std_msgs/Float64, one scalar sample
// per message), accumulates samples into fixed-size windows, and publishes
// the single-sided magnitude spectrum of each window on
// /reson/fft_spectrum as a std_msgs/Float32MultiArray.
//
// Wire format of the published array (all fields required, in this order):
//   data[0] = freq_resolution_hz   (Hz per FFT bin)
//   data[1] = sample_rate_hz       (estimated or configured)
//   data[2] = std_dev              (time-domain std dev of the raw window)
//   data[3] = num_bins             (number of magnitude bins that follow)
//   data[4 .. 4+num_bins-1] = magnitude spectrum, bin i is at i * data[0] Hz

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"

#include "reson/fft.hpp"

class FftNode : public rclcpp::Node
{
public:
  FftNode()
  : Node("fft_node")
  {
    window_size_ = static_cast<std::size_t>(declare_parameter<int>("window_size", 1024));
    hop_size_ = static_cast<std::size_t>(
      declare_parameter<int>("hop_size", static_cast<int>(window_size_)));
    sample_rate_hz_param_ = declare_parameter<double>("sample_rate_hz", 142.7);
    auto_estimate_rate_ = declare_parameter<bool>("auto_estimate_sample_rate", true);
    input_topic_ = declare_parameter<std::string>("input_topic", "reson/vibration_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "reson/fft_spectrum");

    if (hop_size_ == 0 || hop_size_ > window_size_) {
      hop_size_ = window_size_;
    }

    sub_ = create_subscription<std_msgs::msg::Float64>(
      input_topic_, 200,
      std::bind(&FftNode::sample_callback, this, std::placeholders::_1));

    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, 20);

    RCLCPP_INFO(
      get_logger(),
      "fft_node ready: window=%zu hop=%zu sample_rate_param=%.2fHz auto_rate=%s -> '%s'",
      window_size_, hop_size_, sample_rate_hz_param_,
      auto_estimate_rate_ ? "true" : "false", output_topic_.c_str());
  }

private:
  void sample_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (buffer_.empty()) {
      window_start_time_ = now();
    }
    buffer_.push_back(msg->data);

    if (buffer_.size() >= window_size_) {
      process_window();

      // Slide the buffer by hop_size (hop_size == window_size => no overlap).
      for (std::size_t i = 0; i < hop_size_ && !buffer_.empty(); ++i) {
        buffer_.pop_front();
      }
    }
  }

  void process_window()
  {
    std::vector<double> window(buffer_.begin(), buffer_.begin() + window_size_);

    double mean = 0.0;
    for (double v : window) {mean += v;}
    mean /= static_cast<double>(window.size());
    double var = 0.0;
    for (double v : window) {var += (v - mean) * (v - mean);}
    var /= static_cast<double>(window.size());
    const double std_dev = std::sqrt(var);

    double sample_rate_hz = sample_rate_hz_param_;
    if (auto_estimate_rate_) {
      const double elapsed_s = (now() - window_start_time_).seconds();
      if (elapsed_s > 0.0) {
        sample_rate_hz = static_cast<double>(window_size_ - 1) / elapsed_s;
      }
    }

    double freq_resolution_hz = 0.0;
    std::vector<double> mag = reson::magnitude_spectrum(window, sample_rate_hz, freq_resolution_hz);

    std_msgs::msg::Float32MultiArray out;
    std_msgs::msg::MultiArrayDimension meta_dim;
    meta_dim.label = "meta";
    meta_dim.size = 4;
    meta_dim.stride = 4 + mag.size();
    std_msgs::msg::MultiArrayDimension spec_dim;
    spec_dim.label = "spectrum";
    spec_dim.size = static_cast<uint32_t>(mag.size());
    spec_dim.stride = mag.size();
    out.layout.dim.push_back(meta_dim);
    out.layout.dim.push_back(spec_dim);

    out.data.reserve(4 + mag.size());
    out.data.push_back(static_cast<float>(freq_resolution_hz));
    out.data.push_back(static_cast<float>(sample_rate_hz));
    out.data.push_back(static_cast<float>(std_dev));
    out.data.push_back(static_cast<float>(mag.size()));
    for (double m : mag) {
      out.data.push_back(static_cast<float>(m));
    }

    pub_->publish(out);
  }

  std::size_t window_size_ = 1024;
  std::size_t hop_size_ = 1024;
  double sample_rate_hz_param_ = 142.7;
  bool auto_estimate_rate_ = true;
  std::string input_topic_;
  std::string output_topic_;

  std::deque<double> buffer_;
  rclcpp::Time window_start_time_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FftNode>());
  rclcpp::shutdown();
  return 0;
}
