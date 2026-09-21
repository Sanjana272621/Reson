// diagnosis_node
//
// Subscribes to /reson/fft_spectrum (see fft_node.cpp for the wire format),
// extracts a spectral fingerprint, compares it against a baseline "healthy
// motor" fingerprint (similarity/distance) and a rule-based threshold table,
// and publishes:
//   - diagnostic_msgs/DiagnosticArray on /diagnostics (standard ROS 2
//     diagnostics topic, viewable with `ros2 run rqt_runtime_monitor
//     rqt_runtime_monitor` or `ros2 topic echo /diagnostics`)
//   - std_msgs/Float32 health index (0-100) on /reson/health_index

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include "reson/fingerprint.hpp"

using reson::Fingerprint;
using reson::Severity;
using reson::Thresholds;

namespace
{
diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue out;
  out.key = key;
  out.value = value;
  return out;
}

template<typename T>
diagnostic_msgs::msg::KeyValue kv_num(const std::string & key, T value)
{
  return kv(key, std::to_string(value));
}
}  // namespace

class DiagnosisNode : public rclcpp::Node
{
public:
  DiagnosisNode()
  : Node("diagnosis_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "reson/fft_spectrum");
    diagnostic_name_ = declare_parameter<std::string>("diagnostic_name", "motor_vibration");
    hardware_id_ = declare_parameter<std::string>("hardware_id", "motor_1");
    min_freq_hz_ = declare_parameter<double>("min_freq_hz", 5.0);
    peak_ratio_ = declare_parameter<double>("peak_ratio", 0.10);
    distance_scale_ = declare_parameter<double>("distance_scale", 3.0);

    // Baseline "healthy motor" fingerprint. Defaults are the NORMAL row
    // measured on the reference grinder-motor rig; override in
    // config/reson_params.yaml with your own motor's baseline (see
    // scripts/compute_baseline.py to generate one from your normal_*.csv
    // recordings).
    baseline_.std_dev = declare_parameter<double>("baseline.std_dev", 1752.12);
    baseline_.dominant_freq_hz = declare_parameter<double>("baseline.dominant_freq_hz", 25.92);
    baseline_.dominant_mag = declare_parameter<double>("baseline.dominant_mag", 190983.1);
    baseline_.num_peaks = declare_parameter<int>("baseline.num_peaks", 857);
    baseline_.energy_5_10 = declare_parameter<double>("baseline.energy_5_10", 8.2);
    baseline_.energy_10_50 = declare_parameter<double>("baseline.energy_10_50", 57.2);
    baseline_.energy_50_100 = declare_parameter<double>("baseline.energy_50_100", 26.2);
    baseline_.energy_above_100 = declare_parameter<double>("baseline.energy_above_100", 0.0);

    // Per-feature normalization divisors for the fingerprint distance
    // (roughly "one baseline standard deviation" per feature).
    scale_ = {
      declare_parameter<double>("scale.std_dev", 1500.0),
      declare_parameter<double>("scale.dominant_freq_hz", 8.0),
      declare_parameter<double>("scale.log_dominant_mag", 0.5),
      declare_parameter<double>("scale.num_peaks", 250.0),
      declare_parameter<double>("scale.energy_5_10", 10.0),
      declare_parameter<double>("scale.energy_10_50", 15.0),
      declare_parameter<double>("scale.energy_50_100", 15.0),
      declare_parameter<double>("scale.energy_above_100", 5.0),
    };

    thresholds_.std_dev_warning = declare_parameter<double>("thresholds.std_dev_warning", 2000.0);
    thresholds_.std_dev_alert = declare_parameter<double>("thresholds.std_dev_alert", 3000.0);
    thresholds_.std_dev_critical = declare_parameter<double>("thresholds.std_dev_critical", 5000.0);
    thresholds_.freq_warning = declare_parameter<double>("thresholds.freq_warning", 20.0);
    thresholds_.freq_alert = declare_parameter<double>("thresholds.freq_alert", 16.0);
    thresholds_.freq_critical = declare_parameter<double>("thresholds.freq_critical", 13.0);
    thresholds_.mag_warning = declare_parameter<double>("thresholds.mag_warning", 200000.0);
    thresholds_.mag_alert = declare_parameter<double>("thresholds.mag_alert", 400000.0);
    thresholds_.mag_critical = declare_parameter<double>("thresholds.mag_critical", 1000000.0);
    thresholds_.peaks_warning = declare_parameter<double>("thresholds.peaks_warning", 700.0);
    thresholds_.peaks_alert = declare_parameter<double>("thresholds.peaks_alert", 500.0);
    thresholds_.peaks_critical = declare_parameter<double>("thresholds.peaks_critical", 300.0);

    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      input_topic_, 20,
      std::bind(&DiagnosisNode::spectrum_callback, this, std::placeholders::_1));

    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    health_pub_ = create_publisher<std_msgs::msg::Float32>("reson/health_index", 10);

    RCLCPP_INFO(
      get_logger(), "diagnosis_node ready: listening on '%s', baseline dominant_freq=%.2fHz",
      input_topic_.c_str(), baseline_.dominant_freq_hz);
  }

private:
  void spectrum_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      RCLCPP_WARN(get_logger(), "Received malformed spectrum message (too short), dropping.");
      return;
    }
    const double freq_resolution_hz = msg->data[0];
    const double sample_rate_hz = msg->data[1];
    const double std_dev = msg->data[2];
    const std::size_t num_bins = static_cast<std::size_t>(msg->data[3]);

    if (msg->data.size() < 4 + num_bins) {
      RCLCPP_WARN(get_logger(), "Spectrum message shorter than declared num_bins, dropping.");
      return;
    }

    std::vector<double> mag(num_bins);
    for (std::size_t i = 0; i < num_bins; ++i) {
      mag[i] = static_cast<double>(msg->data[4 + i]);
    }

    Fingerprint fp = reson::extract_fingerprint(
      mag, freq_resolution_hz, std_dev, min_freq_hz_, peak_ratio_);

    reson::Diagnosis diag = reson::diagnose(
      fp, baseline_, scale_, thresholds_, distance_scale_);

    publish_diagnostics(fp, diag, sample_rate_hz);

    std_msgs::msg::Float32 health_msg;
    health_msg.data = static_cast<float>(diag.health_index);
    health_pub_->publish(health_msg);

    RCLCPP_INFO(
      get_logger(),
      "[%s] health=%.1f  std=%.1f  f0=%.2fHz  mag0=%.1f  peaks=%d  dist=%.2f  hint=%s",
      reson::severity_to_string(diag.overall).c_str(), diag.health_index, fp.std_dev,
      fp.dominant_freq_hz, fp.dominant_mag, fp.num_peaks, diag.baseline_distance,
      diag.fault_hint.c_str());
  }

  void publish_diagnostics(
    const Fingerprint & fp, const reson::Diagnosis & diag, double sample_rate_hz)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = diagnostic_name_;
    status.hardware_id = hardware_id_;

    switch (diag.overall) {
      case Severity::kNormal:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "Normal - no fault indicators";
        break;
      case Severity::kWarning:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Warning - early deviation from baseline: " + diag.fault_hint;
        break;
      case Severity::kAlert:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Alert - significant fault indicators: " + diag.fault_hint;
        break;
      case Severity::kCritical:
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Critical - schedule maintenance now: " + diag.fault_hint;
        break;
    }

    status.values.push_back(kv_num("health_index", diag.health_index));
    status.values.push_back(kv_num("baseline_distance", diag.baseline_distance));
    status.values.push_back(kv_num("std_dev", fp.std_dev));
    status.values.push_back(kv("std_dev_severity", reson::severity_to_string(diag.std_dev_sev)));
    status.values.push_back(kv_num("dominant_freq_hz", fp.dominant_freq_hz));
    status.values.push_back(kv("dominant_freq_severity", reson::severity_to_string(diag.freq_sev)));
    status.values.push_back(kv_num("dominant_magnitude", fp.dominant_mag));
    status.values.push_back(kv("dominant_magnitude_severity", reson::severity_to_string(diag.mag_sev)));
    status.values.push_back(kv_num("num_significant_peaks", fp.num_peaks));
    status.values.push_back(kv("num_peaks_severity", reson::severity_to_string(diag.peaks_sev)));
    status.values.push_back(kv_num("sample_rate_hz", sample_rate_hz));
    status.values.push_back(kv_num("energy_5_10_pct", fp.energy_5_10));
    status.values.push_back(kv_num("energy_10_50_pct", fp.energy_10_50));
    status.values.push_back(kv_num("energy_50_100_pct", fp.energy_50_100));
    status.values.push_back(kv_num("energy_above_100_pct", fp.energy_above_100));
    status.values.push_back(kv("fault_hint", diag.fault_hint));

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(status);
    diag_pub_->publish(arr);
  }

  std::string input_topic_;
  std::string diagnostic_name_;
  std::string hardware_id_;
  double min_freq_hz_ = 5.0;
  double peak_ratio_ = 0.10;
  double distance_scale_ = 3.0;

  Fingerprint baseline_;
  std::vector<double> scale_;
  Thresholds thresholds_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr health_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiagnosisNode>());
  rclcpp::shutdown();
  return 0;
}
