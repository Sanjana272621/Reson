// csv_replay_node
//
// Reads a two-column vibration CSV (time_ms, value) recorded from the
// physical motor rig and republishes it sample-by-sample on
// /reson/vibration_raw at (approximately) the original sample timing,
// so the rest of the pipeline (fft_node, diagnosis_node) behaves the same
// whether it is fed live sensor data or a recorded run.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    // trim whitespace/CR
    while (!cell.empty() && std::isspace(static_cast<unsigned char>(cell.back()))) {
      cell.pop_back();
    }
    std::size_t start = 0;
    while (start < cell.size() && std::isspace(static_cast<unsigned char>(cell[start]))) {
      ++start;
    }
    out.push_back(cell.substr(start));
  }
  return out;
}

bool is_number(const std::string & s)
{
  if (s.empty()) {return false;}
  char * end = nullptr;
  std::strtod(s.c_str(), &end);
  return end != s.c_str() && *end == '\0';
}

}  // namespace

class CsvReplayNode : public rclcpp::Node
{
public:
  CsvReplayNode()
  : Node("csv_replay_node")
  {
    csv_path_ = declare_parameter<std::string>("csv_path", "");
    time_column_ = declare_parameter<std::string>("time_column", "");
    value_column_ = declare_parameter<std::string>("value_column", "");
    loop_ = declare_parameter<bool>("loop", false);
    playback_rate_ = declare_parameter<double>("playback_rate", 1.0);
    topic_ = declare_parameter<std::string>("output_topic", "reson/vibration_raw");
    default_sample_rate_hz_ = declare_parameter<double>("default_sample_rate_hz", 142.7);

    if (csv_path_.empty()) {
      RCLCPP_FATAL(get_logger(), "Required parameter 'csv_path' was not set.");
      throw std::runtime_error("csv_path parameter is required");
    }

    pub_ = create_publisher<std_msgs::msg::Float64>(topic_, 100);
    status_pub_ = create_publisher<std_msgs::msg::String>("reson/replay_status", 10);

    if (!load_csv(csv_path_)) {
      throw std::runtime_error("Failed to load CSV: " + csv_path_);
    }

    RCLCPP_INFO(
      get_logger(), "Loaded %zu samples from '%s' (loop=%s, rate=x%.2f) -> publishing on '%s'",
      values_.size(), csv_path_.c_str(), loop_ ? "true" : "false", playback_rate_,
      topic_.c_str());

    running_ = true;
    worker_ = std::thread(&CsvReplayNode::replay_loop, this);
  }

  ~CsvReplayNode() override
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  bool load_csv(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_FATAL(get_logger(), "Could not open CSV file: %s", path.c_str());
      return false;
    }

    std::string line;
    std::vector<std::string> header;
    int time_idx = -1;
    int value_idx = -1;

    if (!std::getline(file, line)) {
      RCLCPP_FATAL(get_logger(), "CSV file is empty: %s", path.c_str());
      return false;
    }
    header = split_csv_line(line);

    auto find_col = [&header](const std::vector<std::string> & candidates) -> int {
      for (std::size_t i = 0; i < header.size(); ++i) {
        std::string h = header[i];
        std::transform(h.begin(), h.end(), h.begin(), ::tolower);
        for (const auto & c : candidates) {
          if (h == c) {return static_cast<int>(i);}
        }
      }
      return -1;
    };

    if (!time_column_.empty()) {
      for (std::size_t i = 0; i < header.size(); ++i) {
        if (header[i] == time_column_) {time_idx = static_cast<int>(i);}
      }
    } else {
      time_idx = find_col({"time_ms", "timestamp", "time", "t", "ms"});
    }

    if (!value_column_.empty()) {
      for (std::size_t i = 0; i < header.size(); ++i) {
        if (header[i] == value_column_) {value_idx = static_cast<int>(i);}
      }
    } else {
      value_idx = find_col({"vibration", "value", "accel", "az", "ax", "ay", "z", "amplitude"});
      if (value_idx == -1) {
        // Fall back to "first numeric-looking column that isn't the time column".
        std::getline(file, line);  // peek first data row
        auto cells = split_csv_line(line);
        for (std::size_t i = 0; i < cells.size(); ++i) {
          if (static_cast<int>(i) != time_idx && is_number(cells[i])) {
            value_idx = static_cast<int>(i);
            break;
          }
        }
        file.clear();
        file.seekg(0);
        std::getline(file, line);  // re-skip header
      }
    }

    if (value_idx == -1) {
      RCLCPP_FATAL(get_logger(), "Could not identify a value column in CSV header.");
      return false;
    }

    RCLCPP_INFO(
      get_logger(), "CSV columns -> time: '%s' (idx %d), value: '%s' (idx %d)",
      time_idx >= 0 ? header[time_idx].c_str() : "<none, using default_sample_rate_hz>",
      time_idx, header[value_idx].c_str(), value_idx);

    while (std::getline(file, line)) {
      if (line.empty()) {continue;}
      auto cells = split_csv_line(line);
      if (static_cast<int>(cells.size()) <= value_idx) {continue;}
      if (!is_number(cells[value_idx])) {continue;}

      double v = std::stod(cells[value_idx]);
      double t_ms;
      if (time_idx >= 0 && static_cast<int>(cells.size()) > time_idx &&
        is_number(cells[time_idx]))
      {
        t_ms = std::stod(cells[time_idx]);
      } else {
        t_ms = values_.empty() ? 0.0 :
          times_ms_.back() + 1000.0 / default_sample_rate_hz_;
      }
      times_ms_.push_back(t_ms);
      values_.push_back(v);
    }

    return !values_.empty();
  }

  void publish_status(const std::string & msg)
  {
    std_msgs::msg::String s;
    s.data = msg;
    status_pub_->publish(s);
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
  }

  void replay_loop()
  {
    do {
      publish_status("REPLAY_START:" + csv_path_);
      for (std::size_t i = 0; i < values_.size() && running_ && rclcpp::ok(); ++i) {
        std_msgs::msg::Float64 msg;
        msg.data = values_[i];
        pub_->publish(msg);

        if (i + 1 < values_.size()) {
          double dt_ms = times_ms_[i + 1] - times_ms_[i];
          if (dt_ms <= 0.0) {dt_ms = 1000.0 / default_sample_rate_hz_;}
          dt_ms /= std::max(0.01, playback_rate_);
          std::this_thread::sleep_for(
            std::chrono::duration<double, std::milli>(dt_ms));
        }
      }
      publish_status("REPLAY_END:" + csv_path_);
    } while (loop_ && running_ && rclcpp::ok());
  }

  std::string csv_path_;
  std::string time_column_;
  std::string value_column_;
  std::string topic_;
  bool loop_ = false;
  double playback_rate_ = 1.0;
  double default_sample_rate_hz_ = 142.7;

  std::vector<double> times_ms_;
  std::vector<double> values_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::atomic<bool> running_{false};
  std::thread worker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CsvReplayNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("csv_replay_node"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
