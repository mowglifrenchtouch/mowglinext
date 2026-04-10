#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

class TfLunaNode : public rclcpp::Node
{
public:
  TfLunaNode();
  ~TfLunaNode();

private:
  void tryConnect();
  bool openSerial();
  void closeSerial();

  void readLoop();
  bool readFrame(std::array<uint8_t, 9> &frame);
  bool parseFrame(
    const std::array<uint8_t, 9> &frame,
    float &distance_m,
    float &strength,
    float &temperature_c);

  void publishRange(float distance_m);

private:
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  std::string port_;
  int baudrate_;
  std::string frame_id_;
  std::string topic_name_;
  double field_of_view_;
  double min_range_;
  double max_range_;
  bool clamp_to_limits_;
  bool debug_log_;
  int reconnect_period_ms_;

  int serial_fd_;
  std::atomic<bool> running_;
  std::atomic<bool> connected_;
  std::thread read_thread_;
  std::mutex serial_mutex_;
};