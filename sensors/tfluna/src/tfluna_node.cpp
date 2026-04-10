#include "tfluna_node.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <limits>

namespace
{
speed_t baudrateToTermios(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return B115200;
  }
}
}  // namespace

TfLunaNode::TfLunaNode()
: Node("tfluna_node"),
  serial_fd_(-1),
  running_(true),
  connected_(false)
{
  this->declare_parameter<std::string>("port", "/dev/tfluna_front");
  this->declare_parameter<int>("baudrate", 115200);
  this->declare_parameter<std::string>("frame_id", "tfluna_link");
  this->declare_parameter<std::string>("topic_name", "/sensors/tfluna/range");
  this->declare_parameter<double>("field_of_view", 0.035);
  this->declare_parameter<double>("min_range", 0.2);
  this->declare_parameter<double>("max_range", 8.0);
  this->declare_parameter<bool>("clamp_to_limits", false);
  this->declare_parameter<bool>("debug_log", false);
  this->declare_parameter<int>("reconnect_period_ms", 2000);

  this->get_parameter("port", port_);
  this->get_parameter("baudrate", baudrate_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("topic_name", topic_name_);
  this->get_parameter("field_of_view", field_of_view_);
  this->get_parameter("min_range", min_range_);
  this->get_parameter("max_range", max_range_);
  this->get_parameter("clamp_to_limits", clamp_to_limits_);
  this->get_parameter("debug_log", debug_log_);
  this->get_parameter("reconnect_period_ms", reconnect_period_ms_);

  range_pub_ = this->create_publisher<sensor_msgs::msg::Range>(topic_name_, 10);

  reconnect_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(reconnect_period_ms_),
    std::bind(&TfLunaNode::tryConnect, this));

  read_thread_ = std::thread(&TfLunaNode::readLoop, this);

  RCLCPP_INFO(
    this->get_logger(),
    "tfluna_node prêt. port=%s baudrate=%d topic=%s",
    port_.c_str(), baudrate_, topic_name_.c_str());

  tryConnect();
}

TfLunaNode::~TfLunaNode()
{
  running_ = false;

  if (read_thread_.joinable()) {
    read_thread_.join();
  }

  closeSerial();
}

void TfLunaNode::tryConnect()
{
  if (connected_) {
    return;
  }

  if (openSerial()) {
    connected_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "TF-Luna connecté sur %s à %d bauds",
      port_.c_str(), baudrate_);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "TF-Luna non disponible sur %s, nouvelle tentative...",
      port_.c_str());
  }
}

bool TfLunaNode::openSerial()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  if (serial_fd_ >= 0) {
    return true;
  }

  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    return false;
  }

  struct termios tty {};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    closeSerial();
    return false;
  }

  cfsetospeed(&tty, baudrateToTermios(baudrate_));
  cfsetispeed(&tty, baudrateToTermios(baudrate_));

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    closeSerial();
    return false;
  }

  tcflush(serial_fd_, TCIFLUSH);
  return true;
}

void TfLunaNode::closeSerial()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }

  connected_ = false;
}

bool TfLunaNode::readFrame(std::array<uint8_t, 9> &frame)
{
  uint8_t byte = 0;

  while (running_ && connected_) {
    ssize_t n = read(serial_fd_, &byte, 1);
    if (n <= 0) {
      return false;
    }

    if (byte != 0x59) {
      continue;
    }

    uint8_t second = 0;
    ssize_t n2 = read(serial_fd_, &second, 1);
    if (n2 <= 0) {
      return false;
    }

    if (second != 0x59) {
      continue;
    }

    frame[0] = 0x59;
    frame[1] = 0x59;

    size_t offset = 2;
    while (offset < frame.size() && running_ && connected_) {
      ssize_t nr = read(serial_fd_, frame.data() + offset, frame.size() - offset);
      if (nr > 0) {
        offset += static_cast<size_t>(nr);
      } else {
        return false;
      }
    }

    return offset == frame.size();
  }

  return false;
}

bool TfLunaNode::parseFrame(
  const std::array<uint8_t, 9> &frame,
  float &distance_m,
  float &strength,
  float &temperature_c)
{
  int checksum = 0;
  for (int i = 0; i < 8; ++i) {
    checksum += frame[i];
  }
  checksum &= 0xFF;

  if (checksum != frame[8]) {
    return false;
  }

  const uint16_t distance_cm =
    static_cast<uint16_t>(frame[2]) |
    (static_cast<uint16_t>(frame[3]) << 8);

  const uint16_t amp =
    static_cast<uint16_t>(frame[4]) |
    (static_cast<uint16_t>(frame[5]) << 8);

  const uint16_t temp_raw =
    static_cast<uint16_t>(frame[6]) |
    (static_cast<uint16_t>(frame[7]) << 8);

  distance_m = static_cast<float>(distance_cm) / 100.0f;
  strength = static_cast<float>(amp);
  temperature_c = static_cast<float>(temp_raw) / 8.0f - 256.0f;

  return true;
}

void TfLunaNode::publishRange(float distance_m)
{
  sensor_msgs::msg::Range msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;
  msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  msg.field_of_view = static_cast<float>(field_of_view_);
  msg.min_range = static_cast<float>(min_range_);
  msg.max_range = static_cast<float>(max_range_);

  if (clamp_to_limits_) {
    if (distance_m < msg.min_range) {
      distance_m = msg.min_range;
    }
    if (distance_m > msg.max_range) {
      distance_m = msg.max_range;
    }
    msg.range = distance_m;
  } else {
    if (distance_m < msg.min_range || distance_m > msg.max_range) {
      msg.range = std::numeric_limits<float>::infinity();
    } else {
      msg.range = distance_m;
    }
  }

  range_pub_->publish(msg);
}

void TfLunaNode::readLoop()
{
  std::array<uint8_t, 9> frame{};

  while (rclcpp::ok() && running_) {
    if (!connected_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    if (!readFrame(frame)) {
      if (connected_) {
        RCLCPP_WARN(this->get_logger(), "Perte de communication série avec %s", port_.c_str());
      }
      closeSerial();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      continue;
    }

    float distance_m = 0.0f;
    float strength = 0.0f;
    float temperature_c = 0.0f;

    if (!parseFrame(frame, distance_m, strength, temperature_c)) {
      if (debug_log_) {
        RCLCPP_WARN(this->get_logger(), "Checksum TF-Luna invalide");
      }
      continue;
    }

    if (debug_log_) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "distance=%.3f m strength=%.1f temp=%.1f C",
        distance_m, strength, temperature_c);
    }

    publishRange(distance_m);
  }
}