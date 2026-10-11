// Source compatibility for upstream logging/time only. ROS transport is rclcpp.
#pragma once
#include <cstdio>
#include <cstdlib>
#include <sstream>
namespace ros {
struct Time {
  double seconds{};
  Time() = default;
  explicit Time(double value): seconds(value) {}
  double toSec() const { return seconds; }
};
}
#define ROS_INFO(...) do { std::fprintf(stderr, __VA_ARGS__); std::fputc('\n',stderr); } while (0)
#define ROS_WARN(...) ROS_INFO(__VA_ARGS__)
#define ROS_ERROR(...) ROS_INFO(__VA_ARGS__)
#define ROS_DEBUG(...) do {} while (0)
#define ROS_INFO_STREAM(value) do { std::ostringstream line; line << value; std::fprintf(stderr,"%s\n",line.str().c_str()); } while (0)
#define ROS_WARN_STREAM(value) ROS_INFO_STREAM(value)
#define ROS_ERROR_STREAM(value) ROS_INFO_STREAM(value)
#define ROS_DEBUG_STREAM(value) do {} while (0)
#define ROS_BREAK() std::abort()
