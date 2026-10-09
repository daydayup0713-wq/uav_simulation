#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdio>
// Source compatibility for ROS1 parameter/logging APIs; actual transport is ROS2.
namespace ros {
using Time=rclcpp::Time;
class NodeHandle {
  rclcpp::Node::SharedPtr node;
public:
  explicit NodeHandle(rclcpp::Node::SharedPtr n):node(std::move(n)) {}
  template<class T> void param(const std::string& name,T& value,const T& fallback) {
    if(!node->has_parameter(name))node->declare_parameter<T>(name,fallback);
    node->get_parameter(name,value);
  }
};
}
#define ROS_INFO_STREAM(v) do {std::ostringstream s;s<<v;std::fprintf(stderr,"%s\n",s.str().c_str());} while(0)
#define ROS_WARN(...) do {std::fprintf(stderr,__VA_ARGS__);std::fputc('\n',stderr);} while(0)
#define ROS_ERROR(...) ROS_WARN(__VA_ARGS__)
