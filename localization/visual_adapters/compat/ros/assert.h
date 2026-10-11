#pragma once
#include <ros/ros.h>
#include <cassert>
#define ROS_ASSERT(condition) do { if(!(condition)) { ROS_ERROR("VINS invariant failed: %s",#condition); std::abort(); } } while(0)
#define ROS_ASSERT_MSG(condition, ...) do { if(!(condition)) { ROS_ERROR(__VA_ARGS__); std::abort(); } } while(0)
