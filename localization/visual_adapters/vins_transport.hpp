#pragma once
#include <array>
#include <functional>
#include <vector>
#include <Eigen/Core>
struct VinsKeyframe {
  double stamp;
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
  std::vector<Eigen::Vector3d> points;
  std::vector<std::array<float,5>> observations;
};
extern std::function<void(const VinsKeyframe&)> platform_keyframe;
extern std::function<void(double,const Eigen::Vector3d&,const Eigen::Matrix3d&)> platform_extrinsic;
