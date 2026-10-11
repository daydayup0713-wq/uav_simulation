// ROS 2 output hooks for the unmodified upstream optimization/initialization.
// Original estimator is GPL-3.0 (HKUST Aerial Robotics Group, 2019).
#include "utility/visualization.h"
#include "vins_transport.hpp"
std::function<void(const VinsKeyframe&)> platform_keyframe;
std::function<void(double,const Eigen::Vector3d&,const Eigen::Matrix3d&)> platform_extrinsic;
std::function<void(double,const Eigen::Vector3d&,const Eigen::Matrix3d&,const Eigen::Vector3d&)> platform_odometry;
std::function<void(double,const std::vector<Eigen::Vector3d>&)> platform_landmarks;
void pubOdometry(const Estimator& estimator,const std_msgs::Header& header) {
  if (estimator.solver_flag == Estimator::NON_LINEAR && platform_odometry)
    platform_odometry(header.stamp.toSec(),estimator.Ps[WINDOW_SIZE],estimator.Rs[WINDOW_SIZE],estimator.Vs[WINDOW_SIZE]);
}
void pubPointCloud(const Estimator& estimator,const std_msgs::Header& header) {
  if (estimator.solver_flag != Estimator::NON_LINEAR || !platform_landmarks) return;
  std::vector<Eigen::Vector3d> points;
  for (const auto& feature:estimator.f_manager.feature) {
    if (feature.solve_flag != 1 || feature.feature_per_frame.empty() || feature.estimated_depth<=0) continue;
    const int frame=feature.start_frame;
    if (frame<0 || frame>WINDOW_SIZE) continue;
    Eigen::Vector3d point=feature.feature_per_frame.front().point*feature.estimated_depth;
    points.push_back(estimator.Rs[frame]*(estimator.ric[0]*point+estimator.tic[0])+estimator.Ps[frame]);
  }
  platform_landmarks(header.stamp.toSec(),points);
}
void pubLatestOdometry(const Eigen::Vector3d&,const Eigen::Quaterniond&,const Eigen::Vector3d&,double) {}
void pubTrackImage(const cv::Mat&,double) {}
void printStatistics(const Estimator&,double) {}
void pubInitialGuess(const Estimator&,const std_msgs::Header&) {}
void pubKeyPoses(const Estimator&,const std_msgs::Header&) {}
void pubCameraPose(const Estimator&,const std_msgs::Header&) {}
void pubTF(const Estimator& estimator,const std_msgs::Header& header) {
  if(estimator.solver_flag==Estimator::NON_LINEAR && platform_extrinsic)
    platform_extrinsic(header.stamp.toSec(),estimator.tic[0],estimator.ric[0]);
}
void pubKeyframe(const Estimator& estimator) {
  if(estimator.solver_flag!=Estimator::NON_LINEAR ||
     estimator.marginalization_flag!=Estimator::MARGIN_OLD || !platform_keyframe) return;
  const int frame=WINDOW_SIZE-2;
  VinsKeyframe packet{estimator.Headers[frame],estimator.Ps[frame],estimator.Rs[frame],{}, {}};
  for(const auto& feature:estimator.f_manager.feature) {
    const int start=feature.start_frame;
    const int offset=frame-start;
    if(start<0 || start>=frame || offset>=int(feature.feature_per_frame.size()) ||
       feature.solve_flag!=1 || !std::isfinite(feature.estimated_depth) || feature.estimated_depth<=0) continue;
    const auto& measurement=feature.feature_per_frame[offset];
    const Eigen::Vector3d point=estimator.Rs[start]*(estimator.ric[0]*
        (feature.feature_per_frame[0].point*feature.estimated_depth)+estimator.tic[0])+estimator.Ps[start];
    if(!point.allFinite() || !measurement.point.allFinite() || !measurement.uv.allFinite()) continue;
    packet.points.push_back(point);
    packet.observations.push_back({float(measurement.point.x()),float(measurement.point.y()),
                                  float(measurement.uv.x()),float(measurement.uv.y()),float(feature.feature_id)});
  }
  platform_keyframe(packet);
}
void pubRelocalization(const Estimator&) {}
void pubCar(const Estimator&,const std_msgs::Header&) {}
