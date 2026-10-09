// Native boundary probe of the real estimator's ROS output transport.
#include "utility/visualization.h"
#include "vins_transport.hpp"
#include <cmath>
int main() {
  MULTIPLE_THREAD=0;
  Estimator estimator;
  int calls=0, extrinsics=0;
  VinsKeyframe received;
  platform_keyframe=[&](const VinsKeyframe& packet){received=packet;++calls;};
  platform_extrinsic=[&](double t,const Eigen::Vector3d& p,const Eigen::Matrix3d& r){
    if(t==12.5 && (p-Eigen::Vector3d(1,0,0)).norm()<1e-9 && r.isIdentity()) ++extrinsics;
  };
  std_msgs::Header header;header.stamp=ros::Time(12.5);
  pubKeyframe(estimator);pubTF(estimator,header);
  if(calls || extrinsics) return 2;
  estimator.solver_flag=Estimator::NON_LINEAR;
  estimator.marginalization_flag=Estimator::MARGIN_OLD;
  estimator.Headers[WINDOW_SIZE-2]=11.25;
  estimator.Ps[WINDOW_SIZE-2]=Eigen::Vector3d(3,4,5);
  estimator.Ps[0]=Eigen::Vector3d(0,2,0);
  estimator.tic[0]=Eigen::Vector3d(1,0,0);
  estimator.ric[0].setIdentity();
  FeaturePerId feature(42,0);feature.solve_flag=1;feature.estimated_depth=2;
  Eigen::Matrix<double,7,1> observation;observation << .1,.2,1,123,234,0,0;
  for(int i=0;i<=WINDOW_SIZE-2;++i) feature.feature_per_frame.emplace_back(observation,0);
  estimator.f_manager.feature.push_back(feature);
  pubKeyframe(estimator);pubTF(estimator,header);
  if(calls!=1 || extrinsics!=1 || received.stamp!=11.25 || received.points.size()!=1 ||
     received.observations.size()!=1 || (received.position-Eigen::Vector3d(3,4,5)).norm()>1e-9 ||
     (received.points[0]-Eigen::Vector3d(1.2,2.4,2)).norm()>1e-9 ||
     received.observations[0][2]!=123 || received.observations[0][4]!=42) return 3;
  estimator.marginalization_flag=Estimator::MARGIN_SECOND_NEW;pubKeyframe(estimator);
  if(calls!=1) return 4;
  platform_keyframe={};platform_extrinsic={};return 0;
}
