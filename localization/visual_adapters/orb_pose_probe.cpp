#include "continuous_pose.hpp"
#include <Eigen/Geometry>
int main() {
  ContinuousPose output;
  const Eigen::Matrix4f initial=Eigen::Matrix4f::Identity();
  if(!output.update(10.,initial,initial,9.,1)) return 2;
  Eigen::Matrix4f gauge=Eigen::Matrix4f::Identity();
  gauge.block<3,3>(0,0)=Eigen::AngleAxisf(1.5f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
  gauge.block<3,1>(0,3)=Eigen::Vector3f(100,200,3);
  Eigen::Matrix4f current=initial;current(0,3)=1;
  if(!output.update(10.1,gauge*current,gauge*initial,10.,1)) return 3;
  if((output.local()-current).norm()>1e-4) return 4;
  if((output.correction()*(output.local())-gauge*current).norm()>1e-4) return 5;
  if(output.update(10.2,gauge*current,gauge*current,10.1,2)) return 6;
  if(output.update(10.3,gauge*current,gauge*current,10.2,1)) return 7;
  ContinuousPose dropped;dropped.update(10.,initial,initial,9.,1);
  if(dropped.update(10.2,current,initial,10.1,1)) return 8;
  return 0;
}
