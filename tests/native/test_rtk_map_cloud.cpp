#include "platform_cloud_transform.hpp"
#include <cassert>
int main() {
    pcl::PointCloud<pcl::PointXYZRGB> empty, merged, frame;
    Eigen::Affine3f transform=Eigen::Affine3f::Identity();
    transform.translation()<<1.f,2.f,3.f;
    // Empty colored frames legitimately occur when the camera sees no points.
    // They must preserve the pose keyframe without crashing map export.
    platform_io::appendKeyframeCloud(empty,transform,merged);
    assert(merged.empty());
    pcl::PointXYZRGB point;point.x=2;point.y=3;point.z=4;point.r=10;point.g=20;point.b=30;
    frame.push_back(point);
    platform_io::appendKeyframeCloud(frame,transform,merged);
    platform_io::appendKeyframeCloud(empty,transform,merged);
    assert(merged.size()==1 && merged.width==1 && merged.height==1);
    assert(merged[0].x==3 && merged[0].y==5 && merged[0].z==7 && merged[0].r==10);
}
