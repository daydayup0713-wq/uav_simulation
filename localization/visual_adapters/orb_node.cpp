// ROS 2 monocular-inertial transport for the official ORB-SLAM3 GPL-3.0 core.
#include <deque>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include "System.h"
#include "MapPoint.h"
#include "continuous_pose.hpp"

double seconds(const builtin_interfaces::msg::Time& t) { return t.sec+t.nanosec*1e-9; }
class OrbNode:public rclcpp::Node {
  ORB_SLAM3::System slam;
  std::string output;
  std::deque<ORB_SLAM3::IMU::Point> measurements;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images;
  double last_imu=-1,last_image=-1,last_pose=-1;
  ContinuousPose continuous;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr poses,global_poses;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points,global_points;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr states;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera;
  rclcpp::TimerBase::SharedPtr ready;
public:
  OrbNode(const std::string& vocabulary,const std::string& config,const std::string& directory):
    Node("orb_slam3_core"),slam(vocabulary,config,ORB_SLAM3::System::IMU_MONOCULAR,false),output(directory) {
    const std::string prefix="/uav001/backends/orb_slam3";
    poses=create_publisher<nav_msgs::msg::Odometry>(prefix+"/raw_odometry",rclcpp::SensorDataQoS());
    global_poses=create_publisher<nav_msgs::msg::Odometry>(prefix+"/global_odometry",rclcpp::SensorDataQoS());
    points=create_publisher<sensor_msgs::msg::PointCloud2>(prefix+"/registered_points",rclcpp::SensorDataQoS());
    global_points=create_publisher<sensor_msgs::msg::PointCloud2>(prefix+"/global_registered_points",rclcpp::SensorDataQoS());
    states=create_publisher<std_msgs::msg::String>(prefix+"/core_state",10);
    std_msgs::msg::String initialized;initialized.data="{\"input_ready\":true,\"inertial_initialized\":false}";states->publish(initialized);
    ready=create_wall_timer(std::chrono::seconds(1),[this] {
      Eigen::Matrix4f pose;int map_id=-1;const bool metric=slam.GetMetricImuPose(pose,map_id);
      std_msgs::msg::String msg;msg.data="{\"input_ready\":true,\"tracking_state\":"+std::to_string(slam.GetTrackingState())+",\"inertial_initialized\":"+(metric?"true":"false")+"}";states->publish(msg);
    });
    imu=create_subscription<sensor_msgs::msg::Imu>("/uav001/imu/data",rclcpp::SensorDataQoS().keep_last(512),[this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      const double t=seconds(msg->header.stamp);if(t<=last_imu) return;
      const auto& a=msg->linear_acceleration;const auto& w=msg->angular_velocity;
      if(!std::isfinite(a.x+a.y+a.z+w.x+w.y+w.z)) return;
      last_imu=t;measurements.emplace_back(a.x,a.y,a.z,w.x,w.y,w.z,t);
      if(measurements.size()>2000) {measurements.pop_front();RCLCPP_WARN(get_logger(),"IMU queue overrun");}flush();
    });
    camera=create_subscription<sensor_msgs::msg::Image>("/uav001/camera/image_raw",rclcpp::SensorDataQoS().keep_last(8),[this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      const double t=seconds(msg->header.stamp);if(t<=last_image) return;
      last_image=t;images.push_back(msg);if(images.size()>8) {images.pop_front();RCLCPP_WARN(get_logger(),"camera queue overrun");}flush();
    });
  }
  void flush() {
    while(!images.empty() && seconds(images.front()->header.stamp)<=last_imu) {
      const auto image=images.front();images.pop_front();const double t=seconds(image->header.stamp);
      std::vector<ORB_SLAM3::IMU::Point> batch;
      while(!measurements.empty() && measurements.front().t<=t) {batch.push_back(measurements.front());measurements.pop_front();}
      cv::Mat gray;
      try {gray=cv_bridge::toCvCopy(image,"mono8")->image;}
      catch(const cv_bridge::Exception& error) {RCLCPP_ERROR(get_logger(),"%s",error.what());continue;}
      slam.TrackMonocular(gray,t,batch);
      Eigen::Matrix4f pose,previous_pose;double previous_stamp=-1.;int map_id=-1;
      const bool metric=slam.GetMetricImuPose(pose,map_id,&previous_pose,&previous_stamp);
      const bool was_failed=continuous.failed();
      const bool local=metric && t>last_pose && continuous.update(t,pose,previous_pose,previous_stamp,map_id);
      if(!was_failed && continuous.failed())
        RCLCPP_ERROR(get_logger(),"local continuity stopped: current=%.9f paired_previous=%.9f last_published=%.9f map=%d reason=%s",t,previous_stamp,last_pose,map_id,continuous.reason().c_str());
      std_msgs::msg::String state;state.data="{\"backend\":\"orb_slam3\",\"tracking_state\":"+std::to_string(slam.GetTrackingState())+",\"inertial_initialized\":"+(metric?"true":"false")+",\"map_id\":"+std::to_string(map_id)+",\"continuity_failed\":"+(continuous.failed()?"true":"false")+",\"reason\":\""+continuous.reason()+"\"}";states->publish(state);
      if(!metric || t<=last_pose) continue;
      last_pose=t;nav_msgs::msg::Odometry msg;msg.header.stamp=image->header.stamp;
      msg.header.frame_id="orb_slam3_map";msg.child_frame_id="imu_link";
      auto set_pose=[&msg](const Eigen::Matrix4f& value) {
        msg.pose.pose.position.x=value(0,3);msg.pose.pose.position.y=value(1,3);msg.pose.pose.position.z=value(2,3);
        const Eigen::Quaternionf q(value.block<3,3>(0,0));msg.pose.pose.orientation.x=q.x();msg.pose.pose.orientation.y=q.y();msg.pose.pose.orientation.z=q.z();msg.pose.pose.orientation.w=q.w();
      };
      set_pose(pose);global_poses->publish(msg);
      if(local) {msg.header.frame_id="orb_slam3_odom";set_pose(continuous.local());poses->publish(msg);}
      std::vector<Eigen::Vector3f> landmarks;
      for(auto* p:slam.GetTrackedMapPoints()) if(p && !p->isBad()) landmarks.push_back(p->GetWorldPos());
      sensor_msgs::msg::PointCloud2 cloud;cloud.header=msg.header;cloud.header.frame_id="orb_slam3_map";
      sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(landmarks.size());
      sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z");
      for(const auto& p:landmarks) {*x=p.x();*y=p.y();*z=p.z();++x;++y;++z;}global_points->publish(cloud);
      if(local) {
        cloud.header.frame_id="orb_slam3_odom";
        sensor_msgs::PointCloud2Iterator<float> lx(cloud,"x"),ly(cloud,"y"),lz(cloud,"z");
        const Eigen::Matrix4f transform=continuous.correction().inverse();
        for(const auto& p:landmarks) {const Eigen::Vector4f v=transform*Eigen::Vector4f(p.x(),p.y(),p.z(),1.f);*lx=v.x();*ly=v.y();*lz=v.z();++lx;++ly;++lz;}
        points->publish(cloud);
      }
    }
  }
  bool has_metric_pose() const {return last_pose>0;}
  ~OrbNode() {
    slam.Shutdown();
    if(last_pose>0) {
      slam.SaveTrajectoryEuRoC(output+"/optimized-imu-nanoseconds.tum");
      slam.SaveKeyFrameTrajectoryTUM(output+"/keyframes-camera.tum");
    }
  }
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);if(argc<4) return 2;
  try {
    auto node=std::make_shared<OrbNode>(argv[1],argv[2],argv[3]);
    if(argc>4 && std::string(argv[4])=="--probe-initialization") {
      const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
      while(std::chrono::steady_clock::now()<until) {rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(10));}
      if(node->has_metric_pose()) return 3;
      std::printf("metric_ready=false\n");
    } else rclcpp::spin(node);
  }
  catch(const std::exception& error) {std::fprintf(stderr,"ORB adapter: %s\n",error.what());return 1;}
  rclcpp::shutdown();return 0;
}
