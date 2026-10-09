// ROS 2 transport adapter for pinned HKUST VINS-Fusion (GPL-3.0).
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include "estimator/estimator.h"
#include "utility/visualization.h"
#include "vins_transport.hpp"

double seconds(const builtin_interfaces::msg::Time& t) { return t.sec+t.nanosec*1e-9; }
class VinsNode:public rclcpp::Node {
  Estimator estimator;
  double latest_imu=-1, previous_imu=-1, previous_image=-1, previous_pose=-1;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> images;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr poses;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr keyframe_pose, extrinsic;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr keyframe_points;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera;
  rclcpp::TimerBase::SharedPtr timer;
public:
  explicit VinsNode(const std::string& config):Node("vins_fusion_core") {
    readParameters(config); MULTIPLE_THREAD=0; estimator.setParameter();
    const std::string prefix="/uav001/backends/vins_fusion";
    poses=create_publisher<nav_msgs::msg::Odometry>(prefix+"/raw_odometry",rclcpp::SensorDataQoS());
    keyframe_pose=create_publisher<nav_msgs::msg::Odometry>(prefix+"/keyframe_pose",rclcpp::SensorDataQoS());
    keyframe_points=create_publisher<sensor_msgs::msg::PointCloud>(prefix+"/keyframe_point",rclcpp::SensorDataQoS());
    extrinsic=create_publisher<nav_msgs::msg::Odometry>(prefix+"/extrinsic",rclcpp::SensorDataQoS());
    points=create_publisher<sensor_msgs::msg::PointCloud2>(prefix+"/registered_points",rclcpp::SensorDataQoS());
    state=create_publisher<std_msgs::msg::String>(prefix+"/core_state",10);
    platform_odometry=[this](double t,const Eigen::Vector3d& p,const Eigen::Matrix3d& r,const Eigen::Vector3d& v) {
      if(t<=previous_pose || !p.allFinite() || !r.allFinite() || !v.allFinite()) return;
      previous_pose=t;
      nav_msgs::msg::Odometry msg;msg.header.stamp=rclcpp::Time(int64_t(t*1e9));
      msg.header.frame_id="vins_fusion_odom";msg.child_frame_id="imu_link";
      msg.pose.pose.position.x=p.x();msg.pose.pose.position.y=p.y();msg.pose.pose.position.z=p.z();
      const Eigen::Quaterniond q(r);
      msg.pose.pose.orientation.x=q.x();msg.pose.pose.orientation.y=q.y();msg.pose.pose.orientation.z=q.z();msg.pose.pose.orientation.w=q.w();
      msg.twist.twist.linear.x=v.x();msg.twist.twist.linear.y=v.y();msg.twist.twist.linear.z=v.z();poses->publish(msg);
    };
    platform_landmarks=[this](double t,const std::vector<Eigen::Vector3d>& landmarks) {
      sensor_msgs::msg::PointCloud2 msg;msg.header.stamp=rclcpp::Time(int64_t(t*1e9));msg.header.frame_id="vins_fusion_odom";
      sensor_msgs::PointCloud2Modifier modifier(msg);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(landmarks.size());
      sensor_msgs::PointCloud2Iterator<float> x(msg,"x"),y(msg,"y"),z(msg,"z");
      for(const auto& p:landmarks) {*x=p.x();*y=p.y();*z=p.z();++x;++y;++z;}points->publish(msg);
    };
    platform_keyframe=[this](const VinsKeyframe& packet) {
      auto pose=pose_message(packet.stamp,packet.position,packet.rotation);keyframe_pose->publish(pose);
      sensor_msgs::msg::PointCloud cloud;cloud.header=pose.header;
      for(size_t i=0;i<packet.points.size();++i) {
        geometry_msgs::msg::Point32 point;point.x=packet.points[i].x();point.y=packet.points[i].y();point.z=packet.points[i].z();
        cloud.points.push_back(point);
        sensor_msgs::msg::ChannelFloat32 channel;
        channel.values.assign(packet.observations[i].begin(),packet.observations[i].end());cloud.channels.push_back(channel);
      }
      keyframe_points->publish(cloud);
    };
    platform_extrinsic=[this](double t,const Eigen::Vector3d& p,const Eigen::Matrix3d& r) {
      auto msg=pose_message(t,p,r);msg.header.frame_id="imu_link";msg.child_frame_id="camera_optical";extrinsic->publish(msg);
    };
    imu=create_subscription<sensor_msgs::msg::Imu>("/uav001/imu/data",rclcpp::SensorDataQoS().keep_last(512),[this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
      const double t=seconds(msg->header.stamp);if(t<=previous_imu) return;
      Eigen::Vector3d a(msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z),g(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
      if(!a.allFinite() || !g.allFinite()) return;
      latest_imu=previous_imu=t;estimator.inputIMU(t,a,g);flush();
    });
    camera=create_subscription<sensor_msgs::msg::Image>("/uav001/camera/image_raw",rclcpp::SensorDataQoS().keep_last(8),[this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
      const double t=seconds(msg->header.stamp);if(t<=previous_image) return;
      previous_image=t;images.push_back(msg);if(images.size()>8) {images.pop_front();RCLCPP_WARN(get_logger(),"camera queue overrun");}flush();
    });
    timer=create_wall_timer(std::chrono::seconds(1),[this] {
      std_msgs::msg::String msg;msg.data="{\"backend\":\"vins_fusion\",\"input_ready\":true,\"tracking_state\":"+std::to_string(int(estimator.solver_flag))+",\"inertial_initialized\":"+(estimator.solver_flag==Estimator::NON_LINEAR?"true":"false")+",\"loop_keyframe_transport\":true}";state->publish(msg);
    });
  }
  static nav_msgs::msg::Odometry pose_message(double t,const Eigen::Vector3d& p,const Eigen::Matrix3d& r) {
    nav_msgs::msg::Odometry msg;msg.header.stamp=rclcpp::Time(int64_t(t*1e9));
    msg.header.frame_id="vins_fusion_odom";msg.child_frame_id="imu_link";
    msg.pose.pose.position.x=p.x();msg.pose.pose.position.y=p.y();msg.pose.pose.position.z=p.z();
    const Eigen::Quaterniond q(r);msg.pose.pose.orientation.x=q.x();msg.pose.pose.orientation.y=q.y();
    msg.pose.pose.orientation.z=q.z();msg.pose.pose.orientation.w=q.w();return msg;
  }
  void flush() {
    while(!images.empty() && seconds(images.front()->header.stamp)<=latest_imu) {
      const auto msg=images.front();images.pop_front();
      try {auto gray=cv_bridge::toCvCopy(msg,"mono8");estimator.inputImage(seconds(msg->header.stamp),gray->image);}
      catch(const cv_bridge::Exception& error) {RCLCPP_ERROR(get_logger(),"%s",error.what());}
    }
  }
  ~VinsNode() {platform_odometry={};platform_landmarks={};platform_keyframe={};platform_extrinsic={};}
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);if(argc<2) return 2;
  try {rclcpp::spin(std::make_shared<VinsNode>(argv[1]));}
  catch(const std::exception& error) {std::fprintf(stderr,"VINS adapter: %s\n",error.what());return 1;}
  rclcpp::shutdown();return 0;
}
