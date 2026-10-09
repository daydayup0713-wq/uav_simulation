// Official FAST-Planner kinodynamic search + ESDF B-spline optimization, ROS2 interface port.
#include "request.hpp"
#include <path_searching/kinodynamic_astar.h>
#include <bspline_opt/bspline_optimizer.h>
Json optimize(const Json& j,rclcpp::Node::SharedPtr node) {
  auto map=std::make_shared<ObservedMap>(j);auto env=std::make_shared<fast_planner::EDTEnvironment>(map);
  ros::NodeHandle nh(node);fast_planner::KinodynamicAstar search;search.setParam(nh);search.setEnvironment(env);search.init();search.reset();
  const auto& initial=j.at("initial");
  int status=search.search(vec(initial.at("position")),vec(initial.at("velocity")),vec(initial.at("acceleration")),vec(j.at("goal")),Vec::Zero(),true);
  if(status==fast_planner::KinodynamicAstar::NO_PATH) {search.reset();status=search.search(vec(initial.at("position")),vec(initial.at("velocity")),vec(initial.at("acceleration")),vec(j.at("goal")),Vec::Zero(),false);}
  if(status!=fast_planner::KinodynamicAstar::REACH_END && status!=fast_planner::KinodynamicAstar::NEAR_END)
    throw std::runtime_error("FAST kinodynamic search did not reach target");
  const auto path=search.getKinoTraj(.1);if(path.size()<2)throw std::runtime_error("FAST kinodynamic path empty");
  Json seeded=j;seeded["guide"]=Json::array();
  const size_t step=std::max<size_t>(1,(path.size()+99)/100);
  for(size_t i=0;i<path.size();i+=step)seeded["guide"].push_back({path[i].x(),path[i].y(),path[i].z()});
  seeded["guide"][0]=j.at("initial").at("position");seeded["guide"].push_back(j.at("goal"));
  const double dt=interval(seeded);Eigen::MatrixXd q=seed(seeded,dt);
  fast_planner::BsplineOptimizer opt;opt.setParam(nh);opt.setEnvironment(env);
  const auto out=opt.BsplineOptimizeTraj(q.transpose(),dt,fast_planner::BsplineOptimizer::NORMAL_PHASE,1,1);
  Json reply=result(j,spline_segments(out.transpose(),dt),"official FAST-Planner kinodynamic A* + ESDF + NLopt B-spline optimizer");
  reply["kinodynamic_status"]=status;reply["kinodynamic_samples"]=path.size();return reply;
}
int main(int argc,char** argv) {
  if(argc!=3)return 2;Json j,reply;
  try {
    j=read_json(argv[1]);if(j.at("backend")!="fast_planner" || j.at("frame")!="odom" || j.at("schema")!=1)throw std::runtime_error("invalid request");
    rclcpp::init(0,nullptr);rclcpp::NodeOptions options;
    options.parameter_overrides({{"search/max_tau",.6},{"search/init_max_tau",.6},{"search/max_vel",.45},
      {"search/max_acc",.45},{"search/w_time",10.},{"search/horizon",30.},{"search/resolution_astar",.15},
      {"search/time_resolution",.2},{"search/lambda_heu",2.},{"search/allocate_num",100000},{"search/check_num",12},
      {"search/optimistic",false},{"optimization/lambda1",1.},{"optimization/lambda2",8.},{"optimization/lambda3",1.},
      {"optimization/lambda4",1.},{"optimization/lambda5",1.},{"optimization/lambda6",0.},{"optimization/lambda7",1.},
      {"optimization/lambda8",0.},{"optimization/dist0",.15},{"optimization/max_vel",.45},{"optimization/max_acc",.45},
      {"optimization/visib_min",0.},{"optimization/dlmin",0.},{"optimization/wnl",0.},
      {"optimization/max_iteration_num1",200},{"optimization/max_iteration_num2",200},{"optimization/max_iteration_num3",200},
      {"optimization/max_iteration_num4",200},{"optimization/max_iteration_time1",.2},{"optimization/max_iteration_time2",.5},
      {"optimization/max_iteration_time3",.5},{"optimization/max_iteration_time4",.5},{"optimization/algorithm1",11},
      {"optimization/algorithm2",11},{"optimization/order",3}});
    reply=optimize(j,std::make_shared<rclcpp::Node>("fast_planner_core_adapter",options));
  }catch(const std::exception& error) {reply={{"schema",1},{"success",false},{"reason",error.what()}};}
  std::ofstream(argv[2])<<reply.dump(2)<<'\n';if(rclcpp::ok())rclcpp::shutdown();return reply.value("success",false)?0:1;
}
