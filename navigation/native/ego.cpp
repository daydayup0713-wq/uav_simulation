// Official EGO-Swarm ROS2 optimizer, single drone; no upstream fake-drone executor.
#include "request.hpp"
#include <bspline_opt/bspline_optimizer.h>
Json optimize(const Json& j,rclcpp::Node::SharedPtr node) {
  auto map=std::make_shared<ObservedMap>(j);auto grid=std::make_shared<GridMap>(map);
  ego_planner::BsplineOptimizer opt;opt.setParam(node);opt.setEnvironment(grid);opt.setDroneId(0);
  ego_planner::SwarmTrajData swarm;opt.setSwarmTrajs(&swarm);
  opt.a_star_=std::make_shared<AStar>();opt.a_star_->initGridMap(grid,map->shape+Eigen::Vector3i::Constant(4));
  const double dt=interval(j);auto q=seed(j,dt);const auto boundary=q;
  opt.setLocalTargetPt(vec(j.at("goal")));opt.initControlPoints(q,true);
  Eigen::MatrixXd rebound;
  if(!opt.BsplineOptimizeTrajRebound(rebound,dt))throw std::runtime_error("EGO rebound optimizer rejected seed");
  // Swarm rebound uses a free endpoint. Feed exact endpoint p/v/a to its
  // existing fixed-boundary refine API; never rewrite the optimizer math.
  rebound.leftCols(3)=boundary.leftCols(3);rebound.rightCols(3)=boundary.rightCols(3);
  ego_planner::UniformBspline spline(rebound,3,dt);
  opt.ref_pts_.clear();for(int i=0;i<=rebound.cols()-3;++i)opt.ref_pts_.push_back(spline.evaluateDeBoorT(i*dt));
  Eigen::MatrixXd refined;
  if(!opt.BsplineOptimizeTrajRefine(rebound,dt,refined))throw std::runtime_error("EGO fixed-boundary refine failed");
  return result(j,spline_segments(refined,dt),"official EGO-Swarm ROS2 rebound + fixed-boundary refine; drone_id=0");
}
int main(int argc,char** argv) {
  if(argc!=3)return 2;Json j,reply;
  try {
    j=read_json(argv[1]);if(j.at("backend")!="ego" || j.at("frame")!="odom" || j.at("schema")!=1)throw std::runtime_error("invalid request");
    rclcpp::init(0,nullptr);rclcpp::NodeOptions options;
    options.parameter_overrides({{"optimization/lambda_smooth",1.},{"optimization/lambda_collision",8.},
      {"optimization/lambda_feasibility",1.},{"optimization/lambda_fitness",1.},{"optimization/dist0",.15},
      {"optimization/swarm_clearance",.5},{"optimization/max_vel",j.at("limits").at("speed").get<double>()*.9},
      {"optimization/max_acc",j.at("limits").at("acceleration").get<double>()*.9}});
    reply=optimize(j,std::make_shared<rclcpp::Node>("ego_core_adapter",options));
  } catch(const std::exception& error) {reply={{"schema",1},{"success",false},{"reason",error.what()}};}
  std::ofstream(argv[2])<<reply.dump(2)<<'\n';if(rclcpp::ok())rclcpp::shutdown();return reply.value("success",false)?0:1;
}
