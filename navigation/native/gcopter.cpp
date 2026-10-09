// Certified observed-space corridors feed the real GCOPTER nonlinear MINCO optimizer.
#include "request.hpp"
#include <gcopter/geo_utils.hpp>
#include <gcopter/gcopter.hpp>
int main(int argc,char** argv) {
  if(argc!=3)return 2;Json j,reply;
  try {
    j=read_json(argv[1]);if(j.at("backend")!="gcopter" || j.at("frame")!="odom" || j.at("schema")!=1)throw std::runtime_error("invalid request");
    ObservedMap map(j);Eigen::Matrix3d head,tail;const auto& initial=j.at("initial");
    head.col(0)=vec(initial.at("position"));head.col(1)=vec(initial.at("velocity"));head.col(2)=vec(initial.at("acceleration"));
    tail.setZero();tail.col(0)=vec(j.at("goal"));
    std::vector<Eigen::MatrixX4d> corridor;
    for(const auto& box:j.at("corridors")) {
      const Vec lower=vec(box.at("lower")),upper=vec(box.at("upper"));Eigen::MatrixX4d h(6,4);h.setZero();
      for(int a=0;a<3;++a) {h(2*a,a)=1.;h(2*a,3)=-upper[a];h(2*a+1,a)=-1.;h(2*a+1,3)=lower[a];}corridor.push_back(h);
    }
    if(corridor.empty() || corridor.size()>256)throw std::runtime_error("bounded safe corridors required");
    gcopter::GCOPTER_PolytopeSFC opt;Eigen::VectorXd bounds(5),weights(5),physics(6);
    bounds<<j.at("limits").at("speed").get<double>()*.9,2.,.045,19.4,20.8;
    weights<<1e5,1e4,1e3,1e3,1e3;physics<<2.05,9.81,.0,.0,.0,.0001;
    const double scale=j.value("time_scale",1.);
    if(!opt.setup(10./std::pow(scale,4),head,tail,corridor,.8,1e-4,16,bounds,weights,physics))throw std::runtime_error("GCOPTER corridor setup failed");
    Trajectory<5> trajectory;const double cost=opt.optimize(trajectory,1e-5);
    if(!std::isfinite(cost) || trajectory.getPieceNum()==0)throw std::runtime_error("GCOPTER optimization failed");
    Json segments=Json::array();
    for(int i=0;i<trajectory.getPieceNum();++i) {
      const auto& piece=trajectory[i];const double dt=piece.getDuration();Json rows=Json::array();
      for(int a=0;a<3;++a) {Json row=Json::array();for(int k=0;k<6;++k)row.push_back(piece.getCoeffMat()(a,5-k)*std::pow(dt,k));rows.push_back(row);}
      segments.push_back({{"duration",dt},{"coefficients",rows},{"yaw_coefficients",{0,0,0,0,0,0}}});
    }
    reply=result(j,segments,"certified observed safe corridors + official GCOPTER_PolytopeSFC / MINCO_S3NU");reply["cost"]=cost;reply["corridors"]=corridor.size();
  }catch(const std::exception& error) {reply={{"schema",1},{"success",false},{"reason",error.what()}};}
  std::ofstream(argv[2])<<reply.dump(2)<<'\n';return reply.value("success",false)?0:1;
}
