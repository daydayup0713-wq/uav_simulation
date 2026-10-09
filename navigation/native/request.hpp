// Platform transport only. Optimizer implementations are compiled from locked upstream sources.
#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <vector>
#include <cmath>
using Json=nlohmann::json;
using Vec=Eigen::Vector3d;
inline Vec vec(const Json& j) {
  if(!j.is_array() || j.size()!=3) throw std::runtime_error("three coordinates required");
  Vec v(j[0].get<double>(),j[1].get<double>(),j[2].get<double>());
  if(!v.allFinite()) throw std::runtime_error("nonfinite coordinates");return v;
}
inline Json read_json(const char* path) {std::ifstream f(path);if(!f) throw std::runtime_error("request missing");Json j;f>>j;return j;}
struct ObservedMap {
  Eigen::Vector3i shape;Vec lower;double resolution;std::vector<unsigned char> free;
  std::vector<double> distance;
  explicit ObservedMap(const Json& j):lower(vec(j.at("lower"))),resolution(j.at("resolution")) {
    shape=vec(j.at("shape")).cast<int>();
    const auto count=static_cast<long long>(shape.x())*shape.y()*shape.z();
    if((shape.array()<2).any() || (shape.array()>512).any() || count>2000000
        || static_cast<long long>(shape.x()+4)*(shape.y()+4)*(shape.z()+4)>2000000
        || !std::isfinite(resolution) || resolution<=0)
      throw std::runtime_error("invalid bounded map");
    auto load=[&](const std::string& path,void* output,size_t bytes) {
      std::ifstream f(path,std::ios::binary);if(!f.read(static_cast<char*>(output),bytes) || f.peek()!=EOF)
        throw std::runtime_error("map byte count mismatch");
    };
    free.resize(count);distance.resize(count);
    load(j.at("free_file"),free.data(),free.size());
    load(j.at("distance_file"),distance.data(),distance.size()*sizeof(double));
    for(size_t i=0;i<free.size();++i) if(free[i]>1 || !std::isfinite(distance[i]) || distance[i]<0)
      throw std::runtime_error("invalid observed map values");
  }
  size_t offset(const Eigen::Vector3i& i) const {return (static_cast<size_t>(i.x())*shape.y()+i.y())*shape.z()+i.z();}
  int occupied(const Vec& p) const {
    if(!p.allFinite())return 1;
    Eigen::Vector3i i=((p-lower)/resolution).array().floor().cast<int>();
    return (i.array()<0).any() || (i.array()>=shape.array()).any() || !free[offset(i)] ? 1:0;
  }
  void field(const Vec& p,double& value,Vec& gradient) const {
    value=0.;gradient.setZero();if(occupied(p))return;
    const Vec coordinate=(p-lower)/resolution-Vec::Constant(.5);
    const Eigen::Vector3i base=coordinate.array().floor().cast<int>();
    const Vec u=coordinate-base.cast<double>();
    for(int x=0;x<2;++x)for(int y=0;y<2;++y)for(int z=0;z<2;++z) {
      const Eigen::Vector3i i=base+Eigen::Vector3i(x,y,z);
      if((i.array()<0).any() || (i.array()>=shape.array()).any())continue;
      const Vec w(x?u.x():1-u.x(),y?u.y():1-u.y(),z?u.z():1-u.z());
      const double d=distance[offset(i)];value+=d*w.x()*w.y()*w.z();
      gradient+=d/resolution*Vec((x?1.:-1.)*w.y()*w.z(),(y?1.:-1.)*w.x()*w.z(),(z?1.:-1.)*w.x()*w.y());
    }
  }
};
inline Eigen::MatrixXd seed(const Json& j,double dt) {
  std::vector<Vec> route;for(const auto& p:j.at("guide")) route.push_back(vec(p));
  if(route.size()<2 || route.size()>128)throw std::runtime_error("invalid guide");
  std::vector<double> ends{0.};for(size_t i=1;i<route.size();++i)ends.push_back(ends.back()+(route[i]-route[i-1]).norm());
  if(ends.back()<1e-5)throw std::runtime_error("stationary guide");
  const int n=std::max(12,std::min(100,static_cast<int>(std::ceil(ends.back()/.25))+6));
  Eigen::MatrixXd q(3,n);
  for(int i=0;i<n;++i) {
    const double progress=ends.back()*std::max(0.,std::min(1.,(i-2.)/(n-5.)));
    size_t k=1;while(k<ends.size()-1 && ends[k]<progress)++k;
    const double f=(progress-ends[k-1])/std::max(1e-9,ends[k]-ends[k-1]);
    q.col(i)=route[k-1]*(1-f)+route[k]*f;
  }
  const Vec p=vec(j.at("initial").at("position")),v=vec(j.at("initial").at("velocity")),a=vec(j.at("initial").at("acceleration"));
  q.col(1)=p-a*dt*dt/6.;q.col(0)=q.col(1)+a*dt*dt/2.-v*dt;q.col(2)=q.col(1)+a*dt*dt/2.+v*dt;
  for(int i=n-3;i<n;++i)q.col(i)=vec(j.at("goal"));return q;
}
inline double interval(const Json& j) {
  double length=0.;const auto& g=j.at("guide");for(size_t i=1;i<g.size();++i)length+=(vec(g[i])-vec(g[i-1])).norm();
  const int n=std::max(12,std::min(100,static_cast<int>(std::ceil(length/.25))+6));
  return std::max(.5,length/((n-5)*j.at("limits").at("speed").get<double>()*.7))*j.value("time_scale",1.);
}
inline Json spline_segments(const Eigen::MatrixXd& q,double dt) {
  Json segments=Json::array();
  for(int i=0;i<q.cols()-3;++i) {
    Eigen::Matrix<double,3,6> c=Eigen::Matrix<double,3,6>::Zero();
    c.col(0)=(q.col(i)+4*q.col(i+1)+q.col(i+2))/6.;
    c.col(1)=(q.col(i+2)-q.col(i))/2.;
    c.col(2)=(q.col(i)-2*q.col(i+1)+q.col(i+2))/2.;
    c.col(3)=(-q.col(i)+3*q.col(i+1)-3*q.col(i+2)+q.col(i+3))/6.;
    Json rows=Json::array();for(int a=0;a<3;++a) {Json row=Json::array();for(int k=0;k<6;++k)row.push_back(c(a,k));rows.push_back(row);}
    segments.push_back({{"duration",dt},{"coefficients",rows},{"yaw_coefficients",{0,0,0,0,0,0}}});
  }return segments;
}
inline Json result(const Json& request,const Json& segments,const std::string& method) {
  return {{"schema",1},{"success",true},{"backend",request.at("backend")},{"map_sha256",request.at("map_sha256")},
    {"method",method},{"trajectory",{{"schema",1},{"frame","odom"},{"limits",request.at("limits")},{"segments",segments}}}};
}
