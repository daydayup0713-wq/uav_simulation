#pragma once
#include "request.hpp"
#include <iostream>
using std::cout;
using std::endl;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::string;
using std::max;
namespace fast_planner {
class SDFMap {
  std::shared_ptr<ObservedMap> map;
public:
  explicit SDFMap(std::shared_ptr<ObservedMap> value):map(std::move(value)) {}
  int getInflateOccupancy(const Vec& p) const {return map->occupied(p);}
  void getRegion(Vec& origin,Vec& size) const {origin=map->lower;size=map->shape.cast<double>()*map->resolution;}
};
class EDTEnvironment {
  std::shared_ptr<ObservedMap> map;
public:
  using Ptr=std::shared_ptr<EDTEnvironment>;
  std::shared_ptr<SDFMap> sdf_map_;
  explicit EDTEnvironment(std::shared_ptr<ObservedMap> value):map(value),sdf_map_(std::make_shared<SDFMap>(value)) {}
  void evaluateEDTWithGrad(const Vec& p,double,double& distance,Vec& gradient) const {map->field(p,distance,gradient);}
};
}
