#pragma once
#include "request.hpp"
// The original EGO optimizer reads this immutable, already body-inflated map.
class GridMap {
  std::shared_ptr<ObservedMap> map;
public:
  using Ptr=std::shared_ptr<GridMap>;
  explicit GridMap(std::shared_ptr<ObservedMap> value):map(std::move(value)) {}
  int getInflateOccupancy(const Eigen::Vector3d& p) const {return map->occupied(p);}
  double getResolution() const {return map->resolution;}
};
