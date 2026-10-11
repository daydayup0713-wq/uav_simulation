#pragma once
#include <Eigen/Core>
#include <memory>
#include <stdexcept>
namespace fast_planner {
// Static-map adapter has no predictor. An accidental dynamic request fails.
class ObjPredictor {
public:
  using Ptr=std::shared_ptr<ObjPredictor>;
  int getObjNums() const {throw std::runtime_error("dynamic object predictor unavailable");}
  Eigen::Vector3d evaluateConstVel(int,double) const {throw std::runtime_error("dynamic object predictor unavailable");}
};
}
