#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <string>

// Both source poses must be read in the same, currently optimized map gauge.
// A cached pose from an older gauge would turn a loop correction into motion.
class ContinuousPose {
  Eigen::Matrix4f local_=Eigen::Matrix4f::Identity();
  Eigen::Matrix4f correction_=Eigen::Matrix4f::Identity();
  double last_stamp_=-1.;
  int map_id_=-1;
  bool failed_=false;
  std::string reason_;
  bool fail(const char* reason) {failed_=true;reason_=reason;return false;}
public:
  bool update(double stamp,const Eigen::Matrix4f& current_global,
              const Eigen::Matrix4f& previous_global,double previous_stamp,int map_id) {
    if(failed_) return false;
    if(!std::isfinite(stamp) || stamp<=0 || !current_global.allFinite() || map_id<0)
      return fail("invalid metric pose");
    if(last_stamp_<0) {
      local_=current_global;last_stamp_=stamp;map_id_=map_id;return true;
    }
    if(map_id!=map_id_) return fail("map changed; new ground-selected session required");
    if(stamp<=last_stamp_ || !std::isfinite(previous_stamp) ||
       std::abs(previous_stamp-last_stamp_)>1e-6 || !previous_global.allFinite())
      return fail("tracking frame gap; new ground-selected session required");
    const Eigen::Matrix4f next=local_*(previous_global.inverse()*current_global);
    const Eigen::Matrix4f correction=current_global*next.inverse();
    if(!next.allFinite() || !correction.allFinite()) return fail("nonfinite relative motion");
    local_=next;correction_=correction;last_stamp_=stamp;return true;
  }
  const Eigen::Matrix4f& local() const {return local_;}
  const Eigen::Matrix4f& correction() const {return correction_;}
  bool failed() const {return failed_;}
  const std::string& reason() const {return reason_;}
};
