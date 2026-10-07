#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <gtsam/base/serialization.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <glim/mapping/sub_map.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

// Read the actual optimized native graph, rather than infer accepted loops from candidates.
int main(int argc, char** argv) {
  if (argc != 4) { std::cerr << "usage: uav_map_export dump points.xyz graph-report.json\n"; return 2; }
  try {
    const std::filesystem::path dump(argv[1]);
    gtsam::NonlinearFactorGraph graph;
    if (!gtsam::deserializeFromBinaryFile((dump / "graph.bin").string(), graph))
      throw std::runtime_error("failed to deserialize native graph");
    std::vector<std::pair<size_t, size_t>> loops;
    for (const auto& f : graph) {
      const auto factor = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(f);
      if (!factor) continue;
      const gtsam::Symbol a(factor->key1()), b(factor->key2());
      if (a.chr()=='x' && b.chr()=='x' && std::max(a.index(),b.index())-std::min(a.index(),b.index())>1)
        loops.emplace_back(a.index(),b.index());
    }
    std::vector<std::filesystem::path> submaps;
    for (const auto& item : std::filesystem::directory_iterator(dump)) {
      auto name = item.path().filename().string();
      if (item.is_directory() && name.size()==6 && std::all_of(name.begin(), name.end(), ::isdigit))
        submaps.push_back(item.path());
    }
    std::sort(submaps.begin(),submaps.end());
    if (submaps.empty()) throw std::runtime_error("no native submaps");
    std::ofstream xyz(argv[2]);
    xyz.precision(10);
    size_t count=0;
    for (const auto& path : submaps) {
      auto submap = glim::SubMap::load(path.string());
      if (!submap || !submap->frame || !submap->frame->points) throw std::runtime_error("invalid submap");
      for (size_t i=0; i<submap->frame->size(); ++i) {
        const Eigen::Vector4d p = submap->T_world_origin * submap->frame->points[i];
        if (!p.allFinite()) throw std::runtime_error("non-finite map point");
        xyz << p.x() << ' ' << p.y() << ' ' << p.z() << '\n'; ++count;
      }
    }
    if (!xyz) throw std::runtime_error("map write failed");
    std::ofstream report(argv[3]);
    report << "{\"submaps\":" << submaps.size() << ",\"points\":" << count
           << ",\"factors\":" << graph.size() << ",\"accepted_nonadjacent_loop_factors\":" << loops.size() << ",\"loop_edges\":[";
    for (size_t i=0;i<loops.size();++i) { if(i) report << ','; report << '[' << loops[i].first << ',' << loops[i].second << ']'; }
    report << "]}\n";
    if (!report) throw std::runtime_error("report write failed");
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
