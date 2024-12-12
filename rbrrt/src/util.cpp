#include <rbrrt/util.h>
#include <choreonoid_qhull/choreonoid_qhull.h>

namespace rbrrt {
  cnoid::SgShapePtr generateMeshFromReachabilityMap(const std::shared_ptr<reachability_map_visualizer::ReachabilityMap>& map,
                                                    double solvability_threshold) {
    std::vector<Eigen::Vector3d> vertices;
    for (int i=0;i<map->reachabilityMap.size();i++) {
      if (map->reachabilityMap[i].second >= solvability_threshold) vertices.push_back(map->reachabilityMap[i].first);
    }
    return choreonoid_qhull::generateMeshFromConvexHull(vertices);
  }
}
