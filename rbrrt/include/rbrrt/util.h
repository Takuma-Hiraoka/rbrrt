#pragma once
#include <reachability_map_visualizer/reachability_map_visualizer.h>

namespace rbrrt {
  // mapからsolvabillity_threshold以上にIKが解けるgridの中心点を集めて凸包を作る
  cnoid::SgShapePtr generateMeshFromReachabilityMap(const std::shared_ptr<reachability_map_visualizer::ReachabilityMap>& map,
                                                    double solvability_threshold);
}
