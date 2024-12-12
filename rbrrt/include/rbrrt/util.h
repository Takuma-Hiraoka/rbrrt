#pragma once
#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <rbrrt/rbrrt_state.h>
#include <rbrrt/rbrrt.h>

namespace rbrrt {
  // mapからsolvabillity_threshold以上にIKが解けるgridの中心点を集めて凸包を作る
  cnoid::SgShapePtr generateMeshFromReachabilityMap(const std::shared_ptr<reachability_map_visualizer::ReachabilityMap>& map,
                                                    double solvability_threshold);
  void generateConfigurationDatabase(const std::shared_ptr<rbrrt::RBRRTParam>& param, double sample_num);
  void writeConfigurationDatabase(std::string outputFilePath, std::vector<ConfigurationData> data);
  void readConfigurationDatabase(std::string inputFilePath, std::vector<ConfigurationData>& data);
}
