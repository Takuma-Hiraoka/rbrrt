#pragma once
#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <rbrrt/rbrrt_state.h>
#include <rbrrt/rbrrt.h>

namespace rbrrt {
  enum class IKState
    {
      DETACH, // 触れている接触を離す
      SWING,  // 離れている接触を次に触れる点近くまで移動させる
      ATTACH,  // 離れている接触を触れさせる
      ROOT // ルートリンクを移動させる. 接触の切り替えは行わない
    };
  // targetLimbのconfigurationDatabaseで, eePoseと外部環境との距離が一定値以下のものをhの順番で接触させられるかどうか試していく.接触させるIKが解ければtrueを返す.
  bool searchLimbContact(const std::shared_ptr<rbrrt::RBRRTParam>& param,
                         const std::shared_ptr<rbrrt::Limb> targetLimb,
                         const std::vector<std::shared_ptr<Contact> >& stopContacts,
                         const std::shared_ptr<ik_constraint2::PositionConstraint>& rootConstraint,
                         std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath /* out */);
  bool solveContactIK(const std::shared_ptr<rbrrt::RBRRTParam>& param,
                      const std::vector<cnoid::LinkPtr> variables,
                      const std::vector<std::shared_ptr<Contact> >& stopContacts,
                      const std::shared_ptr<Contact>& nextContact,
                      const std::shared_ptr<ik_constraint2::PositionConstraint>& rootConstraint,
                      const IKState ikstate);
  void calcLevelLinks(const cnoid::LinkPtr inputLink,
                      int level, // input
                      std::vector<cnoid::LinkPtr>& targetLinks // output
                      );
  void calcIgnoreBoundingBox(const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints,
                             const std::shared_ptr<Contact>& contact,
                             int level
                             );
  // mapからsolvabillity_threshold以上にIKが解けるgridの中心点を集めて凸包を作る
  cnoid::SgShapePtr generateMeshFromReachabilityMap(const std::shared_ptr<reachability_map_visualizer::ReachabilityMap>& map,
                                                    double solvability_threshold);
  void generateConfigurationDatabase(const std::shared_ptr<rbrrt::RBRRTParam>& param, double sample_num);
  void writeConfigurationDatabase(std::string outputFilePath, std::vector<ConfigurationData> data);
  void readConfigurationDatabase(std::string inputFilePath, std::vector<ConfigurationData>& data);
}
