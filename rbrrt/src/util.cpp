#include <rbrrt/util.h>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <cnoid/YAMLWriter>
#include <cnoid/YAMLReader>

namespace rbrrt {
  cnoid::SgShapePtr generateMeshFromReachabilityMap(const std::shared_ptr<reachability_map_visualizer::ReachabilityMap>& map,
                                                    double solvability_threshold) {
    std::vector<Eigen::Vector3d> vertices;
    for (int i=0;i<map->reachabilityMap.size();i++) {
      if (map->reachabilityMap[i].second >= solvability_threshold) vertices.push_back(map->reachabilityMap[i].first);
    }
    return choreonoid_qhull::generateMeshFromConvexHull(vertices);
  }
  void randomFrame(const std::vector<cnoid::LinkPtr>& links, std::vector<double>& frame){
    frame.clear();
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        std::uniform_real_distribution<> dist(links[l]->q_lower(),links[l]->q_upper());
        frame.push_back(dist(engine));
      }else if(links[l]->isFreeJoint()) { // 適当
        frame.push_back(links[l]->p()[0]);
        frame.push_back(links[l]->p()[1]);
        frame.push_back(links[l]->p()[2]);
        cnoid::Quaternion q(links[l]->R());
        frame.push_back(q.x());
        frame.push_back(q.y());
        frame.push_back(q.z());
        frame.push_back(q.w());
      }
    }
  }
  struct CompareConfigurationData {
    bool operator() (const rbrrt::ConfigurationData& a, const rbrrt::ConfigurationData& b) const {
      return a.h_w > b.h_w; // hが大きい順にソート
    }
  };
  void generateConfigurationDatabase(const std::shared_ptr<rbrrt::RBRRTParam>& param, double sample_num) {
    for (int l=0;l<param->limbs.size();l++) {
      param->limbs[l]->configurationDatabase.clear();
      for(int i=0;i<sample_num;i++) {
        std::cerr << i << std::endl;
        rbrrt::ConfigurationData data;
        std::vector<double> frame;
        randomFrame(param->limbs[l]->joints, frame);
        data.angles = frame;
        global_inverse_kinematics_solver::frame2Link(frame,param->limbs[l]->joints);
        param->robot->calcForwardKinematics(false);
        data.eePos = (param->robot->rootLink()->R()).transpose() * (param->limbs[l]->eeParentLink->translation() + param->limbs[l]->eeParentLink->R() * param->limbs[l]->eeLocal.translation() - param->robot->rootLink()->translation());

        // jacobianを求める
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = param->limbs[l]->eeParentLink;
        constraint->A_localpos() = param->limbs[l]->eeLocal;
        constraint->B_link() = nullptr;
        constraint->eval_link() = nullptr;
        constraint->updateJacobian(param->limbs[l]->joints);
        Eigen::MatrixXd jacobian = Eigen::MatrixXd(constraint->getJacobian());
        data.h_w = std::sqrt((jacobian * jacobian.transpose()).determinant());
        param->limbs[l]->configurationDatabase.push_back(data);
      }
      std::sort(param->limbs[l]->configurationDatabase.begin(), param->limbs[l]->configurationDatabase.end(), CompareConfigurationData());
    }
  }
  void writeConfigurationDatabase(std::string outputFilePath, std::vector<ConfigurationData> data) {
    cnoid::YAMLWriter writer(outputFilePath);
    writer.startMapping();
    writer.putKey("configurationDatabase");
    writer.startListing();
    for (int i=0; i<data.size(); i++) {
      writer.startMapping();
      writer.startListing();
      {
        writer.putKeyValue("h_w",data[i].h_w);
        writer.putKey("eePos");
        writer.startFlowStyleListing();
        for (int j=0; j<3; j++) writer.putScalar(data[i].eePos[j]);
        writer.endListing();
        writer.putKey("angles");
        writer.startFlowStyleListing();
        for (int j=0; j<data[i].angles.size(); j++) writer.putScalar(data[i].angles[j]);
        writer.endListing();
      }
      writer.endListing();
      writer.endMapping();
    }
    writer.endListing();
    writer.endMapping();
  }
  void readConfigurationDatabase(std::string inputFilePath, std::vector<ConfigurationData>& data) {
    data.clear();
    cnoid::YAMLReader reader;
    cnoid::MappingPtr node;
    try {
      node = reader.loadDocument(inputFilePath)->toMapping();
    } catch(const cnoid::ValueNode::Exception& ex) {
      std::cerr << "cannot load configuration database file" << std::endl;;
      return;
    }

    // load
    cnoid::Listing* configurationDatabase = node->findListing("configurationDatabase");
    if (!configurationDatabase->isValid()) {
      std::cerr << "cannot load configuration database file value" << std::endl;;
      return;
    } else {
      for (int i=0; i<configurationDatabase->size();i++) {
        cnoid::Mapping* info = configurationDatabase->at(i)->toMapping();
        ConfigurationData value;
        cnoid::ValueNodePtr h_w_ = info->extract("h_w");
        if (h_w_ && h_w_->isScalar()){
          value.h_w = h_w_->toDouble();
        }
        cnoid::ValueNodePtr eePos_ = info->extract("eePos");
        if(eePos_){
          cnoid::ListingPtr eePosTmp = eePos_->toListing();
          if(eePosTmp->size()==3){
            value.eePos = cnoid::Vector3(eePosTmp->at(0)->toDouble(), eePosTmp->at(1)->toDouble(), eePosTmp->at(2)->toDouble());
          }
        }
        cnoid::ValueNodePtr angles_ = info->extract("angles");
        if(angles_){
          cnoid::ListingPtr anglesTmp = angles_->toListing();
          value.angles.resize(anglesTmp->size());
          for (int j=0;j<value.angles.size();j++) value.angles[j] = anglesTmp->at(j)->toDouble();
        }
        data.push_back(value);
      }
    }

  }
}
