#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <ros/package.h>
#include "jaxon_common.h"

namespace rbrrt_sample {
  void rmap(){
    std::shared_ptr<rbrrt::Environment> environment = std::make_shared<rbrrt::Environment>();
    std::shared_ptr<rbrrt::RBRRTParam> rbrrtParam = std::make_shared<rbrrt::RBRRTParam>();
    generateJAXON(environment,rbrrtParam);
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    std::shared_ptr<reachability_map_visualizer::ReachabilityMapParam> rmapParam = std::make_shared<reachability_map_visualizer::ReachabilityMapParam>();
    rmapParam->robot = rbrrtParam->robot;
    // joint limit
    for(int i=0;i<rmapParam->robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = rmapParam->robot->joint(i);
      rmapParam->constraints.push_back(constraint);
    }

    {
      int index = 0;
      reachability_map_visualizer::EndEffector ee;
      ee.parent = rbrrtParam->limbs[index]->eeParentLink;
      ee.localPose = rbrrtParam->limbs[index]->eeLocal;
      rmapParam->endEffectors.push_back(ee);
      for (int i=0;i<rbrrtParam->limbs[index]->joints.size();i++) {
        rmapParam->variables.push_back(rbrrtParam->limbs[index]->joints[i]);
      }
    }

    rmapParam->posResolution = 0.05;
    rmapParam->pikParam.maxIteration = 30;
    rmapParam->testPerGrid = 50;
    rmapParam->origin = cnoid::Vector3(0.0, -0.1,0.5);
    rmapParam->size = cnoid::Vector3(2.0,2.0,2.0);
    rmapParam->weight[5] = 0.0;
    std::shared_ptr<reachability_map_visualizer::ReachabilityMap> map = std::make_shared<reachability_map_visualizer::ReachabilityMap>();
    reachability_map_visualizer::createMap(rmapParam, map);
    reachability_map_visualizer::writeMap(ros::package::getPath("rbrrt_sample") + "/config/rmap_rarm.yaml",map);
    viewer->objects(rmapParam->robot);
    reachability_map_visualizer::visualizeMap(map, viewer);
    viewer->drawObjects();

  }
}
