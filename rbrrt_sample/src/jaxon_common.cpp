#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <iostream>
#include <ros/package.h>
#include "jaxon_common.h"

namespace rbrrt_sample {
  void generateJAXON(const std::shared_ptr<rbrrt::RBRRTParam>& param
                     ) {
    cnoid::BodyLoader bodyLoader;
    param->robot = bodyLoader.load(ros::package::getPath("jvrc_models") + "/JAXON_JVRC/JAXON_JVRCmain.wrl");
    if(!(param->robot)) std::cerr << "!robot" << std::endl;
    param->robot->rootLink()->p() = cnoid::Vector3(0,0,0.0);
    param->robot->rootLink()->v().setZero();
    param->robot->rootLink()->R() = cnoid::Matrix3::Identity();
    param->robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// rleg
        0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// lleg
        0.0, 0.0, 0.0, // torso
        0.0, 0.0, // head
        0.0, 0.959931, -0.349066, -0.261799, -1.74533, -0.436332, 0.0, -0.785398,// rarm
        0.0, 0.959931, 0.349066, 0.261799, -1.74533, 0.436332, 0.0, -0.785398,// larm
        };

    for(int j=0; j < param->robot->numJoints(); ++j){
      param->robot->joint(j)->q() = reset_manip_pose[j];
    }
    param->robot->calcForwardKinematics();
    param->robot->calcCenterOfMass();

    // limbs
    {
      {
        std::shared_ptr<rbrrt::Limb> rarm = std::make_shared<rbrrt::Limb>();
        rarm->eeParentLink = param->robot->link("RARM_JOINT7");
        rarm->eeLocal.translation() = cnoid::Vector3(-0.03,0.0,-0.15);
        rarm->eeLocal.linear() = cnoid::rotFromRpy(0.0,M_PI/2,0.0);
        rarm->joints.push_back(param->robot->link("RARM_JOINT0"));
        rarm->joints.push_back(param->robot->link("RARM_JOINT1"));
        rarm->joints.push_back(param->robot->link("RARM_JOINT2"));
        rarm->joints.push_back(param->robot->link("RARM_JOINT3"));
        rarm->joints.push_back(param->robot->link("RARM_JOINT4"));
        rarm->joints.push_back(param->robot->link("RARM_JOINT5"));
        rarm->joints.push_back(param->robot->link("RARM_JOINT6"));
        rarm->joints.push_back(param->robot->link("RARM_JOINT7"));
        param->limbs.push_back(rarm);
      }
      {
        std::shared_ptr<rbrrt::Limb> larm = std::make_shared<rbrrt::Limb>();
        larm->eeParentLink = param->robot->link("LARM_JOINT7");
        larm->eeLocal.translation() = cnoid::Vector3(-0.03,0.0,-0.15);
        larm->eeLocal.linear() = cnoid::rotFromRpy(0.0,M_PI/2,0.0);
        larm->joints.push_back(param->robot->link("LARM_JOINT0"));
        larm->joints.push_back(param->robot->link("LARM_JOINT1"));
        larm->joints.push_back(param->robot->link("LARM_JOINT2"));
        larm->joints.push_back(param->robot->link("LARM_JOINT3"));
        larm->joints.push_back(param->robot->link("LARM_JOINT4"));
        larm->joints.push_back(param->robot->link("LARM_JOINT5"));
        larm->joints.push_back(param->robot->link("LARM_JOINT6"));
        larm->joints.push_back(param->robot->link("LARM_JOINT7"));
        param->limbs.push_back(larm);
      }
      {
        std::shared_ptr<rbrrt::Limb> rleg = std::make_shared<rbrrt::Limb>();
        rleg->eeParentLink = param->robot->link("RLEG_JOINT5");
        rleg->eeLocal.translation() = cnoid::Vector3(0.0,0.0,-0.1);
        rleg->joints.push_back(param->robot->link("RLEG_JOINT0"));
        rleg->joints.push_back(param->robot->link("RLEG_JOINT1"));
        rleg->joints.push_back(param->robot->link("RLEG_JOINT2"));
        rleg->joints.push_back(param->robot->link("RLEG_JOINT3"));
        rleg->joints.push_back(param->robot->link("RLEG_JOINT4"));
        rleg->joints.push_back(param->robot->link("RLEG_JOINT5"));
        param->limbs.push_back(rleg);
      }
      {
        std::shared_ptr<rbrrt::Limb> lleg = std::make_shared<rbrrt::Limb>();
        lleg->eeParentLink = param->robot->link("LLEG_JOINT5");
        lleg->eeLocal.translation() = cnoid::Vector3(0.0,0.0,-0.1);
        lleg->joints.push_back(param->robot->link("LLEG_JOINT0"));
        lleg->joints.push_back(param->robot->link("LLEG_JOINT1"));
        lleg->joints.push_back(param->robot->link("LLEG_JOINT2"));
        lleg->joints.push_back(param->robot->link("LLEG_JOINT3"));
        lleg->joints.push_back(param->robot->link("LLEG_JOINT4"));
        lleg->joints.push_back(param->robot->link("LLEG_JOINT5"));
        param->limbs.push_back(lleg);
      }
    }

  }
}
