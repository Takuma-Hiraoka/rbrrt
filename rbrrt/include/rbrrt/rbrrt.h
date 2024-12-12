#pragma once
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <trajectory_optimizer/trajectory_optimizer.h>
#include <rbrrt/rbrrt_state.h>
#include <rbrrt/util.h>

namespace rbrrt {
  class RBRRTParam {
  public:
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    cnoid::BodyPtr robot;
    cnoid::BodyPtr abstractRobot;
    std::vector<std::shared_ptr<rbrrt::Limb>> limbs;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > reachabilityConstraints;
    double s = 1.2;
  };
}
