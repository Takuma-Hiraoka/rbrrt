#pragma once
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <trajectory_optimizer/trajectory_optimizer.h>
#include <rbrrt/rbrrt_state.h>

namespace rbrrt {
  class RBRRTParam {
  public:
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    cnoid::BodyPtr robot;
    std::vector<std::shared_ptr<rbrrt::Limb>> limbs;
  };
}
