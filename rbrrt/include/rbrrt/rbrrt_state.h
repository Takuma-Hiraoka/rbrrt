#pragma once
#include <cnoid/Body>

namespace rbrrt {
  class Limb {
  public:
    cnoid::Isometry3 eeLocal = cnoid::Isometry3::Identity();
    cnoid::LinkPtr eeParentLink = nullptr;
    std::vector<cnoid::LinkPtr> joints;
  };
}
