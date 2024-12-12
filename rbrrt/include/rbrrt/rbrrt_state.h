#pragma once
#include <cnoid/Body>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>

namespace rbrrt {
  class Environment {
  public:
    std::shared_ptr<moveit_extensions::InterpolatedPropagationDistanceField> field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(5,//size_x
                                                                                                                                                               5,//size_y
                                                                                                                                                               5,//size_z
                                                                                                                                                               0.04,//resolution
                                                                                                                                                               -2.5,//origin_x
                                                                                                                                                               -2.5,//origin_y
                                                                                                                                                               -2.5,//origin_z
                                                                                                                                                               0.5, // max_distance
                                                                                                                                                               false// propagate_negative_distances
                                                                                                                                                               );
    cnoid::LinkPtr rootLink = nullptr;
    std::vector<std::shared_ptr<btConvexShape> > bulletModel;
  };
  class ConfigurationData {
  public:
    double h_w;
    cnoid::Vector3 eePos;
    std::vector<double> angles;
  };
  class Limb {
  public:
    std::string name;
    cnoid::Isometry3 eeLocal = cnoid::Isometry3::Identity();
    cnoid::LinkPtr eeParentLink = nullptr;
    std::vector<cnoid::LinkPtr> joints;
    std::vector<ConfigurationData> configurationDatabase; // rootLink座標系
  };
}
