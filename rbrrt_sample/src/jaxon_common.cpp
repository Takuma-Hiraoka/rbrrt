#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/MeshGenerator>
#include <iostream>
#include <ros/package.h>
#include "jaxon_common.h"

namespace rbrrt_sample {
  void generateJAXON(const std::shared_ptr<rbrrt::Environment>& environment,
                     const std::shared_ptr<rbrrt::RBRRTParam>& param
                     ) {
    cnoid::BodyLoader bodyLoader;
    param->robot = bodyLoader.load(ros::package::getPath("jvrc_models") + "/JAXON_JVRC/JAXON_JVRCmain.wrl");
    if(!(param->robot)) std::cerr << "!robot" << std::endl;
    param->robot->rootLink()->p() = cnoid::Vector3(0,0,0.88);
    param->robot->rootLink()->v().setZero();
    param->robot->rootLink()->R() = cnoid::Matrix3::Identity();
    param->robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0.0, 0.0, -0.549066, 1.298132, -0.749066, 0.0,// rleg
        0.0, 0.0, -0.549066, 1.298132, -0.749066, 0.0,// lleg
        0.0, 0.0, 0.0, // torso
        0.0, 0.0, // head
        0.0, 0.959931, -0.349066, -0.261799, -1.74533, -0.436332, 0.0, -0.785398,// rarm
        0.0, 0.959931, 0.349066, 0.261799, -1.74533, 0.436332, 0.0, -0.785398,// larm
        -1.3, 1.3, // lfinger
        -1.3, 1.3, // rfinger
        };

    for(int j=0; j < param->robot->numJoints(); ++j){
      param->robot->joint(j)->q() = reset_manip_pose[j];
    }
    param->robot->calcForwardKinematics();
    param->robot->calcCenterOfMass();

    // variables
    {
      param->variables.push_back(param->robot->rootLink());
      for(int i=0;i<param->robot->numJoints();i++){
        if ((param->robot->joint(i)->name() == "RARM_F_JOINT0") ||
            (param->robot->joint(i)->name() == "RARM_F_JOINT1") ||
            (param->robot->joint(i)->name() == "LARM_F_JOINT0") ||
            (param->robot->joint(i)->name() == "LARM_F_JOINT1")) continue;
        param->variables.push_back(param->robot->joint(i));
      }
    }

    param->currentContactPoints.clear();
    {
      {
        {
          std::shared_ptr<rbrrt::Contact> lleg = std::make_shared<rbrrt::Contact>();
          lleg->name = "LLEG_JOINT5";
          lleg->link1 = param->robot->link("LLEG_JOINT5");
          lleg->localPose1.translation() = cnoid::Vector3(0.0,0.0,-0.1);
          lleg->localPose2 = lleg->link1->T() * lleg->localPose1;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
          C.insert(0,2) = 1.0;
          C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
          C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
          C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
          C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
          C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
          C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
          C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
          C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
          C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
          C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
          lleg->C = C;
          cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
          lleg->dl = dl;
          cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
          du[0] = 2000.0;
          lleg->du = du;
          param->currentContactPoints.push_back(lleg);
        }
        {
          std::shared_ptr<rbrrt::Contact> rleg = std::make_shared<rbrrt::Contact>();
          rleg->name = "RLEG_JOINT5";
          rleg->link1 = param->robot->link("RLEG_JOINT5");
          rleg->localPose1.translation() = cnoid::Vector3(0.0,0.0,-0.1);
          rleg->localPose2 = rleg->link1->T() * rleg->localPose1;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
          C.insert(0,2) = 1.0;
          C.insert(1,0) = 1.0; C.insert(1,2) = 0.2;
          C.insert(2,0) = -1.0; C.insert(2,2) = 0.2;
          C.insert(3,1) = 1.0; C.insert(3,2) = 0.2;
          C.insert(4,1) = -1.0; C.insert(4,2) = 0.2;
          C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
          C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
          C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
          C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
          C.insert(9,2) = 0.005; C.insert(9,5) = 1.0;
          C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
          rleg->C = C;
          cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
          rleg->dl = dl;
          cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
          du[0] = 2000.0;
          rleg->du = du;
          param->currentContactPoints.push_back(rleg);
        }
      }
    }

    // abstractRobot
    {
      param->abstractRobot = new cnoid::Body();
      cnoid::LinkPtr rootLink = new cnoid::Link();
      cnoid::MeshGenerator meshGenerator;
      double solvability_threshold = 0.5;
      double limb_transparency = 0.8;
      rootLink->setJointType(cnoid::Link::JointType::FreeJoint);
      {
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::SgMeshPtr mesh = meshGenerator.generateCylinder(0.25/*radius*/, 1.1/*height*/);
        Eigen::Matrix<double,3,Eigen::Dynamic> vertices;
        // 拡大
        {
          for (int v=0; v<mesh->vertices()->size(); v++) {
            mesh->vertices()->at(v) *= param->s;
          }
        }
        shape->setMesh(mesh);
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(-0.1,0,0.3);
        posTransform->rotation() = cnoid::AngleAxis(M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        rootLink->addShapeNode(posTransform);
      }
      {
        cnoid::LinkPtr rarmLink = new cnoid::Link();
        std::shared_ptr<reachability_map_visualizer::ReachabilityMap> rarmmap = std::make_shared<reachability_map_visualizer::ReachabilityMap>();
        reachability_map_visualizer::readMap(ros::package::getPath("reachability_map_visualizer_sample") + "/config/jaxon_rhand.yaml", rarmmap);
        rarmLink->setJointType(cnoid::Link::JointType::FixedJoint);
        rarmLink->setOffsetTranslation(rarmmap->origin);
        rarmLink->setName("RARM");
	rootLink->appendChild(rarmLink);
        cnoid::SgShapePtr shape = rbrrt::generateMeshFromReachabilityMap(rarmmap, solvability_threshold);
        if(!shape) std::cerr << "cannnot create shape from reachability map" << std::endl;
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(limb_transparency);
        material->setDiffuseColor(cnoid::Vector3f(0.0,1.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->addChild(shape);
        rarmLink->addShapeNode(posTransform);
      }
      {
        cnoid::LinkPtr larmLink = new cnoid::Link();
        std::shared_ptr<reachability_map_visualizer::ReachabilityMap> larmmap = std::make_shared<reachability_map_visualizer::ReachabilityMap>();
        reachability_map_visualizer::readMap(ros::package::getPath("reachability_map_visualizer_sample") + "/config/jaxon_lhand.yaml", larmmap);
        larmLink->setJointType(cnoid::Link::JointType::FixedJoint);
        larmLink->setOffsetTranslation(larmmap->origin);
        larmLink->setName("LARM");
	rootLink->appendChild(larmLink);
        cnoid::SgShapePtr shape = rbrrt::generateMeshFromReachabilityMap(larmmap, solvability_threshold);
        if(!shape) std::cerr << "cannnot create shape from reachability map" << std::endl;
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(limb_transparency);
        material->setDiffuseColor(cnoid::Vector3f(0.0,1.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->addChild(shape);
        larmLink->addShapeNode(posTransform);
      }
      {
        cnoid::LinkPtr rlegLink = new cnoid::Link();
        std::shared_ptr<reachability_map_visualizer::ReachabilityMap> rlegmap = std::make_shared<reachability_map_visualizer::ReachabilityMap>();
        reachability_map_visualizer::readMap(ros::package::getPath("reachability_map_visualizer_sample") + "/config/jaxon_rfoot.yaml", rlegmap);
        rlegLink->setJointType(cnoid::Link::JointType::FixedJoint);
        rlegLink->setOffsetTranslation(rlegmap->origin);
        rlegLink->setName("RLEG");
	rootLink->appendChild(rlegLink);
        cnoid::SgShapePtr shape = rbrrt::generateMeshFromReachabilityMap(rlegmap, solvability_threshold);
        if(!shape) std::cerr << "cannnot create shape from reachability map" << std::endl;
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(limb_transparency);
        material->setDiffuseColor(cnoid::Vector3f(0.0,1.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->addChild(shape);
        rlegLink->addShapeNode(posTransform);
      }
      {
        cnoid::LinkPtr llegLink = new cnoid::Link();
        std::shared_ptr<reachability_map_visualizer::ReachabilityMap> llegmap = std::make_shared<reachability_map_visualizer::ReachabilityMap>();
        reachability_map_visualizer::readMap(ros::package::getPath("reachability_map_visualizer_sample") + "/config/jaxon_lfoot.yaml", llegmap);
        llegLink->setJointType(cnoid::Link::JointType::FixedJoint);
        llegLink->setOffsetTranslation(llegmap->origin);
        llegLink->setName("LLEG");
	rootLink->appendChild(llegLink);
        cnoid::SgShapePtr shape = rbrrt::generateMeshFromReachabilityMap(llegmap, solvability_threshold);
        if(!shape) std::cerr << "cannnot create shape from reachability map" << std::endl;
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(limb_transparency);
        material->setDiffuseColor(cnoid::Vector3f(0.0,1.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->addChild(shape);
        llegLink->addShapeNode(posTransform);
      }
      param->abstractRobot->setRootLink(rootLink);
      param->abstractRobot->rootLink()->T() = param->robot->rootLink()->T();
      param->abstractRobot->calcForwardKinematics();
      param->abstractRobot->calcCenterOfMass();
    }

    // reachability
    {
      for (int i=0;i<param->abstractRobot->numLinks();i++) {
        if (param->abstractRobot->link(i) == param->abstractRobot->rootLink()) continue;
        std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
        constraint->A_link() = param->abstractRobot->link(i);
        constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
        constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
        choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                   constraint->A_FACE_C()[0],
                                                   constraint->A_FACE_dl()[0],
                                                   constraint->A_FACE_du()[0]);
        constraint->B_link() = environment->rootLink;
        constraint->B_link_bulletModel() = constraint->B_link();
        constraint->B_bulletModel() = environment->bulletModel;
        constraint->useSingleMeshB() = false; // 個別にチェック
        choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                    constraint->B_FACE_C(),
                                                    constraint->B_FACE_dl(),
                                                    constraint->B_FACE_du());
        constraint->debugLevel() = 0;
        constraint->updateBounds(); // キャッシュを内部に作る.
        param->reachabilityConstraints.push_back(constraint);
      }
      // rootlink
      {
        std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
        constraint->A_link() = param->abstractRobot->rootLink();
        constraint->field() = environment->field;
        constraint->tolerance() = 0.06; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ
        constraint->precision() = 0.05; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする
        constraint->ignoreDistance() = 0.5; // rbrttは大きく動くので、ignoreも大きくする必要がある
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        param->reachabilityConstraints.push_back(constraint);
      }
    }

    // limbs
    {
      {
        std::shared_ptr<rbrrt::Limb> rarm = std::make_shared<rbrrt::Limb>();
        rarm->name = "RARM";
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
        rbrrt::readConfigurationDatabase(ros::package::getPath("rbrrt_sample") + "/config/" + rarm->name + "_configuration_database.yaml", rarm->configurationDatabase);
        param->limbs.push_back(rarm);
      }
      {
        std::shared_ptr<rbrrt::Limb> larm = std::make_shared<rbrrt::Limb>();
        larm->name = "LARM";
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
        rbrrt::readConfigurationDatabase(ros::package::getPath("rbrrt_sample") + "/config/" + larm->name + "_configuration_database.yaml", larm->configurationDatabase);
        param->limbs.push_back(larm);
      }
      {
        std::shared_ptr<rbrrt::Limb> rleg = std::make_shared<rbrrt::Limb>();
        rleg->name = "RLEG";
        rleg->eeParentLink = param->robot->link("RLEG_JOINT5");
        rleg->eeLocal.translation() = cnoid::Vector3(0.0,0.0,-0.1);
        rleg->joints.push_back(param->robot->link("RLEG_JOINT0"));
        rleg->joints.push_back(param->robot->link("RLEG_JOINT1"));
        rleg->joints.push_back(param->robot->link("RLEG_JOINT2"));
        rleg->joints.push_back(param->robot->link("RLEG_JOINT3"));
        rleg->joints.push_back(param->robot->link("RLEG_JOINT4"));
        rleg->joints.push_back(param->robot->link("RLEG_JOINT5"));
        rbrrt::readConfigurationDatabase(ros::package::getPath("rbrrt_sample") + "/config/" + rleg->name + "_configuration_database.yaml", rleg->configurationDatabase);
        param->limbs.push_back(rleg);
      }
      {
        std::shared_ptr<rbrrt::Limb> lleg = std::make_shared<rbrrt::Limb>();
        lleg->name = "LLEG";
        lleg->eeParentLink = param->robot->link("LLEG_JOINT5");
        lleg->eeLocal.translation() = cnoid::Vector3(0.0,0.0,-0.1);
        lleg->joints.push_back(param->robot->link("LLEG_JOINT0"));
        lleg->joints.push_back(param->robot->link("LLEG_JOINT1"));
        lleg->joints.push_back(param->robot->link("LLEG_JOINT2"));
        lleg->joints.push_back(param->robot->link("LLEG_JOINT3"));
        lleg->joints.push_back(param->robot->link("LLEG_JOINT4"));
        lleg->joints.push_back(param->robot->link("LLEG_JOINT5"));
        rbrrt::readConfigurationDatabase(ros::package::getPath("rbrrt_sample") + "/config/" + lleg->name + "_configuration_database.yaml", lleg->configurationDatabase);
        param->limbs.push_back(lleg);
      }
    }

  }
}
