#include "jaxon_common.h"
#include <cnoid/MeshGenerator>

namespace rbrrt_sample {
  void walk() {
    std::shared_ptr<rbrrt::Environment> environment = std::make_shared<rbrrt::Environment>();
    cnoid::BodyPtr obstacle = new cnoid::Body();
    cnoid::MeshGenerator meshGenerator;
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(4,2.3,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.0,0,-0.05);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
          environment->bulletModel.push_back(choreonoid_bullet::convertToBulletModel(shape));
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(4,0.5,1.75)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1,0.9,0.9);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
          environment->bulletModel.push_back(choreonoid_bullet::convertToBulletModel(shape));
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(4,0.5,1.75)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1,-0.9,0.9);
          posTransform->addChild(shape);
          rootLink->addShapeNode(posTransform);
          environment->bulletModel.push_back(choreonoid_bullet::convertToBulletModel(shape));
        }
      }
      obstacle->setRootLink(rootLink);
      environment->rootLink = rootLink;
    }
    {
      // collision world
      environment->field = std::make_shared<moveit_extensions::InterpolatedPropagationDistanceField>(7,//size_x
                                                                                                         5,//size_y
                                                                                                         5,//size_z
                                                                                                         0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
                                                                                                         -0.5,//origin_x
                                                                                                         -2.5,//origin_y
                                                                                                         -2.5,//origin_z
                                                                                                         0.5, // max_distance
                                                                                                         true// propagate_negative_distances
                                                                                                         );
      EigenSTL::vector_Vector3d vertices;
      for(int i=0;i<obstacle->numLinks();i++){
        std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.01);
        for(int j=0;j<vertices_.size();j++){
          vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
        }
      }
      environment->field->addPointsToField(vertices);
    }
    std::shared_ptr<rbrrt::RBRRTParam> param = std::make_shared<rbrrt::RBRRTParam>();
    param->environment = environment;
    generateJAXON(param);
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(obstacle);
    viewer->objects(param->robot);
    viewer->objects(param->abstractRobot);
    viewer->drawObjects();

    param->gikRootParam.threads = 10;
    cnoid::Isometry3 goal = param->robot->rootLink()->T();
    goal.translation()[0] += 0.5;

    std::vector<std::vector<double> > path;

    bool solved = rbrrt::solveRBPath(goal,
                                     param,
                                     path);
    std::cerr << "path size : " << path.size() << std::endl;

    while (true) {
      // main loop
      for(int i=0;i<path.size();i++){
        global_inverse_kinematics_solver::frame2Link(path.at(i),std::vector<cnoid::LinkPtr>{param->abstractRobot->rootLink()});
        param->abstractRobot->calcForwardKinematics(false);
        param->abstractRobot->calcCenterOfMass();
        viewer->drawObjects();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
    }

  }
}
