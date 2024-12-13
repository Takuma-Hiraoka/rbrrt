#include <rbrrt/util.h>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <cnoid/YAMLWriter>
#include <cnoid/YAMLReader>
#include <cnoid/MeshExtractor>
#include <ik_constraint2_scfr/KeepCollisionScfrConstraint.h>

namespace rbrrt {
  bool searchLimbContact(const std::shared_ptr<rbrrt::RBRRTParam>& param,
                         const std::shared_ptr<rbrrt::Limb> targetLimb,
                         const std::vector<std::shared_ptr<Contact> >& stopContacts,
                         std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath /* out */) {
    outputPath.clear();
    std::shared_ptr<Contact> nextContact = std::make_shared<Contact>();
    nextContact->name = targetLimb->name;
    nextContact->link1 = targetLimb->eeParentLink;
    nextContact->localPose1 = targetLimb->eeLocal;
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
    nextContact->C = C;
    cnoid::VectorX dl = Eigen::VectorXd::Zero(11);
    nextContact->dl = dl;
    cnoid::VectorX du = 1e10 * Eigen::VectorXd::Ones(11);
    du[0] = 2000.0;
    nextContact->du = du;
    nextContact->calcBoundingBox();

    for (int i=0;i<targetLimb->configurationDatabase.size();i++) {
      // configurationDataBaseはhが高い順にソートされている
      cnoid::Vector3 eeP = param->robot->rootLink()->p() + param->robot->rootLink()->R() * targetLimb->configurationDatabase[i].eePos;
      cnoid::Vector3 grad;
      bool in_bound; // Whether or not the (x,y,z) is valid for gradient purposes.
      double dist = param->environment->field->getDistanceGradient(eeP[0],eeP[1],eeP[2],grad[0],grad[1],grad[2],in_bound);
      if (dist > 0 && // めり込んでいると干渉回避制約が難しいので0以上
          dist < param->contactCandidateDistance) {
        global_inverse_kinematics_solver::frame2Link(targetLimb->configurationDatabase[i].angles,targetLimb->joints);
        param->robot->calcForwardKinematics();
        param->robot->calcCenterOfMass();

        // nextContactの目標生成
        nextContact->localPose2.translation() = eeP + grad;
        cnoid::Vector3d z_axis = grad;
        cnoid::Vector3d x_axis = (z_axis==cnoid::Vector3d::UnitY() || z_axis==-cnoid::Vector3d::UnitY()) ? cnoid::Vector3d::UnitZ() : cnoid::Vector3d::UnitY().cross(z_axis);
        cnoid::Vector3d y_axis = z_axis.cross(x_axis);
        nextContact->localPose2.linear().col(0) = x_axis.normalized(); nextContact->localPose2.linear().col(1) = y_axis.normalized(); nextContact->localPose2.linear().col(2) = z_axis.normalized();

        if (solveContactIK(param, targetLimb->joints, stopContacts, nextContact, nullptr, IKState::SWING)) {
          std::vector<double> frame;
          global_inverse_kinematics_solver::link2Frame(param->variables, frame);
          outputPath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > >(frame, stopContacts));
          if (solveContactIK(param, targetLimb->joints, stopContacts, nextContact, nullptr, IKState::ATTACH)) {
            global_inverse_kinematics_solver::link2Frame(param->variables, frame);
            std::vector<std::shared_ptr<Contact> > nextContacts = stopContacts;
            nextContacts.push_back(nextContact);
            outputPath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > >(frame, nextContacts));
            return true;
          } else {
            // ATTACHはできなかったので他のconfigurationを試す
            outputPath.clear();
          }
        }
      }
    }
    return false;
  }
  bool solveContactIK(const std::shared_ptr<rbrrt::RBRRTParam>& param,
                      const std::vector<cnoid::LinkPtr> variables,
                      const std::vector<std::shared_ptr<Contact> >& stopContacts,
                      const std::shared_ptr<Contact>& nextContact,
                      const std::shared_ptr<ik_constraint2::PositionConstraint>& rootConstraint,
                      const IKState ikstate) {
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    for (int i=0; i<param->fullBodyConstraints.size(); i++) {
      if (typeid(*(param->fullBodyConstraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
        bool skip=false;
        // すでに触れているリンクは干渉回避制約に含めない
        for (int j=0; j<stopContacts.size() && !skip;j++) {
          if (stopContacts[j]->link1->name() == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->fullBodyConstraints[i])->A_link()->name()) skip = true;
        }
        if (!skip && nextContact && nextContact->link1->name() == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->fullBodyConstraints[i])->A_link()->name()) {
          if ((ikstate == IKState::ATTACH) ||
              (ikstate == IKState::DETACH)) { // 実際に触れされるときだけ、触れるリンクの干渉は無視する. detach-attachならdetachのときに干渉を考慮した姿勢が出ているので、そこから先は干渉しないと仮定.
            // DETACH時もすでに触れているときの挙動を回避するため
            skip = true;
          } else {
            std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->fullBodyConstraints[i])->tolerance() = 0.02;
            std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->fullBodyConstraints[i])->precision() = 0.015;
          }
        }
        if(skip) continue;
      }
      constraints0.push_back(param->fullBodyConstraints[i]);
    }

    std::shared_ptr<ik_constraint2_scfr::ScfrConstraint> scfrConstraint = std::make_shared<ik_constraint2_scfr::ScfrConstraint>();
    scfrConstraint->A_robot() = param->robot;
    std::vector<cnoid::Isometry3> poses;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
    std::vector<cnoid::VectorX> bs;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
    std::vector<cnoid::VectorX> dls;
    std::vector<cnoid::VectorX> dus;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2;
    for (int i=0; i<stopContacts.size(); i++) {
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = stopContacts[i]->link1;
      constraint->A_localpos() = stopContacts[i]->localPose1;
      constraint->B_link() = stopContacts[i]->link2;
      constraint->B_localpos() = stopContacts[i]->localPose2;
      constraint->eval_link() = nullptr;
      constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
      constraints1.push_back(constraint);
      poses.push_back(stopContacts[i]->localPose2);
      As.emplace_back(0,6);
      bs.emplace_back(0);
      Cs.push_back(stopContacts[i]->C);
      dls.push_back(stopContacts[i]->dl);
      dus.push_back(stopContacts[i]->du);
      calcIgnoreBoundingBox(param->fullBodyConstraints, stopContacts[i], 3);
    }
    // nextContact
    if (nextContact) {
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = nextContact->link1;
      constraint->A_localpos() = nextContact->localPose1;
      constraint->B_link() = nextContact->link2;
      constraint->B_localpos() = nextContact->localPose2;
      if ((ikstate==IKState::DETACH) ||
          (ikstate==IKState::SWING)) constraint->B_localpos().translation() += nextContact->localPose2.rotation() * cnoid::Vector3(0,0,0.03);
      if (ikstate==IKState::ATTACH) calcIgnoreBoundingBox(param->fullBodyConstraints, nextContact, 3);
      constraint->eval_link() = nullptr;
      constraint->eval_localR() = constraint->B_localpos().rotation();
      constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 0.0;
      constraints2.push_back(constraint);
    }
    if (rootConstraint) {
      constraints2.push_back(rootConstraint);
    }
    scfrConstraint->poses() = poses;
    scfrConstraint->As() = As;
    scfrConstraint->bs() = bs;
    scfrConstraint->Cs() = Cs;
    scfrConstraint->dls() = dls;
    scfrConstraint->dus() = dus;
    constraints0.push_back(scfrConstraint);

    bool solved = false;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0, constraints1, constraints2, param->nominals};
    std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
    solved  =  prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                   constraints,
                                                                   prevTasks,
                                                                   param->pikParam
                                                                   );

    if(!solved && param->useSwingGIK && (ikstate!=IKState::ROOT)) {
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > gikConstraints{constraints0, constraints1};
      param->gikParam.projectLink.resize(1);
      param->gikParam.projectLink[0] = nextContact->link1;
      param->gikParam.projectLocalPose = nextContact->localPose1;
      std::shared_ptr<std::vector<std::vector<double> > > path;
      // 関節角度上下限を厳密に満たしていないと、omplのstart stateがエラーになるので
      for(int i=0;i<param->variables.size();i++){
        if(param->variables[i]->isRevoluteJoint() || param->variables[i]->isPrismaticJoint()) {
          param->variables[i]->q() = std::max(std::min(param->variables[i]->q(),param->variables[i]->q_upper()),param->variables[i]->q_lower());
        }
      }
      solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                          gikConstraints,
                                                          constraints2,
                                                          param->nominals,
                                                          param->gikParam,
                                                          path);
    }

    for (int i=0; i<param->fullBodyConstraints.size(); i++) {
      if (typeid(*(param->fullBodyConstraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
        std::static_pointer_cast<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>(param->fullBodyConstraints[i])->ignoreBoundingBox().clear();
        if (nextContact && nextContact->link1->name() == std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->fullBodyConstraints[i])->A_link()->name()) {
            std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->fullBodyConstraints[i])->tolerance() = param->envCollisionDefaultTolerance;
            std::static_pointer_cast<ik_constraint2::CollisionConstraint>(param->fullBodyConstraints[i])->precision() = param->envCollisionDefaultPrecision;
        }
      }
    }

    return solved;
  }
  void calcLevelLinks(const cnoid::LinkPtr inputLink,
                      int level, // input
                      std::vector<cnoid::LinkPtr>& targetLinks // output
                      ){ // inputLinkのlevel等親のリンクをtargetLinksとして返す.
    targetLinks.clear();
    targetLinks.push_back(inputLink);
    for (int iter=0; iter<level; iter++) {
      int prevLevelSize = targetLinks.size();
      for (int i=0; i<prevLevelSize; i++) {
        if ((targetLinks[i]->parent() != nullptr) && (std::find(targetLinks.begin(), targetLinks.end(), targetLinks[i]->parent()) == targetLinks.end())) targetLinks.push_back(targetLinks[i]->parent());
        cnoid::LinkPtr child = targetLinks[i]->child();
        while (child != nullptr) {
          if (std::find(targetLinks.begin(), targetLinks.end(), child) == targetLinks.end()) targetLinks.push_back(child);
          child = child->sibling();
        }
      }
    }
  }

  void calcIgnoreBoundingBox(const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints,
                             const std::shared_ptr<Contact>& contact,
                             int level
                             ) { // constraint中のcollisionConstraintについて、contactのlink1のlevel等親のリンクの干渉回避である場合、contactのlink1のBoundingBoxを追加する.
    std::vector<cnoid::LinkPtr> targetLinks;
    calcLevelLinks(contact->link1, level, targetLinks);

    for (int i=0; i<constraints.size(); i++) {
      if (typeid(*(constraints[i]))==typeid(ik_constraint2_distance_field::DistanceFieldCollisionConstraint)) {
        if (std::find(targetLinks.begin(), targetLinks.end(), std::static_pointer_cast<ik_constraint2::CollisionConstraint>(constraints[i])->A_link()) != targetLinks.end()) {
          if (std::static_pointer_cast<ik_constraint2::CollisionConstraint>(constraints[i])->A_link() == contact->link1) continue; // この関数が呼ばれるのはsolveContactIK中で、接触リンクそのもののcollisionConstraintはそもそもconstraintに入っていない. よってcontactと一致するものはどのみちconstraintに入らないが、下と仕様をそろえるため.
          ik_constraint2_distance_field::DistanceFieldCollisionConstraint::BoundingBox ignoreBoundingBox;
          ignoreBoundingBox.parentLink = contact->link1;
          ignoreBoundingBox.localPose.translation() = contact->bbx.center();
          ignoreBoundingBox.dimensions = contact->bbx.max() - contact->bbx.min();
          std::static_pointer_cast<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>(constraints[i])->ignoreBoundingBox().push_back(ignoreBoundingBox);
        }
      }
    }

  }

  inline void addMesh(cnoid::SgMeshPtr model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
    cnoid::SgMeshPtr mesh = meshExtractor->currentMesh();
    const cnoid::Affine3& T = meshExtractor->currentTransform();

    const int vertexIndexTop = model->vertices()->size();

    const cnoid::SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
      const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
      model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
      cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
      const int v0 = vertexIndexTop + tri[0];
      const int v1 = vertexIndexTop + tri[1];
      const int v2 = vertexIndexTop + tri[2];
      model->addTriangle(v0, v1, v2);
    }
  }

  cnoid::SgMeshPtr convertToSgMesh (const cnoid::SgNodePtr collisionshape){
    if (!collisionshape) return nullptr;
    std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
    cnoid::SgMeshPtr model = new cnoid::SgMesh; model->getOrCreateVertices();
    if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
    }else{
      //      std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return nullptr;
    }
    model->setName(collisionshape->name());

    return model;
  }

  void Contact::calcBoundingBox() {
    cnoid::SgMeshPtr mesh = convertToSgMesh(this->link1->collisionShape());
    if(mesh && (mesh->numTriangles() != 0)) {
      mesh->updateBoundingBox();
      this->bbx = mesh->boundingBox();
    }
  }

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
