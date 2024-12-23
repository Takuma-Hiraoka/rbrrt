#include <rbrrt/rbrrt.h>
#include <rbrrt/util.h>

namespace rbrrt {
  bool solveRBPath(const cnoid::Isometry3 goal, // rootLink
                   const std::shared_ptr<RBRRTParam>& param,
                   std::vector<std::vector<double> >& outputPath
                   ) {
    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(param->abstractRobot->rootLink());

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
    {
      // pitch > 0
      std::shared_ptr<ik_constraint2::RegionConstraint> constraint = std::make_shared<ik_constraint2::RegionConstraint>();
      constraint->A_link() = param->abstractRobot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.1,0.0,0.0);
      constraint->B_link() = param->abstractRobot->rootLink();
      constraint->eval_link() = nullptr;
      constraint->weightR().setZero();
      constraint->C().resize(1,3);
      constraint->C().insert(0,2) = 1.0;
      constraint->dl().resize(1);
      constraint->dl()[0] = -1e10;
      constraint->du().resize(1);
      constraint->du()[0] = 0.0;
      constraints0.push_back(constraint);
    }
    {
      // pitch < 90
      std::shared_ptr<ik_constraint2::RegionConstraint> constraint = std::make_shared<ik_constraint2::RegionConstraint>();
      constraint->A_link() = param->abstractRobot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
      constraint->B_link() = param->abstractRobot->rootLink();
      constraint->eval_link() = nullptr;
      constraint->weightR().setZero();
      constraint->C().resize(1,3);
      constraint->C().insert(0,2) = 1.0;
      constraint->dl().resize(1);
      constraint->dl()[0] = -1e10;
      constraint->du().resize(1);
      constraint->du()[0] = 0.0;
      constraints0.push_back(constraint);
    }
    {
      // roll = 0
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = param->abstractRobot->rootLink();
      constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.1,0.0);
      constraint->B_link() = param->abstractRobot->rootLink();
      constraint->B_localpos().translation() = cnoid::Vector3(0.0,-0.1,0.0);
      constraint->eval_link() = nullptr;
      constraint->weight() << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
      constraints0.push_back(constraint);
    }
    constraints.push_back(constraints0);
    constraints.push_back(param->reachabilityConstraints);

    std::vector<double> goals;
    {
      cnoid::Isometry3 org = param->abstractRobot->rootLink()->T();
      param->abstractRobot->rootLink()->T() = goal;
      global_inverse_kinematics_solver::link2Frame(variables, goals);
      param->abstractRobot->rootLink()->T() = org;
    }

    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
    if(!global_inverse_kinematics_solver::solveGIK(variables,
                                                   constraints,
                                                   goals,
                                                   param->gikRootParam,
                                                   path
                                                   )){
      std::cerr << "solveRBPath failed" << std::endl;
      return false;
    }

    // 余分な軌道を修正, カット
    if (param->OptimizeTrajectory) {
      param->toParam.pikParam.convergeThre=param->gikRootParam.pikParam.convergeThre * path->size();
      trajectory_optimizer::solveTO(variables,
                                    constraints,
                                    param->toParam,
                                    path);
    }
    outputPath.resize(path->size());
    for (int i=0; i<path->size(); i++) {
      outputPath[i] = path->at(i);
    }
    return true;
  }

  bool solveRBLP(const std::shared_ptr<RBRRTParam>& param,
                 const std::vector<std::vector<double> >& guidePath,
                 std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath // angle, contact
                 ) {
    // 1~4をguidePathが終わるまで繰り返す.
    // 1. 現在の接触状態のままルートリンクをguidePathに従ってIKが解けなくなるまで動かす.
    // 2. guidePathに従うIKが解けなければ,離れている接触を全てつける.
    // 3. 2でもguidePathに従うIKが解けなければ,ついている接触を一つ離す.
    // 4. 2~3をguidePathに従うIKが解けるまで繰り返す. 一定回数に達したら失敗
    if (param->debugLevel >= 2) {
      std::cerr << "[solveRBLP] start. guide path size : " << guidePath.size() << std::endl;
    }
    // bbxを計算
    for (int i=0; i<param->currentContactPoints.size(); i++) {
      param->currentContactPoints[i]->calcBoundingBox();
    }
    outputPath.clear();
    std::vector<std::shared_ptr<Contact> > currentContact = param->currentContactPoints;
    for (int guidePathId=0;guidePathId<guidePath.size();guidePathId++) {
      // goal
      cnoid::Isometry3 org = param->robot->rootLink()->T();
      global_inverse_kinematics_solver::frame2Link(guidePath[guidePathId],std::vector<cnoid::LinkPtr>{param->robot->rootLink()});
      cnoid::Isometry3 goal = param->robot->rootLink()->T();
      param->robot->rootLink()->T() = org;

      std::shared_ptr<ik_constraint2::PositionConstraint> rootConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
      rootConstraint->A_link() = param->robot->rootLink();
      rootConstraint->B_link() = nullptr;
      rootConstraint->B_localpos() = goal;
      rootConstraint->eval_link() = nullptr;
      rootConstraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
      bool rootSolved = solveContactIK(param, param->variables, currentContact, nullptr, rootConstraint, IKState::ROOT);
      if(rootSolved) {
        std::vector<double> frame;
        global_inverse_kinematics_solver::link2Frame(param->variables, frame);
        outputPath.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > >(frame, currentContact));
        if(param->debugLevel >= 3){
          if(param->viewer){
            param->viewer->drawObjects();
          }
          std::cerr << "[solveRBLP] proceed guidePath. current index : " << guidePathId << " Press ENTER:" << std::endl;
          getchar();
        }
        continue;
      }
      else {
          std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > > path;
        for (int i=0;i<param->maxTRIES;i++) {
          bool change = false;

          for (int l=0;l<param->limbs.size();l++) {
            if (!(param->limbs[l]->isContact)) {
              std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > > addPath;
              bool attachSolved = searchLimbContact(param, param->limbs[l], currentContact, rootConstraint, addPath);
              if (attachSolved) {
                change = true;
                param->limbs[l]->isContact = true;
                currentContact = (addPath.back()).second;
                path.insert(path.end(), addPath.begin(), addPath.end());

                if(param->debugLevel >= 3){
                  if(param->viewer){
                    param->viewer->drawObjects();
                  }
                  std::cerr << "[solveRBLP] attach contact. current guidePathid : " << guidePathId << " current itr : " << i  << " Press ENTER:" << std::endl;
                  getchar();
                }
              }
            }
          }

          if (change) rootSolved = solveContactIK(param, param->variables, currentContact, nullptr, rootConstraint, IKState::ROOT);

          if (rootSolved) {
            std::vector<double> frame;
            global_inverse_kinematics_solver::link2Frame(param->variables, frame);
            path.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > >(frame, currentContact));
            break;
          } else {
            if (currentContact.size() > 1) {
              std::shared_ptr<Contact> nextContact = currentContact[0];
              currentContact.erase(currentContact.begin());
              bool detachSolved = solveContactIK(param, param->variables, currentContact, nextContact, rootConstraint, IKState::DETACH);
              if (detachSolved) {
                change = true;
                std::vector<double> frame;
                global_inverse_kinematics_solver::link2Frame(param->variables, frame);
                path.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > >(frame, currentContact));
                for (int l=0;l<param->limbs.size();l++) {
                  if (param->limbs[l]->name == nextContact->name) param->limbs[l]->isContact = false;
                }
                if(param->debugLevel >= 3){
                  if(param->viewer){
                    param->viewer->drawObjects();
                  }
                  std::cerr << "[solveRBLP] detach contact. current guidePathid : " << guidePathId << " current itr : " << i  << " Press ENTER:" << std::endl;
                  getchar();
                }
              }
            }
          } // rootSolved
          if (change) rootSolved = solveContactIK(param, param->variables, currentContact, nullptr, rootConstraint, IKState::ROOT);
          if (rootSolved) {
            std::vector<double> frame;
            global_inverse_kinematics_solver::link2Frame(param->variables, frame);
            path.push_back(std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > >(frame, currentContact));
            break;
          }
          if(!change) {
            std::cerr << "[solveRBLP] failed. cannot attach nor detach. current guidePathid : " << guidePathId << std::endl;
            return false;
          }
        } // maxTRIES
        if (rootSolved) {
          outputPath.insert(outputPath.end(), path.begin(), path.end());
          if(param->debugLevel >= 3){
            if(param->viewer){
              param->viewer->drawObjects();
            }
            std::cerr << "[solveRBLP] proceed guidePath. current index : " << guidePathId << " Press ENTER:" << std::endl;
            getchar();
          }
        } else {
          std::cerr << "[solveRBLP] failed. maxTRIES." << std::endl;
          return false;
        }
      } // rootSolved
    } // guidePathId

    if (param->debugLevel >= 0) {
      std::cerr << "[solveRBLP] succeeded." << std::endl;
    }
    return true;
  }

}
