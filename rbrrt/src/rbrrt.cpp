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

  }

}
