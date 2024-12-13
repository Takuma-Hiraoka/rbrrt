#pragma once
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <trajectory_optimizer/trajectory_optimizer.h>
#include <rbrrt/rbrrt_state.h>

namespace rbrrt {
  class RBRRTParam {
  public:
    int debugLevel = 0;
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    cnoid::BodyPtr robot;
    cnoid::BodyPtr abstractRobot;
    std::shared_ptr<rbrrt::Environment> environment; // planはenvironment->field座標系
    std::vector<cnoid::LinkPtr> variables;
    std::vector<std::shared_ptr<rbrrt::Limb>> limbs;
    std::vector<std::shared_ptr<rbrrt::Contact> > currentContactPoints;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > reachabilityConstraints;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > fullBodyConstraints;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
    global_inverse_kinematics_solver::GIKParam gikRootParam;
    global_inverse_kinematics_solver::GIKParam gikParam;
    prioritized_inverse_kinematics_solver2::IKParam pikParam;
    trajectory_optimizer::TOParam toParam;
    bool OptimizeTrajectory = true; // 関節角度軌道を最適化、近いstate同士をショートカットするかどうか.
    bool useSwingGIK = true;
    double s = 1.2;
    int maxTRIES = 10; // ルートリンクを動かせなくなったときに接触の付け外しを最大何回行うか. この回数行ってもルートリンクをguidePath上で動かせなければ計画失敗.
    double contactCandidateDistance = 0.1; // 接触を追加する際に、configurationDatabaseのうちエンドエフェクタと環境との距離が0以上contactCandidataDistance以下のものを候補にする.
    double envCollisionDefaultTolerance = 0.04; // 環境干渉回避制約. 触れる直前のリンクはこれより小さい値に変える.
    double envCollisionDefaultPrecision = 0.03;
    RBRRTParam() {
      gikRootParam.range = 0.1; // state間の距離の最大値
      gikRootParam.delta = 0.05; // state間の距離の最小値、サンプリングしてIKを解いてこの距離より小さい変化しかない場合は探索木に追加しない。
      gikRootParam.timeout = 10;
      gikRootParam.maxTranslation = 3;
      gikRootParam.threads = 1;
      gikRootParam.goalBias = 0.2; // state生成後この確率で更にgoalへ近づくためにstateを作る。あまり大きいと局所最適解ばかり見つかって遅い
      gikRootParam.projectCellSize = 0.4; // 0.05よりも0.1の方が速い. 0.2より0.4のほうが速い? 2m * 2m * 2mの空間を動くとして、samplingを200個くらいまでにしたければ、cellの大きさもそれなりに大きくないとスカスカになってしまう.
      gikRootParam.pikParam.we = 3e1; // 逆運動学が振動しないこと優先. 1e0だと不安定. 1e3だと大きすぎる
      gikRootParam.pikParam.wmax = 3e0; // 1e2程度にすると関節がめり込まなくなるが、ほとんど動かない.
      gikRootParam.pikParam.maxIteration = 100; // max iterationに達するか、convergeしたら終了する. isSatisfiedでは終了しない. ゼロ空間でreference angleに可能な限り近づけるタスクがあるので. 1 iterationで0.5msくらいかかるので、stateを1つ作るための時間の上限が見積もれる. 一見、この値を小さくすると早くなりそうだが、goalSampling時に本当はgoalに到達できるのにその前に返ってしまうことで遅くなることがあるため、少ないiterationでも収束するように他のパラメータを調整したほうがいい
      gikRootParam.pikParam.minIteration = 20;
      gikRootParam.pikParam.checkFinalState = true; // ゼロ空間でreference angleに可能な限り近づけるタスクのprecitionは大きくして、常にsatisfiedになることに注意
      gikRootParam.pikParam.calcVelocity = false; // 疎な軌道生成なので、velocityはチェックしない
      gikRootParam.pikParam.convergeThre = 5e-2; // 要パラチューン. IKConsraintのmaxErrorより小さくないと、収束誤判定する. maxErrorが5e-2の場合、5e-2だと大きすぎる. 5e-3だと小さすぎて時間がかかる. ikのwe, wn, wmax, maxErrorといったパラメータと連動してパラチューンせよ.
      gikRootParam.pikParam.pathOutputLoop = 5;
      toParam.initialShortcut = true;
      toParam.shortcut = true;
      toParam.shortcutThre = gikRootParam.delta/10;
      gikParam = gikRootParam;
      gikRootParam.range = 0.2; // state
      gikParam.delta = 0.01; // この距離内のstateは、中間のconstraintチェック無しで遷移可能. stateごとの距離がこの距離以内だとそもそも同じstateとみなされてあたらしくstateを作らない. 足を浮かせるとき等はstateが大きく変化しないので、deltaも小さくしておかないとstateが増えない.
      gikParam.projectCellSize = 0.02;
      gikParam.threads = 10;
      gikParam.timeout = 2;
      gikParam.goalBias = 0.2;
      gikParam.pikParam.we = 1e1; // 逆運動学が振動しないこと優先. 1e0だと不安定. 1e3だと大きすぎる
      gikParam.pikParam.wmax = 1e0; // 1e2程度にすると関節がめり込まなくなるが、ほとんど動かない.
      gikParam.pikParam.convergeThre = 5e-3;

      toParam.shortcutThre=4e-2;
      pikParam.checkFinalState=true;
      pikParam.calcVelocity = false;
      pikParam.debugLevel = 0;
      pikParam.we = 1e2;
      pikParam.wmax = 1e1;
      pikParam.convergeThre = 5e-3;
      pikParam.maxIteration = 100;
    }
  };
  bool solveRBPath(const cnoid::Isometry3 goal, // rootLink
                   const std::shared_ptr<RBRRTParam>& param,
                   std::vector<std::vector<double> >& outputPath
                   );
  bool solveRBLP(const std::shared_ptr<RBRRTParam>& param,
                 const std::vector<std::vector<double> >& guidePath,
                 std::vector<std::pair<std::vector<double>, std::vector<std::shared_ptr<Contact> > > >& outputPath // angle, contact
                 );
}
