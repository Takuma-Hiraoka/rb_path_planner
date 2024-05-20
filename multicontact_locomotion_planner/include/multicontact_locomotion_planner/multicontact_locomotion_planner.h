#ifndef MULTICONTACT_LOCOMOTION_PLANNER_H
#define MULTICONTACT_LOCOMOTION_PLANNER_H

#include <ik_constraint2/ik_constraint2.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <multicontact_locomotion_planner/RobotState.h>
#include <multicontact_locomotion_planner/CnoidStateSpace.h>
#include <multicontact_locomotion_planner/RBRRTStateValidityChecker.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>

namespace multicontact_locomotion_planner{

  class RobotIKInfo {
  public:
    cnoid::BodyPtr robot;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > selfConstraints; // 自己干渉, 関節上下限
    std::vector<std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> > envConstraints; // 環境干渉
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominalConstraints; // reset-manip-pose. 常にisSatisfied = trueであること
    prioritized_inverse_kinematics_solver2::IKParam pikParam; // 接触遷移を伴わないcontact breakability check, 及び, subGoal到達に用いる
    std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
    global_inverse_kinematics_solver::GIKParam gikParam;
    RobotIKInfo() {
      pikParam.we = 3e2; // 1e2だと解けないときに振動する
      pikParam.maxIteration = 100;

      gikParam.range = 0.5; // 0.2よりも0.3の方が速い. sample一回につきprojectGoalを行うので、rangeはなるべく大きい方がいい.
      gikParam.delta = 0.4; // 大きければ大きいほど速いはずだが、干渉計算や補間の正確さが犠牲になる. 0.2だと正確. 0.4だと速い
      gikParam.goalBias = 0.2; // 0.05よりも0.2や0.3の方が速い. goalSampingはIKの変位が大きいので、この値が大きいとsample1回あたりの時間が長くなるデメリットもある.
      gikParam.timeout = 10.0;
      gikParam.projectCellSize = 0.2; // 0.05よりも0.1の方が速い. 0.3よりも0.2の方が速い? 2m * 2m * 2mの空間を動くとして、samplingを200個くらいまでにしたければ、cellの大きさもそれなりに大きくないとスカスカになってしまう.
      //gikParam.viewer = viewer;
      gikParam.drawLoop = 1;
      gikParam.threads = 1;
      gikParam.pikParam.convergeThre = 5e-2; // 2.5e-2は小さすぎる. gikParam.pikParam.debugLevel = 1にして観察せよ. goalのprecision()の値をこれにあわせて大きくせよ
      gikParam.pikParam.pathOutputLoop = 5;
      gikParam.pikParam.we = 1e2; // 逆運動学が振動しないこと優先. 1e0だと不安定. 1e3だと大きすぎる
      gikParam.pikParam.maxIteration = 100; // max iterationに達するか、convergeしたら終了する. isSatisfiedでは終了しない. ゼロ空間でreference angleに可能な限り近づけるタスクがあるので. 1 iterationで0.5msくらいかかるので、stateを1つ作るための時間の上限が見積もれる. 一見、この値を小さくすると早くなりそうだが、goalSampling時に本当はgoalに到達できるのにその前に返ってしまうことで遅くなることがあるため、少ないiterationでも収束するように他のパラメータを調整したほうがいい
      gikParam.pikParam.minIteration = 100;
      gikParam.pikParam.checkFinalState = true; // ゼロ空間でreference angleに可能な限り近づけるタスクのprecitionは大きくして、常にsatisfiedになることに注意
      gikParam.pikParam.calcVelocity = false; // 疎な軌道生成なので、velocityはチェックしない
      gikParam.pikParam.convergeThre = 5e-2; // 要パラチューン. IKConsraintのmaxErrorより小さくないと、収束誤判定する. maxErrorが5e-2の場合、5e-2だと大きすぎる. 5e-3だと小さすぎて時間がかかる. ikのwe, wn, wmax, maxErrorといったパラメータと連動してパラチューンせよ.
      gikParam.pikParam.pathOutputLoop = 5;

    }
  public:
    bool solveFullbodyIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                         const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                         const std::unordered_map<std::string, std::shared_ptr<Contact> >& nearContacts,
                         const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& targetConstraints,
                         const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& bestEffortConstraints,
                         std::shared_ptr<std::vector<std::vector<double> > >& path
                         );
    bool solveGlobalIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                       const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                       const std::unordered_map<std::string, std::shared_ptr<Contact> >& nearContacts,
                       const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& targetConstraints,
                       const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& bestEffortConstraints,
                       const cnoid::LinkPtr& projectLink,
                       const cnoid::Isometry3& projectLocalPose,
                       std::shared_ptr<std::vector<std::vector<double> > >& path
                       );

    void updateCOMConstraint(const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts, std::shared_ptr<ik_constraint2::COMConstraint>& constraint);

  protected:
    std::shared_ptr<ik_constraint2::COMConstraint> comConstraint = std::make_shared<ik_constraint2::COMConstraint>();
  };

  class MLPParam {
  public:
    int debugLevel = 0; // 0: no debug message. 1: time measure. 2: internal state
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    unsigned int drawLoop = 100; // スレッド0が、drawLoopに一回描画する

    unsigned int threads = 1;

    cnoid::BodyPtr robot;
    cnoid::BodyPtr abstractRobot;
    cnoid::BodyPtr horizontalRobot;
    std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > assocs;
    std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > horizontals;
    std::unordered_map<std::string, std::shared_ptr<EndEffector> > endEffectors;
    std::unordered_map<std::string, std::shared_ptr<Mode> > modes;
    std::shared_ptr<RobotIKInfo> robotIKInfo = std::make_shared<RobotIKInfo>();

    global_inverse_kinematics_solver::GIKParam rbrrtParam;

    double subGoalDistanceFar = 0.6;
    double subGoalDistanceNear= 0.15;
    double subGoalRotScale = 0.1; // 回転の重みを小さくしないと、subGoalDistanceNear内の点が見つからなかったり、translation的に不自然なsubGoalになったりする

    MLPParam(){
      rbrrtParam.range = 2.0; // rootlinkのtranslationのuniform samplingの幅が大きすぎて、rangeで縮小すると、rootLinkのrotationの変位が小さくなってしまうことから、rangeは大きい方がいい
      rbrrtParam.delta = 0.2; // 大きければ大きいほど速いが、干渉計算の正確さが犠牲になる
      rbrrtParam.timeout = 30.0;
      rbrrtParam.maxTranslation = 3.0;
      //rbrrtParam.pikParam.maxIteration = 100; // env collisionは振動しやすいので15くらいがいい?. 確実に制約を満たすために大きい方がいい?
      rbrrtParam.threads = 10;
    };
  };

  bool solveMLP(const cnoid::BodyPtr currentRobot,
                const std::shared_ptr<Environment>& environment,
                const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                const std::string& prevSwingEEF,
                const std::vector<std::pair<std::vector<double>, std::string> >& targetRootPath, // angle, mode
                const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                std::vector<RobotState>& outputPath, // variablesの次元に対応
                std::string& swingEEF,

                const MLPParam& param
                );

  bool solveRBRRT(const std::shared_ptr<Environment>& environment,
                  const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                  const cnoid::Isometry3 goal,
                  const MLPParam& param,
                  std::vector<std::pair<std::vector<double>, std::string> >& outputRootPath // angle, mode
                  );

};

#endif
