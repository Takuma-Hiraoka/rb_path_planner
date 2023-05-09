#ifndef MULTICONTACT_LOCOMOTION_PLANNER_H
#define MULTICONTACT_LOCOMOTION_PLANNER_H

#include <ik_constraint2/ik_constraint2.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <multicontact_locomotion_planner/RobotState.h>
#include <multicontact_locomotion_planner/CnoidStateSpace.h>
#include <multicontact_locomotion_planner/RBRRTStateValidityChecker.h>

namespace multicontact_locomotion_planner{

  class MLPParam {
  public:
    int debugLevel = 0; // 0: no debug message. 1: time measure. 2: internal state
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    unsigned int drawLoop = 100; // スレッド0が、drawLoopに一回描画する

    unsigned int threads = 1;

    cnoid::BodyPtr robot;
    cnoid::BodyPtr abstractRobot;
    cnoid::BodyPtr horizontalRobot;
    std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > horizontals;
    std::unordered_map<std::string, std::shared_ptr<EndEffector> > endEffectors;
    std::unordered_map<std::string, std::shared_ptr<Mode> > modes;


    double subGoalDistance = 0.5;
  };

  bool solveMLP(const cnoid::BodyPtr currentRobot,
                const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                const std::string& prevSwingEEF,
                const std::vector<std::pair<std::vector<double>, std::string> >& targetRootPath, // angle, mode
                const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                std::vector<RobotState>& outputPath, // variablesの次元に対応

                const MLPParam& param
                );

  bool solveRootIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                   const std::vector<double>& subGoal,
                   std::shared_ptr<std::vector<std::vector<double> > >& path
                   );

};

#endif
