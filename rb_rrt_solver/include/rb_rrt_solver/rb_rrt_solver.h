#ifndef RB_RRT_SOLVER_H
#define RB_RRT_SOLVER_H

#include <ik_constraint2/ik_constraint2.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <rb_rrt_solver/CnoidStateSpace.h>
#include <rb_rrt_solver/RBRRTStateValidityChecker.h>

namespace rb_rrt_solver{

  class RBRRTParam {
  public:
    int debugLevel = 0; // 0: no debug message. 1: time measure. 2: internal state
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    unsigned int drawLoop = 100; // スレッド0が、drawLoopに一回描画する

    double maxTranslation = 3.0;
    double stateValidityCheckingResolution = 0.01; // statespaceの上下限の、この値倍の区間は、validity check不要
    double range = 0.5;
    double timeout = 30.0;

    unsigned int threads = 1;
  };


  bool solveRBRRT(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                  const std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> >& horizontals, // 0: variables
                  const std::shared_ptr<Condition>& condition,
                  const std::vector<double>& goal, // 0: angles
                  std::shared_ptr<std::vector<std::vector<double> > > path = nullptr, // 0: states. 1: angles
                  const RBRRTParam& param = RBRRTParam()
                  );


  bool solveRBRRT(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                  const std::vector<std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > >& horizontals, // 0: modelQueue, 1: horizontals
                  const std::vector<std::shared_ptr<Condition> >& condition, // 0: modelQueue
                  const std::vector<double>& goal, // 0: angles
                  std::shared_ptr<UintQueue> modelQueue,
                  std::shared_ptr<std::vector<std::vector<double> > > path, // 0: states. 1: angles
                  const RBRRTParam& param
                  );

};

#endif
