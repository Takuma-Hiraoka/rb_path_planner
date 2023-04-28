#ifndef RB_RRT_SOLVER_H
#define RB_RRT_SOLVER_H

#include <ik_constraint2/ik_constraint2.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <rb_rrt_solver/CnoidStateSpace.h>

namespace rb_rrt_solver{

  class RBRRTParam {
  public:
    int debugLevel = 0; // 0: no debug message. 1: time measure. 2: internal state

    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    unsigned int drawLoop = 100; // スレッド0が、drawLoopに一回描画する

  };


  bool solveRBRRT(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                  const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, // 0: constriant priority 1: constraints
                  const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& goals, // 0: goals(AND).
                  std::shared_ptr<std::vector<std::vector<double> > > path = nullptr, // 0: states. 1: angles
                  const RBRRTParam& param = RBRRTParam()
                  );


  bool solveRBRRT(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                  const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, // 0: modelQueue, 1: constriant priority 2: constraints
                  const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals, // 0: modelQueue. 1: goalSpace(AND).
                  std::shared_ptr<UintQueue> modelQueue,
                  std::shared_ptr<std::vector<std::vector<double> > > path, // 0: states. 1: angles
                  const RBRRTParam& param
                  );

};

#endif
