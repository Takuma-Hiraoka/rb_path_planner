#ifndef RB_RRT_SOLVER_CNOIDSTATESPACE_H
#define RB_RRT_SOLVER_CNOIDSTATESPACE_H

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/WrapperStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <cnoid/Body>
#include <set>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace rb_rrt_solver{

  void state2Link(const ompl::base::StateSpacePtr& space, const ompl::base::State *state, const std::vector<cnoid::LinkPtr>& links);
  void state2Link(const ompl::base::StateSpace* space, const ompl::base::State *state, const std::vector<cnoid::LinkPtr>& links);
  void link2State(const std::vector<cnoid::LinkPtr>& links, const ompl::base::StateSpacePtr& space, ompl::base::State *state);
  void link2State(const std::vector<cnoid::LinkPtr>& links, const ompl::base::StateSpace* space, ompl::base::State *state);
  void state2Frame(const ompl::base::StateSpacePtr& space, const ompl::base::State *state, std::vector<double>& frame);
  void state2Frame(const ompl::base::StateSpace* space, const ompl::base::State *state, std::vector<double>& frame);
  void frame2State(const std::vector<double>& frame, const ompl::base::StateSpacePtr& space, ompl::base::State *state);
  void frame2State(const std::vector<double>& frame, const ompl::base::StateSpace* space, ompl::base::State *state);
  void frame2Link(const std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& links);
  void link2Frame(const std::vector<cnoid::LinkPtr>& links, std::vector<double>& frame);

  std::set<cnoid::BodyPtr> getBodies(const std::vector<cnoid::LinkPtr>& links);

  ompl::base::StateSpacePtr createAmbientSpace(const std::vector<cnoid::LinkPtr>& variables, double maxtranslation=1.0);

  class DummyProjectionEvaluator : public ompl::base::ProjectionEvaluator {
  public:
    DummyProjectionEvaluator(const ompl::base::StateSpace *space) : ProjectionEvaluator(space) {
      // boundのサイズが0だとワーニングがでるので適当に与える
      bounds_ = ompl::base::RealVectorBounds(1);
      bounds_.setLow(-1);
      bounds_.setHigh(1);
    }
    DummyProjectionEvaluator(const ompl::base::StateSpacePtr &space) : ProjectionEvaluator(space) {
      // boundのサイズが0だとワーニングがでるので適当に与える
      bounds_ = ompl::base::RealVectorBounds(1);
      bounds_.setLow(-1);
      bounds_.setHigh(1);
    }
    virtual unsigned int getDimension() const override { return 1; }
    virtual void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override {}
  };

  // 何番目のロボットモデルが空いているか. スレッドセーフにモデルを使うために必要
  class UintQueue {
  public:
    void push(const unsigned int& m);
    unsigned int pop();
  protected:
    std::queue<unsigned int> queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
  };

  ompl::base::StateSamplerPtr allocCnoidCompoundStateSampler(const ompl::base::StateSpace *space);
};

#endif
