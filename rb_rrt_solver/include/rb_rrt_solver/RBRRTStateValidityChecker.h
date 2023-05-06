#include <ik_constraint2/ik_constraint2.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <rb_rrt_solver/CnoidStateSpace.h>
#include <ompl/base/StateValidityChecker.h>

namespace rb_rrt_solver{
  OMPL_CLASS_FORWARD(RBRRTStateValidityChecker); // *Ptrを定義. (shared_ptr)

  class Condition {
  public:
    virtual bool isValid() const = 0;
    virtual std::shared_ptr<Condition> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const = 0;
  };
  class ConditionAND : public Condition {
  public:
    bool isValid() const override;
    std::vector<std::shared_ptr<Condition> > children;
    std::shared_ptr<Condition> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
  };
  class ConditionOR : public Condition {
  public:
    bool isValid() const override;
    std::vector<std::shared_ptr<Condition> > children;
    std::shared_ptr<Condition> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
  };
  class ConditionNOT : public Condition {
  public:
    bool isValid() const override;
    std::shared_ptr<Condition> child;
    std::shared_ptr<Condition> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
  };
  class ConditionConstraint : public Condition {
  public:
    bool isValid() const override;
    std::shared_ptr<ik_constraint2::IKConstraint> constraint;
    std::shared_ptr<Condition> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
  };

  class RBRRTStateValidityChecker: public ompl::base::StateValidityChecker {
  public:
    RBRRTStateValidityChecker(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<UintQueue>& modelQueue, const std::vector<std::shared_ptr<Condition> >& condition, const std::vector<std::vector<cnoid::LinkPtr> >& variables, const std::vector<std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > >& horizontals) :
      StateValidityChecker(si),
      modelQueue_(modelQueue),
      variables_(variables),
      horizontals_(horizontals),
      condition_(condition)
    {
      for(int i=0;i<variables_.size();i++){
        bodies_.push_back(getBodies(variables_[i]));
        bodiesHorizontal_.push_back(getBodies(horizontals_[i]));
      }
    }

    bool isValid(const ompl::base::State *state) const override;

    std::shared_ptr<choreonoid_viewer::Viewer>& viewer() {return viewer_;}
    const std::shared_ptr<choreonoid_viewer::Viewer>& viewer() const {return viewer_;}
    unsigned int& drawLoop() { return drawLoop_; }
    const unsigned int& drawLoop() const { return drawLoop_; }
  protected:
    mutable std::shared_ptr<UintQueue> modelQueue_;
    const std::vector<std::vector<cnoid::LinkPtr> > variables_;
    const std::vector<std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > > horizontals_;
    const std::vector<std::shared_ptr<Condition> > condition_;
    std::vector<std::set<cnoid::BodyPtr> > bodies_;
    std::vector<std::set<cnoid::BodyPtr> > bodiesHorizontal_;

    std::shared_ptr<choreonoid_viewer::Viewer> viewer_ = nullptr;
    mutable int loopCount_ = 0;
    unsigned int drawLoop_ = 100;

  };
}
