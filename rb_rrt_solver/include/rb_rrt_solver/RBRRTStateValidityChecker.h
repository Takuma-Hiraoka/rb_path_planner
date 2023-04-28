#include <ik_constraint2/ik_constraint2.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <rb_rrt_solver/CnoidStateSpace.h>
#include <ompl/base/StateValidityChecker.h>

namespace rb_rrt_solver{
  OMPL_CLASS_FORWARD(RBRRTStateValidityChecker); // *Ptrを定義. (shared_ptr)

  class RBRRTStateValidityChecker: public ompl::base::StateValidityChecker {
  public:
    RBRRTStateValidityChecker(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<UintQueue>& modelQueue, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& inverseConstraints, const std::vector<std::vector<cnoid::LinkPtr> >& variables) :
      StateValidityChecker(si),
      modelQueue_(modelQueue),
      variables_(variables),
      constraints_(constraints),
      inverseConstraints_(inverseConstraints)
    {
      for(int i=0;i<variables_.size();i++){
        bodies_.push_back(getBodies(variables_[i]));
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
    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints_;
    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > inverseConstraints_;
    std::vector<std::set<cnoid::BodyPtr> > bodies_;

    std::shared_ptr<choreonoid_viewer::Viewer> viewer_ = nullptr;
    mutable int loopCount_ = 0;
    unsigned int drawLoop_ = 100;

  };
}
