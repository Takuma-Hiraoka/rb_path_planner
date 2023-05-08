#include <multicontact_locomotion_planner/RBRRTStateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>

namespace multicontact_locomotion_planner{
  bool ConditionAND::isValid() const {
    for(int i=0;i<children.size();i++){
      if(!children[i]->isValid()) return false;
    }
    return true;
  }
  std::shared_ptr<Condition> ConditionAND::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<ConditionAND> ret = std::make_shared<ConditionAND>();
    for(int i=0;i<children.size();i++){
      ret->children.push_back(children[i]->clone(modelMap));
    }
    return ret;
  }
  bool ConditionOR::isValid() const {
    for(int i=0;i<children.size();i++){
      if(children[i]->isValid()) return true;
    }
    return children.size()==0;
  }
  std::shared_ptr<Condition> ConditionOR::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<ConditionOR> ret = std::make_shared<ConditionOR>();
    for(int i=0;i<children.size();i++){
      ret->children.push_back(children[i]->clone(modelMap));
    }
    return ret;
  }
  bool ConditionNOT::isValid() const {
    return !child->isValid();
  }
  std::shared_ptr<Condition> ConditionNOT::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<ConditionNOT> ret = std::make_shared<ConditionNOT>();
    ret->child = child->clone(modelMap);
    return ret;
  }
  bool ConditionConstraint::isValid() const {
    constraint->updateBounds();
    return constraint->isSatisfied();
  }
  std::shared_ptr<Condition> ConditionConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<ConditionConstraint> ret = std::make_shared<ConditionConstraint>();
    ret->constraint = constraint->clone(modelMap);
    return ret;
  }
  bool RBRRTStateValidityChecker::isValid(const ompl::base::State *state) const {
    const unsigned int m = modelQueue_->pop();
    state2Link(si_->getStateSpace(), state, variables_[m]); // spaceとstateの空間をそろえる

    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_[m].begin(); it != bodies_[m].end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    calcHorizontal(horizontals_[m]);
    for(std::set<cnoid::BodyPtr>::const_iterator it=bodiesHorizontal_[m].begin(); it != bodiesHorizontal_[m].end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    bool satisfied = condition_[m]->isValid();

    if(viewer_ != nullptr && m==0){
      loopCount_++;
      if(loopCount_%drawLoop_==0){
        viewer_->drawObjects(true);
      }
    }
    modelQueue_->push(m);
    return satisfied;
  }
};
