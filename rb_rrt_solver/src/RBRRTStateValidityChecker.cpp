#include <rb_rrt_solver/RBRRTStateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>

namespace rb_rrt_solver{

  bool RBRRTStateValidityChecker::isValid(const ompl::base::State *state) const {
    const unsigned int m = modelQueue_->pop();
    state2Link(si_->getStateSpace(), state, variables_[m]); // spaceとstateの空間をそろえる

    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_[m].begin(); it != bodies_[m].end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    bool satisfied = true;
    for(size_t i=0;i<constraints_[m].size();i++){
      constraints_[m][i]->updateBounds();
      if(!constraints_[m][i]->isSatisfied()) {
        if(viewer_ == nullptr || m!=0) {
          modelQueue_->push(m);
          return false;
        }
        satisfied = false;
      }
    }
    for(size_t i=0;i<inverseConstraints_[m].size();i++){
      inverseConstraints_[m][i]->updateBounds();
      if(inverseConstraints_[m][i]->isSatisfied()) {
        if(viewer_ == nullptr || m!=0) {
          modelQueue_->push(m);
          return false;
        }
        satisfied = false;
      }
    }

    if(viewer_ != nullptr && m==0){
      loopCount_++;
      if(loopCount_%drawLoop_==0){
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints_[m].size();j++){
          const std::vector<cnoid::SgNodePtr>& marker = constraints_[m][j]->getDrawOnObjects();
          std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
        }
        for(int j=0;j<inverseConstraints_[m].size();j++){
          const std::vector<cnoid::SgNodePtr>& marker = inverseConstraints_[m][j]->getDrawOnObjects();
          std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
        }
        viewer_->drawOn(markers);
        viewer_->drawObjects(true);
      }
    }
    modelQueue_->push(m);
    return satisfied;
  }
};
