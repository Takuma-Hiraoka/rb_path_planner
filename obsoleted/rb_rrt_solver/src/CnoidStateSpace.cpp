#include <rb_rrt_solver/CnoidStateSpace.h>

namespace rb_rrt_solver{

  void state2Link(const ompl::base::StateSpacePtr& space, const ompl::base::State *state, const std::vector<cnoid::LinkPtr>& links){
    state2Link(space.get(),state,links);
  }
  void state2Link(const ompl::base::StateSpace* space, const ompl::base::State *state, const std::vector<cnoid::LinkPtr>& links){
    unsigned int i=0;
    // StateSpace::getValueAddressAtIndex()や、ScopedState::reals()は、O(n)の計算量がかかるので、getValueAddressAtLocationを使う
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        links[l]->q() = *(space->getValueAddressAtLocation(state, space->getValueLocations()[i]));
        i+=1;
      }else if(links[l]->isFreeJoint()) {
        links[l]->p()[0] = *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+0]));
        links[l]->p()[1] = *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+1]));
        links[l]->p()[2] = *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+2]));
        links[l]->R() = cnoid::Quaternion(*(space->getValueAddressAtLocation(state, space->getValueLocations()[i+6])),
                                          *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+3])),
                                          *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+4])),
                                          *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+5]))).toRotationMatrix();
        i+=7;
      }
    }
  }
  void link2State(const std::vector<cnoid::LinkPtr>& links, const ompl::base::StateSpacePtr& space, ompl::base::State *state){
    link2State(links,space.get(),state);
  }
  void link2State(const std::vector<cnoid::LinkPtr>& links, const ompl::base::StateSpace* space, ompl::base::State *state){
    unsigned int i=0;
    // StateSpace::getValueAddressAtIndex()や、ScopedState::reals()は、O(n)の計算量がかかるので、getValueAddressAtLocationを使う
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        *(space->getValueAddressAtLocation(state, space->getValueLocations()[i])) = links[l]->q();
        i+=1;
      }else if(links[l]->isFreeJoint()) {
        *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+0])) = links[l]->p()[0];
        *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+1])) = links[l]->p()[1];
        *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+2])) = links[l]->p()[2];
        cnoid::Quaternion q(links[l]->R());
        *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+6])) = q.w();
        *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+3])) = q.x();
        *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+4])) = q.y();
        *(space->getValueAddressAtLocation(state, space->getValueLocations()[i+5])) = q.z();
        i+=7;
      }
    }
  }
  void state2Frame(const ompl::base::StateSpacePtr& space, const ompl::base::State *state, std::vector<double>& frame){
    state2Frame(space.get(),state,frame);
  }
  void state2Frame(const ompl::base::StateSpace* space, const ompl::base::State *state, std::vector<double>& frame){
    space->copyToReals(frame, state);
  }
  void frame2State(const std::vector<double>& frame, const ompl::base::StateSpacePtr& space, ompl::base::State *state){
    frame2State(frame,space.get(),state);
  }
  void frame2State(const std::vector<double>& frame, const ompl::base::StateSpace* space, ompl::base::State *state){
    space->copyFromReals(state,frame);
  }
  void frame2Link(const std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& links){
    unsigned int i=0;
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        links[l]->q() = frame[i];
        i+=1;
      }else if(links[l]->isFreeJoint()) {
        links[l]->p()[0] = frame[i+0];
        links[l]->p()[1] = frame[i+1];
        links[l]->p()[2] = frame[i+2];
        links[l]->R() = cnoid::Quaternion(frame[i+6],
                                          frame[i+3],
                                          frame[i+4],
                                          frame[i+5]).toRotationMatrix();
        i+=7;
      }
    }
  }
  void link2Frame(const std::vector<cnoid::LinkPtr>& links, std::vector<double>& frame){
    frame.clear();
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        frame.push_back(links[l]->q());
      }else if(links[l]->isFreeJoint()) {
        frame.push_back(links[l]->p()[0]);
        frame.push_back(links[l]->p()[1]);
        frame.push_back(links[l]->p()[2]);
        cnoid::Quaternion q(links[l]->R());
        frame.push_back(q.x());
        frame.push_back(q.y());
        frame.push_back(q.z());
        frame.push_back(q.w());
      }
    }
  }


  std::set<cnoid::BodyPtr> getBodies(const std::vector<cnoid::LinkPtr>& links){
    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<links.size();i++){
      if(links[i]->body()) bodies.insert(links[i]->body());
    }
    return bodies;
  }

  std::set<cnoid::BodyPtr> getBodies(const std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> >& links){
    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<links.size();i++){
      if(links[i].second->body()) bodies.insert(links[i].second->body());
    }
    return bodies;
  }

  ompl::base::StateSpacePtr createAmbientSpace(const std::vector<cnoid::LinkPtr>& variables, double maxtranslation){
    ompl::base::StateSpacePtr ambientSpace = nullptr;
    unsigned int realVectorDim = 0;
    std::vector<double> low, high; // boundが無いとエラーになるので、一応関節角度上下限 or rootLinkの現在地+-1mで与えている
    for(int i=0;i<variables.size();i++){
      if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()) {
        realVectorDim+=1;
        low.push_back(variables[i]->q_lower());
        high.push_back(variables[i]->q_upper());
      }else if(variables[i]->isFreeJoint()) {
        if(realVectorDim > 0){
          std::shared_ptr<ompl::base::RealVectorStateSpace> realVectorStateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(realVectorDim);
          ompl::base::RealVectorBounds bounds(realVectorDim);
          bounds.low = low;
          bounds.high = high;
          realVectorStateSpace->setBounds(bounds);
          ambientSpace = ambientSpace + realVectorStateSpace;
          realVectorDim=0;
          low.clear();
          high.clear();
        }
        std::shared_ptr<ompl::base::SE3StateSpace> se3StateSpace = std::make_shared<ompl::base::SE3StateSpace>();
        ompl::base::RealVectorBounds bounds(3);
        bounds.low = std::vector<double>{variables[i]->p()[0]-maxtranslation,variables[i]->p()[1]-maxtranslation,variables[i]->p()[2]-maxtranslation};
        bounds.high = std::vector<double>{variables[i]->p()[0]+maxtranslation,variables[i]->p()[1]+maxtranslation,variables[i]->p()[2]+maxtranslation};
        se3StateSpace->setBounds(bounds);
        se3StateSpace->setStateSamplerAllocator(&allocRootStateSampler); // weightImportanceを常に1にすることで、plannerのrangeに設定した値だけ各関節(rootLink含む)が均等に動くようになる. & roll, pitchの回転を制限
        ambientSpace = ambientSpace + se3StateSpace;
      }
    }
    if(realVectorDim > 0){
      std::shared_ptr<ompl::base::RealVectorStateSpace> realVectorStateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(realVectorDim);
      ompl::base::RealVectorBounds bounds(realVectorDim);
      bounds.low = low;
      bounds.high = high;
      realVectorStateSpace->setBounds(bounds);
      ambientSpace = ambientSpace + realVectorStateSpace;
      realVectorDim=0;
      low.clear();
      high.clear();
    }

    if(ambientSpace == nullptr){
      ambientSpace = std::make_shared<ompl::base::RealVectorStateSpace>(0);
    }

    ambientSpace->setStateSamplerAllocator(&allocRootStateSampler); // weightImportanceを常に1にすることで、plannerのrangeに設定した値だけ各関節(rootLink含む)が均等に動くようになる. & roll, pitchの回転を制限
    ambientSpace->registerDefaultProjection(std::make_shared<DummyProjectionEvaluator>(ambientSpace)); // WrapperStateSpace::setup()のときに、ambientSpaceのdefaultProjectionが無いとエラーになる. CompoundStateSpaceにはDefaultProjectionが無いので、とりあえず適当に与えておく

    return ambientSpace;
  }


  void UintQueue::push(const unsigned int& m){
    {
      const std::lock_guard<std::mutex> lock(mtx_);
      queue_.push(m);
    }
    cv_.notify_one();
  }

  unsigned int UintQueue::pop(){
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [&] {return !queue_.empty(); });
    unsigned int ret = std::move(queue_.front());
    queue_.pop();
    return ret;
  }

  // ompl::base::CompoundStateSpace::allocDefaultStateSamplerとの差異は、weightImportanceが常に1である点. デフォルトを使うと、SE3StateSpaceのtranslation, rotationをsampleするときの変位が、関節(RealVectorStateSpace)をsampleするときの半分になってしまい、rootLinkが動きにくくなってしまう.
  ompl::base::StateSamplerPtr allocCnoidCompoundStateSampler(const ompl::base::StateSpace *space) {
    auto ss(std::make_shared<ompl::base::CompoundStateSampler>(space));
    for (unsigned int i = 0; i < space->as<ompl::base::CompoundStateSpace>()->getSubspaceCount(); ++i)
      ss->addSampler(space->as<ompl::base::CompoundStateSpace>()->getSubspace(i)->allocStateSampler(), 1.0/*weights_[i] / weightSum_*/);
    return ss;
  }

  void SO3RootStateSampler::sampleUniform(ompl::base::State *state) {
    double y = rng_.uniformReal(-M_PI, M_PI);
    double p = rng_.uniformReal(0, M_PI/2);
    Eigen::Quaterniond q = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY());
    state->as<ompl::base::SO3StateSpace::StateType>()->x = q.x();
    state->as<ompl::base::SO3StateSpace::StateType>()->y = q.y();
    state->as<ompl::base::SO3StateSpace::StateType>()->z = q.z();
    state->as<ompl::base::SO3StateSpace::StateType>()->w = q.w();
  }

  ompl::base::StateSamplerPtr allocRootStateSampler(const ompl::base::StateSpace *space) {
    auto ss(std::make_shared<ompl::base::CompoundStateSampler>(space));
    for (unsigned int i = 0; i < space->as<ompl::base::CompoundStateSpace>()->getSubspaceCount(); ++i) {
      if(std::dynamic_pointer_cast<ompl::base::SO3StateSpace>(space->as<ompl::base::CompoundStateSpace>()->getSubspace(i))){
        ss->addSampler(std::make_shared<SO3RootStateSampler>(space->as<ompl::base::CompoundStateSpace>()->getSubspace(i).get()), 1.0);
      }else{
        ss->addSampler(space->as<ompl::base::CompoundStateSpace>()->getSubspace(i)->allocStateSampler(), 1.0/*weights_[i] / weightSum_*/);
      }
    }
    return ss;
  }

  Eigen::Matrix3d orientCoordToAxis(const Eigen::Matrix3d& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis){
    // axisとlocalaxisはノルムが1, mは回転行列でなければならない.
    // axisとlocalaxisがピッタリ180反対向きの場合、回転方向が定まらないので不安定
    Eigen::AngleAxisd m_ = Eigen::AngleAxisd(m); // Eigen::Matrix3dの空間で積算していると数値誤差によってだんたん回転行列ではなくなってくるので
    Eigen::Vector3d localaxisdir = m_ * localaxis;
    Eigen::Vector3d cross = localaxisdir.cross(axis);
    double dot = std::min(1.0,std::max(-1.0,localaxisdir.dot(axis))); // acosは定義域外のときnanを返す
    if(cross.norm()==0){
      if(dot == -1) return Eigen::Matrix3d(-m);
      else return Eigen::Matrix3d(m_);
    }else{
      double angle = std::acos(dot); // 0~pi
      Eigen::Vector3d axis = cross.normalized(); // include sign
      return Eigen::Matrix3d(Eigen::AngleAxisd(angle, axis) * m_);
    }
  }

  Eigen::Transform<double, 3, Eigen::AffineCompact> orientCoordToAxis(const Eigen::Transform<double, 3, Eigen::AffineCompact>& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis){
    Eigen::Transform<double, 3, Eigen::AffineCompact> ret = m;
    ret.linear() = orientCoordToAxis(ret.linear(), axis, localaxis);
    return ret;
  }

  void calcHorizontal(const std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> >& horizontals){
    for(int i=0;i<horizontals.size();i++){
      horizontals[i].second->T() = orientCoordToAxis(horizontals[i].first->T(), cnoid::Vector3::UnitZ());
    }
  }
};

