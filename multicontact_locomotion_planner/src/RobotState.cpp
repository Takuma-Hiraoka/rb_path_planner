#include <multicontact_locomotion_planner/RobotState.h>

namespace multicontact_locomotion_planner{

  std::shared_ptr<Contact> EndEffector::generateContact(){
    std::shared_ptr<Contact> contact = std::make_shared<Contact>();

    contact->name = this->name;
    contact->link1 = this->parentLink;
    contact->localPose1 = this->localPose;
    contact->link2 = nullptr;
    contact->localPose2 = this->parentLink->T() * this->localPose;
    contact->ikConstraint = this->ikConstraint;
    contact->C = this->C;
    contact->dl = this->dl;
    contact->du = this->du;
    contact->preContactOffset = this->preContactOffset;
    contact->ignoreLinks = this->ignoreLinks;
    contact->ignoreBoundingBox = this->ignoreBoundingBox;
    contact->preContactAngles = this->preContactAngles;

    return contact;
  }

  std::shared_ptr<rb_rrt_solver::Condition> Mode::generateCondition(const std::unordered_map<std::string, std::shared_ptr<EndEffector> >& endEffectors, const std::shared_ptr<Environment>& environment){
    std::shared_ptr<rb_rrt_solver::ConditionAND> conditions = std::make_shared<rb_rrt_solver::ConditionAND>();
    for(int i=0;i<this->rootConstraints.size();i++){
      std::shared_ptr<rb_rrt_solver::ConditionConstraint> condition1 = std::make_shared<rb_rrt_solver::ConditionConstraint>();
      condition1->constraint = this->rootConstraints[i];
      conditions->children.push_back(condition1);
    }
    for(int i=0;i<eefs.size();i++){
      std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = this->reachabilityConstraintsSmall[i];
      std::shared_ptr<EndEffector> endEffector = endEffectors.find(this->eefs[i])->second;
      const cnoid::BodyPtr& supportBody =
        (endEffector->environmentType==EndEffector::EnvironmentType::LARGESURFACE) ? environment->largeSurfacesBody :
        (endEffector->environmentType==EndEffector::EnvironmentType::SMALLSURFACE) ? environment->smallSurfacesBody :
        environment->graspsBody;
      constraint->B_link() = supportBody->rootLink();
      constraint->B_link_vclipModel() = nullptr; // environmentのbodyは変化するので、キャッシュ削除
      constraint->updateBounds(); // キャッシュを内部に作る.
      std::shared_ptr<rb_rrt_solver::ConditionConstraint> condition1 = std::make_shared<rb_rrt_solver::ConditionConstraint>();
      condition1->constraint = constraint;
      std::shared_ptr<rb_rrt_solver::ConditionNOT> condition2 = std::make_shared<rb_rrt_solver::ConditionNOT>();
      condition2->child = condition1;
      conditions->children.push_back(condition2);
    }
    return conditions;
  }
}
