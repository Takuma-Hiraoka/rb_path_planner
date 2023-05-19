#include <multicontact_locomotion_planner/RobotState.h>
#include <cnoid/MeshGenerator>

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

  bool Mode::isContactSatisfied(const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                                bool isLarge,
                                std::vector<std::string>& moveEEF,
                                std::vector<std::string>& newEEF,
                                std::vector<std::string>& excessContact){
    moveEEF.clear();
    newEEF.clear();
    excessContact.clear();

    for(int i=0;i<eefs.size();i++){
      std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = isLarge ? this->reachabilityConstraintsLarge[i] : this->reachabilityConstraintsSmall[i];

      if(currentContacts.find(eefs[i]) == currentContacts.end()){
        newEEF.push_back(eefs[i]);
      }else{

        {
          cnoid::MeshGenerator meshGenerator;
          cnoid::LinkPtr tmpLink = new cnoid::Link();
          tmpLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.001,0.001,0.001)));
          cnoid::SgGroupPtr group = new cnoid::SgGroup();
          group->addChild(shape);
          tmpLink->setShape(group);
          tmpLink->T() = currentContacts.find(eefs[i])->second->link1->T() * currentContacts.find(eefs[i])->second->localPose1;
          constraint->B_link() = tmpLink;
        }
        constraint->updateBounds();
        if(constraint->isSatisfied()){
          moveEEF.push_back(eefs[i]);
        }
      }
    }

    for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it=currentContacts.begin();it!=currentContacts.end();it++){
      if(std::find(eefs.begin(),eefs.end(),it->first) == eefs.end()){
        excessContact.push_back(it->first);
      }
    }

    if((moveEEF.size() + newEEF.size() < 2) && (moveEEF.size() + excessContact.size() < 2)){
      return true;
    }else{
      return false;
    }
  }
}
