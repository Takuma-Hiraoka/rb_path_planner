#include <multicontact_locomotion_planner/RobotState.h>
#include <cnoid/MeshGenerator>
#include <choreonoid_cddlib/choreonoid_cddlib.h>

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

  std::shared_ptr<ik_constraint2::IKConstraint> Mode::generateCondition(const std::unordered_map<std::string, std::shared_ptr<EndEffector> >& endEffectors, const std::shared_ptr<Environment>& environment){
    std::shared_ptr<ik_constraint2::ANDConstraint> conditions = std::make_shared<ik_constraint2::ANDConstraint>();
    //conditions->debugLevel() = 2;
    for(int i=0;i<this->rootConstraints.size();i++){
      conditions->children().push_back(this->rootConstraints[i]);
    }
    for(int i=0;i<eefs.size();i++){
      std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = this->reachabilityConstraintsSmall[i];
      std::shared_ptr<EndEffector> endEffector = endEffectors.find(this->eefs[i])->second;

      cnoid::BodyPtr supportBody;
      std::vector<std::shared_ptr<btConvexShape> > supportBulletModel;
      if(endEffector->environmentType==EndEffector::EnvironmentType::LARGESURFACE){
        supportBody = environment->largeSurfacesBody;
        supportBulletModel = environment->largeSurfacesBulletModel;
      }else if(endEffector->environmentType==EndEffector::EnvironmentType::SMALLSURFACE){
        supportBody = environment->smallSurfacesBody;
        supportBulletModel = environment->smallSurfacesBulletModel;
      }else{
        supportBody = environment->graspsBody;
        supportBulletModel = environment->graspsBulletModel;
      }

      constraint->B_link() = supportBody->rootLink();
      constraint->B_link_bulletModel() = constraint->B_link();
      constraint->B_bulletModel() = supportBulletModel;
      constraint->useSingleMeshB() = false; // support polygonを個別にチェック
      choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                  constraint->B_FACE_C(),
                                                  constraint->B_FACE_dl(),
                                                  constraint->B_FACE_du());
      constraint->updateBounds(); // キャッシュを内部に作る.
      conditions->children().push_back(constraint);
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
      std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = isLarge ? this->reachabilityConstraintsLarge[i] : this->reachabilityConstraintsSmall[i];

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
          choreonoid_cddlib::convertToFACEExpressions(constraint->B_link()->collisionShape(),
                                                      constraint->B_FACE_C(),
                                                      constraint->B_FACE_dl(),
                                                      constraint->B_FACE_du());
        }
        constraint->updateBounds();
        if(!constraint->isSatisfied()){
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
