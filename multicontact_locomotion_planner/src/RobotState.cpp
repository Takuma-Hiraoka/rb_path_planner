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
    contact->ld = this->ld;
    contact->ud = this->ud;
    contact->preContactOffset = this->preContactOffset;
    contact->ignoreLinks = this->ignoreLinks;
    contact->ignoreBoundingBox = this->ignoreBoundingBox;
    contact->preContactAngles = this->preContactAngles;

    return contact;
  }
}
