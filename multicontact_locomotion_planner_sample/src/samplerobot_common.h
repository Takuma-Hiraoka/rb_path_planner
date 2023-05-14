#ifndef SAMPLEROBOT_COMMON_H
#define SAMPLEROBOT_COMMON_H

#include <cnoid/Body>

namespace multicontact_locomotion_planner_sample{

  void generateSampleRobot(cnoid::BodyPtr& robot, cnoid::BodyPtr& abstractRobot, cnoid::BodyPtr& horizontalRobot);

};

#endif
