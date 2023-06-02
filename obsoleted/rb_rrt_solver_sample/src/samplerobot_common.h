#ifndef SAMPLEROBOT_COMMON_H
#define SAMPLEROBOT_COMMON_H

#include <cnoid/Body>

namespace rb_rrt_solver_sample{

  void generateSampleRobot(cnoid::BodyPtr& robot, cnoid::BodyPtr& abstractRobot, cnoid::BodyPtr& horizontalRobot);

};

#endif
