#ifndef SAMPLEROBOT_COMMON_H
#define SAMPLEROBOT_COMMON_H

#include <cnoid/Body>
#include <multicontact_locomotion_planner/multicontact_locomotion_planner.h>

namespace multicontact_locomotion_planner_sample{

  void generateSampleRobot(const std::shared_ptr<distance_field::PropagationDistanceField>& field,
                           cnoid::BodyPtr& robot,
                           cnoid::BodyPtr& abstractRobot,
                           cnoid::BodyPtr& horizontalRobot,
                           std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> >& horizontals,
                           std::unordered_map<std::string, std::shared_ptr<multicontact_locomotion_planner::EndEffector> >& endEffectors,
                           std::unordered_map<std::string, std::shared_ptr<multicontact_locomotion_planner::Mode> >& modes,
                           std::shared_ptr<multicontact_locomotion_planner::RobotIKInfo>& robotIKInfo
                           );

};

#endif
