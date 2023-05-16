#ifndef WORLD_COMMON_H
#define WORLD_COMMON_H

#include <cnoid/Body>
#include <multicontact_locomotion_planner/multicontact_locomotion_planner.h>

namespace multicontact_locomotion_planner_sample{

  void generateStepWorld(cnoid::BodyPtr& obstacle, // for visual
                         std::shared_ptr<multicontact_locomotion_planner::Environment>& environment
                         );

  void generateLadderWorld(cnoid::BodyPtr& obstacle, // for visual
                           std::shared_ptr<multicontact_locomotion_planner::Environment>& environment
                           );

};

#endif
