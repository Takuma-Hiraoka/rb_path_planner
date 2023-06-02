#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/MeshGenerator>
#include <iostream>
#include <ros/package.h>

#include <rb_rrt_solver/rb_rrt_solver.h>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>

#include "samplerobot_common.h"

namespace rb_rrt_solver_sample{
  void sample2_root(){
    cnoid::BodyPtr robot;
    cnoid::BodyPtr abstractRobot;
    cnoid::BodyPtr horizontalRobot;

    generateSampleRobot(robot, abstractRobot, horizontalRobot);

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(robot);
    viewer->objects(abstractRobot);
    viewer->objects(horizontalRobot);

    viewer->drawObjects();

  }

}
