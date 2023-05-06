#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <iostream>
#include <ros/package.h>

#include <rb_rrt_solver/rb_rrt_solver.h>
#include <ik_constraint2/ik_constraint2.h>

namespace rb_rrt_solver_sample{
  void sample1_4limb(){
    // load robot
    std::string modelfile = ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body";
    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(modelfile);

    // reset manip pose
    robot->rootLink()->p() = cnoid::Vector3(0,0,0.6);
    robot->rootLink()->v().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// rleg
        0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045,// rarm
        0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// lleg
        0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045,// larm
        0.0, 0.0, 0.0};

    for(int j=0; j < robot->numJoints(); ++j){
      robot->joint(j)->q() = reset_manip_pose[j];
    }
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints;

    // setup viewer
    choreonoid_viewer::Viewer viewer;
    viewer.objects(robot);

    // main loop
    std::vector<cnoid::SgNodePtr> markers;
    for(int j=0;j<constraints.size();j++){
      for(int k=0;k<constraints[j].size(); k++){
        constraints[j][k]->updateBounds();
        const std::vector<cnoid::SgNodePtr>& marker = constraints[j][k]->getDrawOnObjects();
        std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
      }
    }
    viewer.drawOn(markers);
    viewer.drawObjects();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  }

}
