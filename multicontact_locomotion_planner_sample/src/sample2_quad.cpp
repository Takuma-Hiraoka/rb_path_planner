#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/MeshGenerator>
#include <iostream>
#include <ros/package.h>

#include <multicontact_locomotion_planner/multicontact_locomotion_planner.h>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>

#include "samplerobot_common.h"
#include "world_common.h"

namespace multicontact_locomotion_planner_sample{
  void sample2_quad(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<multicontact_locomotion_planner::Environment> environment;
    generateStepWorld(obstacle, environment);

    cnoid::BodyPtr robot;
    cnoid::BodyPtr abstractRobot;
    cnoid::BodyPtr horizontalRobot;
    std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > assocs;
    std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > horizontals;
    std::unordered_map<std::string, std::shared_ptr<multicontact_locomotion_planner::EndEffector> > endEffectors;
    std::unordered_map<std::string, std::shared_ptr<multicontact_locomotion_planner::Mode> > modes;
    std::shared_ptr<multicontact_locomotion_planner::RobotIKInfo> robotIKInfo;
    generateSampleRobot(environment->obstacles,
                        robot,
                        abstractRobot,
                        horizontalRobot,
                        assocs,
                        horizontals,
                        endEffectors,
                        modes,
                        robotIKInfo);

    cnoid::MeshGenerator meshGenerator;


    std::vector<std::pair<std::vector<double>, std::string> > targetRootPath;
    {
      std::vector<double> currentAngle;
      multicontact_locomotion_planner::link2Frame(std::vector<cnoid::LinkPtr>{robot->rootLink()}, currentAngle);
      std::vector<double> angle;
      multicontact_locomotion_planner::link2Frame(std::vector<cnoid::LinkPtr>{robot->rootLink()}, angle);
      targetRootPath.push_back(std::pair<std::vector<double>, std::string>(angle, "biped"));

      for(int i=0;i<8;i++){
        robot->rootLink()->translation() += cnoid::Vector3(0.0,0.0,-0.02);
        robot->rootLink()->R() = robot->rootLink()->R() * Eigen::AngleAxisd(M_PI/16,Eigen::Vector3d::UnitY());
        multicontact_locomotion_planner::link2Frame(std::vector<cnoid::LinkPtr>{robot->rootLink()}, angle);
        targetRootPath.push_back(std::pair<std::vector<double>, std::string>(angle, "biped"));
      }

      targetRootPath.push_back(std::pair<std::vector<double>, std::string>(targetRootPath.back().first, "quadruped_large"));

      for(int i=0;i<30;i++){
        robot->rootLink()->translation() += cnoid::Vector3(-0.05,0.0,0.0);
        multicontact_locomotion_planner::link2Frame(std::vector<cnoid::LinkPtr>{robot->rootLink()}, angle);
        targetRootPath.push_back(std::pair<std::vector<double>, std::string>(angle, "quadruped"));
      }

      targetRootPath.push_back(std::pair<std::vector<double>, std::string>(targetRootPath.back().first, "quadruped_large"));

      for(int i=0;i<8;i++){
        robot->rootLink()->translation() += cnoid::Vector3(0.0,0.0,0.02);
        robot->rootLink()->R() = robot->rootLink()->R() * Eigen::AngleAxisd(-M_PI/16,Eigen::Vector3d::UnitY());
        multicontact_locomotion_planner::link2Frame(std::vector<cnoid::LinkPtr>{robot->rootLink()}, angle);
        targetRootPath.push_back(std::pair<std::vector<double>, std::string>(angle, "biped"));
      }

      multicontact_locomotion_planner::frame2Link(currentAngle, std::vector<cnoid::LinkPtr>{robot->rootLink()});
    }

    std::unordered_map<std::string, std::shared_ptr<multicontact_locomotion_planner::Contact> > currentContacts;
    currentContacts["rleg"] = endEffectors["rleg"]->generateContact();
    currentContacts["lleg"] = endEffectors["lleg"]->generateContact();

    std::string prevSwingEEF = "";

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(robot);
    viewer->objects(abstractRobot);
    viewer->objects(horizontalRobot);
    viewer->objects(obstacle);
    viewer->objects(environment->largeSurfacesBody);
    viewer->objects(environment->smallSurfacesBody);
    viewer->objects(environment->graspsBody);

    viewer->drawObjects();

    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(robot->rootLink());
    for(int i=0;i<robot->numJoints();i++){
      variables.push_back(robot->joint(i));
    }

    multicontact_locomotion_planner::MLPParam param;
    param.viewer = viewer;
    param.robot = robot;
    param.abstractRobot = abstractRobot;
    param.horizontalRobot = horizontalRobot;
    param.assocs = assocs;
    param.horizontals = horizontals;
    param.endEffectors = endEffectors;
    param.modes = modes;
    param.robotIKInfo = robotIKInfo;
    param.debugLevel = 3;
    //param.debugLevel = 2;

    //param.robotIKInfo->pikParam.debugLevel = 2;
    param.robotIKInfo->gikParam.viewer = viewer;

    std::vector<std::vector<multicontact_locomotion_planner::RobotState> > path;

    for(int i=0;i<30;i++){
      std::vector<multicontact_locomotion_planner::RobotState> tmp_path;
      std::string swingEEF;
      multicontact_locomotion_planner::solveMLP(robot,
                                                environment,
                                                currentContacts,
                                                prevSwingEEF,
                                                targetRootPath,
                                                variables,
                                                tmp_path,
                                                swingEEF,
                                                param
                                                );
      if(tmp_path.size() > 0) {
        path.push_back(tmp_path);
        currentContacts = tmp_path.back().contacts;
        prevSwingEEF = swingEEF;
      }

      cnoid::Vector3 targetp(targetRootPath.back().first[0],targetRootPath.back().first[1],targetRootPath.back().first[2]);
      cnoid::Quaternion targetR(targetRootPath.back().first[6],
                                targetRootPath.back().first[3],
                                targetRootPath.back().first[4],
                                targetRootPath.back().first[5]);
      double dist = std::sqrt((robot->rootLink()->p()-targetp).squaredNorm() + std::pow(cnoid::AngleAxis(robot->rootLink()->R().transpose()*targetR).angle(),2));
      if(dist < 0.1) break;
    }

    // main loop
    for(int i=0;i<path.size();i++){
      for(int j=0;j<path[i].size();j++){
        multicontact_locomotion_planner::frame2Link(path[i][j].jointAngle,variables);

        robot->calcForwardKinematics(false);
        robot->calcCenterOfMass();

        multicontact_locomotion_planner::calcAssoc(assocs);
        abstractRobot->calcForwardKinematics(false);
        abstractRobot->calcCenterOfMass();
        multicontact_locomotion_planner::calcHorizontal(horizontals);
        horizontalRobot->calcForwardKinematics(false);
        horizontalRobot->calcCenterOfMass();

        viewer->drawObjects();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

  }

}
