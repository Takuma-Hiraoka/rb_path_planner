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
  void sample4_ladder(){
    cnoid::BodyPtr obstacle;
    std::shared_ptr<multicontact_locomotion_planner::Environment> environment;
    generateLadderWorld(obstacle, environment);

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

    // robot->rootLink()->R() = cnoid::AngleAxis(1.57, cnoid::Vector3::UnitZ()) * robot->rootLink()->R();
    // abstractRobot->rootLink()->T() = robot->rootLink()->T();
    // horizontalRobot->rootLink()->T() = robot->rootLink()->T();

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
    param.debugLevel = 2;

    //param.robotIKInfo->pikParam.debugLevel = 2;
    param.robotIKInfo->gikParam.viewer = viewer;
    param.rbrrtParam.viewer = viewer;
    param.rbrrtParam.drawLoop = 10; // 1drawに10msくらいかかることに注意
    param.rbrrtParam.maxTranslation = 2.0;
    param.rbrrtParam.threads = 10;
    //param.rbrrtParam.pikParam.debugLevel = 1;
    param.robotIKInfo->gikParam.threads = 10;
    param.robotIKInfo->gikParam.timeout = 20;


    std::vector<std::pair<std::vector<double>, std::string> > targetRootPath;
    cnoid::Isometry3 goal = abstractRobot->rootLink()->T();
    // goal.translation() += cnoid::Vector3(1.0,0.0,1.5);
    // goal.linear() = cnoid::AngleAxis(M_PI/2, cnoid::Vector3::UnitY()).toRotationMatrix();
    goal.translation() += cnoid::Vector3(1.5,0.0,1.8);

    multicontact_locomotion_planner::solveRBRRT(environment,
                                                currentContacts,
                                                goal,
                                                param,
                                                targetRootPath
                                                );

    for(int i=0;i<targetRootPath.size();i++){
      multicontact_locomotion_planner::frame2Link(targetRootPath[i].first,std::vector<cnoid::LinkPtr>{abstractRobot->rootLink()});
      abstractRobot->calcForwardKinematics(false);
      abstractRobot->calcCenterOfMass();
      multicontact_locomotion_planner::calcHorizontal(horizontals);
      horizontalRobot->calcForwardKinematics(false);
      horizontalRobot->calcCenterOfMass();

      viewer->drawObjects();

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    multicontact_locomotion_planner::calcAssoc(assocs);
    abstractRobot->calcForwardKinematics(false);
    abstractRobot->calcCenterOfMass();
    multicontact_locomotion_planner::calcHorizontal(horizontals);
    horizontalRobot->calcForwardKinematics(false);
    horizontalRobot->calcCenterOfMass();



    std::vector<std::vector<multicontact_locomotion_planner::RobotState> > path;

    for(int i=0;i<70;i++){
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

      prevSwingEEF = swingEEF;
      if(tmp_path.size() > 0) {
        path.push_back(tmp_path);
        currentContacts = tmp_path.back().contacts;
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
    viewer->objects(std::vector<cnoid::BodyPtr>{robot,obstacle,environment->largeSurfacesBody,environment->smallSurfacesBody,environment->graspsBody});

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
