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

namespace multicontact_locomotion_planner_sample{
  void sample1_walk(){
    cnoid::BodyPtr robot;
    cnoid::BodyPtr abstractRobot;
    cnoid::BodyPtr horizontalRobot;
    generateSampleRobot(robot, abstractRobot, horizontalRobot);

    cnoid::MeshGenerator meshGenerator;
    cnoid::BodyPtr obstacle = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,1,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0,-0.05);
          posTransform->addChild(shape);
          group->addChild(posTransform);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(1,1,0.1)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1,0,0.35);
          posTransform->addChild(shape);
          group->addChild(posTransform);
        }
        rootLink->setShape(group);
      }
      obstacle->setRootLink(rootLink);
    }

    // collision world
    std::shared_ptr<distance_field::PropagationDistanceField> field = std::make_shared<distance_field::PropagationDistanceField>(5,//size_x
                                                                                                                                 5,//size_y
                                                                                                                                 5,//size_z
                                                                                                                                 0.04,//resolution
                                                                                                                                 -2.5,//origin_x
                                                                                                                                 -2.5,//origin_y
                                                                                                                                 -2.5,//origin_z
                                                                                                                                 0.5, // max_distance
                                                                                                                                 false// propagate_negative_distances
                                                                                                                                 );
    EigenSTL::vector_Vector3d vertices;
    for(int i=0;i<obstacle->numLinks();i++){
      std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.04);
      for(int j=0;j<vertices_.size();j++){
        vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
      }
    }
    field->addPointsToField(vertices);



    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(robot);
    viewer->objects(abstractRobot);
    viewer->objects(horizontalRobot);
    viewer->objects(obstacle);

    viewer->drawObjects();

    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(abstractRobot->rootLink());

    std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > horizontals;
    horizontals.push_back(std::make_pair<cnoid::LinkPtr, cnoid::LinkPtr>(abstractRobot->rootLink(), horizontalRobot->rootLink()));


    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();

    // main loop
    for(int i=0;i<path->size();i++){
      multicontact_locomotion_planner::frame2Link(path->at(i),variables);
      abstractRobot->calcForwardKinematics(false);
      abstractRobot->calcCenterOfMass();

      robot->rootLink()->T() = abstractRobot->rootLink()->T();
      robot->calcForwardKinematics(false);
      robot->calcCenterOfMass();
      multicontact_locomotion_planner::calcHorizontal(horizontals);
      horizontalRobot->calcForwardKinematics(false);
      horizontalRobot->calcCenterOfMass();

      viewer->drawObjects();

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  }

}
