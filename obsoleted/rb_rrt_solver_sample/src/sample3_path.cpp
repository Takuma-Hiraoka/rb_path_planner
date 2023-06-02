#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/MeshGenerator>
#include <iostream>
#include <ros/package.h>

#include <rb_rrt_solver/rb_rrt_solver.h>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>

#include "samplerobot_common.h"

namespace rb_rrt_solver_sample{
  void sample3_path(){
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

    // support polygon
    cnoid::BodyPtr support = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          cnoid::MeshGenerator::Extrusion extrusion;
          extrusion.crossSection.push_back(cnoid::Vector2(+0.4,+0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(-0.4,+0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(-0.4,-0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(+0.4,-0.4));
          extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
          extrusion.spine.push_back(cnoid::Vector3(0,0,0));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          shape->setMesh(meshGenerator.generateExtrusion(extrusion));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0,0,0.0);
          posTransform->addChild(shape);
          group->addChild(posTransform);
        }
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          cnoid::MeshGenerator::Extrusion extrusion;
          extrusion.crossSection.push_back(cnoid::Vector2(+0.4,+0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(-0.4,+0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(-0.4,-0.4));
          extrusion.crossSection.push_back(cnoid::Vector2(+0.4,-0.4));
          extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
          extrusion.spine.push_back(cnoid::Vector3(0,0,0));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          shape->setMesh(meshGenerator.generateExtrusion(extrusion));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(1.0,0,0.4);
          posTransform->addChild(shape);
          group->addChild(posTransform);
        }
        rootLink->setShape(group);
      }
      support->setRootLink(rootLink);
    }


    std::shared_ptr<rb_rrt_solver::ConditionAND> conditions = std::make_shared<rb_rrt_solver::ConditionAND>();
    {
      // root collision
      std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
      constraint->A_link() = abstractRobot->rootLink();
      constraint->field() = field;
      constraint->tolerance() = 0.04;
      constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
      std::shared_ptr<rb_rrt_solver::ConditionConstraint> condition1 = std::make_shared<rb_rrt_solver::ConditionConstraint>();
      condition1->constraint = constraint;
      conditions->children.push_back(condition1);
    }

    std::shared_ptr<rb_rrt_solver::ConditionOR> supportconditions = std::make_shared<rb_rrt_solver::ConditionOR>();
    conditions->children.push_back(supportconditions);
    {
      // 両足起立
      std::shared_ptr<rb_rrt_solver::ConditionAND> condition1 = std::make_shared<rb_rrt_solver::ConditionAND>();
      supportconditions->children.push_back(condition1);
      {
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = horizontalRobot->link("RLEG");
        constraint->B_link() = support->rootLink();
        constraint->tolerance() = 0.01;
        constraint->useSingleMesh() = false; // support polygonを個別にチェック
        constraint->debugLevel() = 0;
        constraint->updateBounds(); // キャッシュを内部に作る.
        std::shared_ptr<rb_rrt_solver::ConditionConstraint> condition2 = std::make_shared<rb_rrt_solver::ConditionConstraint>();
        condition2->constraint = constraint;
        std::shared_ptr<rb_rrt_solver::ConditionNOT> condition3 = std::make_shared<rb_rrt_solver::ConditionNOT>();
        condition3->child = condition2;
        condition1->children.push_back(condition3);
      }
      {
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = horizontalRobot->link("LLEG");
        constraint->B_link() = support->rootLink();
        constraint->tolerance() = 0.01;
        constraint->useSingleMesh() = false; // support polygonを個別にチェック
        constraint->debugLevel() = 0;
        constraint->updateBounds(); // キャッシュを内部に作る.
        std::shared_ptr<rb_rrt_solver::ConditionConstraint> condition2 = std::make_shared<rb_rrt_solver::ConditionConstraint>();
        condition2->constraint = constraint;
        std::shared_ptr<rb_rrt_solver::ConditionNOT> condition3 = std::make_shared<rb_rrt_solver::ConditionNOT>();
        condition3->child = condition2;
        condition1->children.push_back(condition3);
      }
    }
    {
      // 多点接触
      std::shared_ptr<rb_rrt_solver::ConditionAND> condition1 = std::make_shared<rb_rrt_solver::ConditionAND>();
      supportconditions->children.push_back(condition1);
      {
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = abstractRobot->link("RLEG");
        constraint->B_link() = support->rootLink();
        constraint->tolerance() = 0.01;
        constraint->useSingleMesh() = false; // support polygonを個別にチェック
        constraint->debugLevel() = 0;
        constraint->updateBounds(); // キャッシュを内部に作る.
        std::shared_ptr<rb_rrt_solver::ConditionConstraint> condition2 = std::make_shared<rb_rrt_solver::ConditionConstraint>();
        condition2->constraint = constraint;
        std::shared_ptr<rb_rrt_solver::ConditionNOT> condition3 = std::make_shared<rb_rrt_solver::ConditionNOT>();
        condition3->child = condition2;
        condition1->children.push_back(condition3);
      }
      {
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = abstractRobot->link("LLEG");
        constraint->B_link() = support->rootLink();
        constraint->tolerance() = 0.01;
        constraint->useSingleMesh() = false; // support polygonを個別にチェック
        constraint->debugLevel() = 0;
        constraint->updateBounds(); // キャッシュを内部に作る.
        std::shared_ptr<rb_rrt_solver::ConditionConstraint> condition2 = std::make_shared<rb_rrt_solver::ConditionConstraint>();
        condition2->constraint = constraint;
        std::shared_ptr<rb_rrt_solver::ConditionNOT> condition3 = std::make_shared<rb_rrt_solver::ConditionNOT>();
        condition3->child = condition2;
        condition1->children.push_back(condition3);
      }
      {
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = abstractRobot->link("RARM");
        constraint->B_link() = support->rootLink();
        constraint->tolerance() = 0.01;
        constraint->useSingleMesh() = false; // support polygonを個別にチェック
        constraint->debugLevel() = 0;
        constraint->updateBounds(); // キャッシュを内部に作る.
        std::shared_ptr<rb_rrt_solver::ConditionConstraint> condition2 = std::make_shared<rb_rrt_solver::ConditionConstraint>();
        condition2->constraint = constraint;
        std::shared_ptr<rb_rrt_solver::ConditionNOT> condition3 = std::make_shared<rb_rrt_solver::ConditionNOT>();
        condition3->child = condition2;
        condition1->children.push_back(condition3);
      }
      {
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = abstractRobot->link("LARM");
        constraint->B_link() = support->rootLink();
        constraint->tolerance() = 0.01;
        constraint->useSingleMesh() = false; // support polygonを個別にチェック
        constraint->debugLevel() = 0;
        constraint->updateBounds(); // キャッシュを内部に作る.
        std::shared_ptr<rb_rrt_solver::ConditionConstraint> condition2 = std::make_shared<rb_rrt_solver::ConditionConstraint>();
        condition2->constraint = constraint;
        std::shared_ptr<rb_rrt_solver::ConditionNOT> condition3 = std::make_shared<rb_rrt_solver::ConditionNOT>();
        condition3->child = condition2;
        condition1->children.push_back(condition3);
      }
    }


    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(robot);
    viewer->objects(abstractRobot);
    viewer->objects(horizontalRobot);
    viewer->objects(obstacle);
    viewer->objects(support);

    viewer->drawObjects();

    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(abstractRobot->rootLink());

    std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > horizontals;
    horizontals.push_back(std::make_pair<cnoid::LinkPtr, cnoid::LinkPtr>(abstractRobot->rootLink(), horizontalRobot->rootLink()));

    std::vector<double> goal;
    {
      cnoid::Position org = abstractRobot->rootLink()->T();
      abstractRobot->rootLink()->translation() += cnoid::Vector3(1.0,0.0,0.4);
      rb_rrt_solver::link2Frame(variables, goal);
      abstractRobot->rootLink()->T() = org;
    }

    rb_rrt_solver::RBRRTParam param;
    param.viewer = viewer;
    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
    bool solved = rb_rrt_solver::solveRBRRT(variables,
                                            horizontals,
                                            conditions,
                                            goal,
                                            path,
                                            param
                                            );

    // main loop
    for(int i=0;i<path->size();i++){
      rb_rrt_solver::frame2Link(path->at(i),variables);
      abstractRobot->calcForwardKinematics(false);
      abstractRobot->calcCenterOfMass();

      robot->rootLink()->T() = abstractRobot->rootLink()->T();
      robot->calcForwardKinematics(false);
      robot->calcCenterOfMass();
      rb_rrt_solver::calcHorizontal(horizontals);
      horizontalRobot->calcForwardKinematics(false);
      horizontalRobot->calcCenterOfMass();

      viewer->drawObjects();

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

  }

}
