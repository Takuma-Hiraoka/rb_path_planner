#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/MeshGenerator>
#include <iostream>
#include <ros/package.h>

#include <rb_rrt_solver/rb_rrt_solver.h>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>

namespace rb_rrt_solver_sample{
  void sample2_root(){
    cnoid::BodyLoader bodyLoader;
    cnoid::MeshGenerator meshGenerator;

    cnoid::BodyPtr robot = bodyLoader.load(ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body");
    robot->rootLink()->p() = cnoid::Vector3(0,0,0.6);
    robot->rootLink()->v().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();

    cnoid::BodyPtr abstractRobot = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateCylinder(0.25/*radius*/, 0.9/*height*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.3);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,0,0.35);
        posTransform->rotation() = cnoid::AngleAxis(M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rootLink->setShape(group);
      }
      {
        cnoid::LinkPtr rarmLink = new cnoid::Link();
        rarmLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rarmLink->setOffsetTranslation(robot->link("RARM_SHOULDER_P")->p() - robot->rootLink()->p());
        rarmLink->setName("RARM");
        rootLink->appendChild(rarmLink);
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateSphere(0.4/*radius*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.6);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.4,0,-0.1);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rarmLink->setShape(group);
      }
      {
        cnoid::LinkPtr larmLink = new cnoid::Link();
        larmLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        larmLink->setOffsetTranslation(robot->link("LARM_SHOULDER_P")->p() - robot->rootLink()->p());
        larmLink->setName("LARM");
        rootLink->appendChild(larmLink);
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateSphere(0.4/*radius*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.6);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.4,0,-0.1);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        larmLink->setShape(group);
      }
      {
        cnoid::LinkPtr rlegLink = new cnoid::Link();
        rlegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rlegLink->setOffsetTranslation(robot->link("RLEG_HIP_Y")->p() - robot->rootLink()->p());
        rlegLink->setName("RLEG");
        rootLink->appendChild(rlegLink);
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateSphere(0.4/*radius*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.6);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.2,0,-0.0);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rlegLink->setShape(group);
      }
      {
        cnoid::LinkPtr llegLink = new cnoid::Link();
        llegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        llegLink->setOffsetTranslation(robot->link("LLEG_HIP_Y")->p() - robot->rootLink()->p());
        llegLink->setName("LLEG");
        rootLink->appendChild(llegLink);
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateSphere(0.4/*radius*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.6);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.2,0,-0.0);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        llegLink->setShape(group);
      }
      abstractRobot->setRootLink(rootLink);
    }
    abstractRobot->rootLink()->T() = robot->rootLink()->T();
    abstractRobot->calcForwardKinematics();
    abstractRobot->calcCenterOfMass();

    cnoid::BodyPtr horizontalRobot = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        cnoid::LinkPtr rlegLink = new cnoid::Link();
        rlegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rlegLink->setOffsetTranslation(robot->link("RLEG_HIP_Y")->p() - robot->rootLink()->p());
        rlegLink->setName("RLEG");
        rootLink->appendChild(rlegLink);
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateCylinder(0.2/*radius*/, 0.2/*height*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.6);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,-0.1,-0.15);
        posTransform->rotation() = cnoid::AngleAxis(M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rlegLink->setShape(group);
      }
      {
        cnoid::LinkPtr llegLink = new cnoid::Link();
        llegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        llegLink->setOffsetTranslation(robot->link("LLEG_HIP_Y")->p() - robot->rootLink()->p());
        llegLink->setName("LLEG");
        rootLink->appendChild(llegLink);
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateCylinder(0.2/*radius*/, 0.2/*height*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.6);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,+0.1,-0.15);
        posTransform->rotation() = cnoid::AngleAxis(M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        llegLink->setShape(group);
      }
      horizontalRobot->setRootLink(rootLink);
    }
    horizontalRobot->rootLink()->T() = robot->rootLink()->T();
    horizontalRobot->calcForwardKinematics();
    horizontalRobot->calcCenterOfMass();

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(robot);
    viewer->objects(abstractRobot);
    viewer->objects(horizontalRobot);

    viewer->drawObjects();

  }

}
