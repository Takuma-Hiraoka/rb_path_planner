#include "samplerobot_common.h"

#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/MeshGenerator>
#include <iostream>
#include <ros/package.h>

namespace rb_rrt_solver_sample{

  void generateSampleRobot(cnoid::BodyPtr& robot, cnoid::BodyPtr& abstractRobot, cnoid::BodyPtr& horizontalRobot) {

    cnoid::BodyLoader bodyLoader;
    cnoid::MeshGenerator meshGenerator;

    robot = bodyLoader.load(ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body");
    robot->rootLink()->p() = cnoid::Vector3(0,0,0.65);
    robot->rootLink()->v().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// rleg
        0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045,// rarm
        0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// lleg
        0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045,// larm
        0.1, 0.0, 0.0}; // torso. waist-pを少し前に傾けておくと、後ろにひっくり返りにくくなる
    for(int j=0; j < robot->numJoints(); ++j){
      robot->joint(j)->q() = reset_manip_pose[j];
    }

    robot->calcForwardKinematics();
    robot->calcCenterOfMass();

    abstractRobot = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      rootLink->setJointType(cnoid::Link::JointType::FREE_JOINT);
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

    horizontalRobot = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      rootLink->setJointType(cnoid::Link::JointType::FREE_JOINT);
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
        posTransform->translation() = cnoid::Vector3(0,-0.1,-0.25);
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
        posTransform->translation() = cnoid::Vector3(0,+0.1,-0.25);
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

  }
  

};
