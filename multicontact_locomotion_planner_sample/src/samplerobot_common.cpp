#include "samplerobot_common.h"

#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <cnoid/MeshGenerator>
#include <iostream>
#include <ros/package.h>
#include <convex_polyhedron_intersection/convex_polyhedron_intersection.h>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <choreonoid_cddlib/choreonoid_cddlib.h>

namespace multicontact_locomotion_planner_sample{

  void generateSampleRobot(const std::shared_ptr<distance_field::PropagationDistanceField>& field,
                           cnoid::BodyPtr& robot,
                           cnoid::BodyPtr& abstractRobot,
                           cnoid::BodyPtr& horizontalRobot,
                           std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> >& assocs,
                           std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> >& horizontals,
                           std::unordered_map<std::string, std::shared_ptr<multicontact_locomotion_planner::EndEffector> >& endEffectors,
                           std::unordered_map<std::string, std::shared_ptr<multicontact_locomotion_planner::Mode> >& modes,
                           std::shared_ptr<multicontact_locomotion_planner::RobotIKInfo>& robotIKInfo
                           ){

    cnoid::BodyLoader bodyLoader;
    cnoid::MeshGenerator meshGenerator;
    meshGenerator.setDivisionNumber(8); // default 20. 20だとcddlibが遅い

    robot = bodyLoader.load(ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body");

    // 梯子ではcrotch-p, knee-pのrangeが必要
    robot->link("RLEG_HIP_P")->setJointRange(-150.0/180.0*M_PI, 30.0/180.0*M_PI);
    robot->link("LLEG_HIP_P")->setJointRange(-150.0/180.0*M_PI, 30.0/180.0*M_PI);
    robot->link("RLEG_KNEE")->setJointRange(robot->link("RLEG_KNEE")->q_lower(), 150.0/180.0*M_PI);
    robot->link("LLEG_KNEE")->setJointRange(robot->link("LLEG_KNEE")->q_lower(), 150.0/180.0*M_PI);

    // 胴の自由度は少ないほうが変な姿勢にならないので解けやすい. 股関節の逆クルンを防ぐ
    robot->link("WAIST_P")->setJointRange(-30.0/180.0*M_PI, 60.0/180.0*M_PI);
    robot->link("WAIST_R")->setJointRange(-30.0/180.0*M_PI, 30.0/180.0*M_PI);
    robot->link("CHEST")->setJointRange(-30.0/180.0*M_PI, 30.0/180.0*M_PI);
    robot->link("RLEG_HIP_R")->setJointRange(-45.0/180.0*M_PI, 30.0/180.0*M_PI);
    robot->link("LLEG_HIP_R")->setJointRange(-30.0/180.0*M_PI, 45.0/180.0*M_PI);
    robot->link("RLEG_HIP_Y")->setJointRange(-60.0/180.0*M_PI, 60.0/180.0*M_PI);
    robot->link("LLEG_HIP_Y")->setJointRange(-60.0/180.0*M_PI, 60.0/180.0*M_PI);

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
        shape->setMesh(meshGenerator.generateCylinder(0.25/*radius*/, 1.1/*height*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,0,0.3); // ある程度低くないといけない
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
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateSphere(0.55/*radius*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.2,1.2,1.2)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(0,-0.4,0);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.3,0,0.1);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rarmLink->setShape(group);
      }
      {
        cnoid::LinkPtr rarmLink = new cnoid::Link();
        rarmLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rarmLink->setOffsetTranslation(robot->link("RARM_SHOULDER_P")->p() - robot->rootLink()->p());
        rarmLink->setName("RARMSMALL");
        rootLink->appendChild(rarmLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateSphere(0.5/*radius*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.2,1.2,1.2)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(0,-0.45,0);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.3,0,0.1);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rarmLink->setShape(group);
      }
      {
        cnoid::LinkPtr rarmLink = new cnoid::Link();
        rarmLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rarmLink->setOffsetTranslation(robot->link("RARM_SHOULDER_P")->p() - robot->rootLink()->p());
        rarmLink->setName("RARMUPPER");
        rootLink->appendChild(rarmLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateSphere(0.55/*radius*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.2,1.2,1.2)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(0,-0.4,0.25);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.3,0,0.1);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rarmLink->setShape(group);
      }
      {
        cnoid::LinkPtr rarmLink = new cnoid::Link();
        rarmLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rarmLink->setOffsetTranslation(robot->link("RARM_SHOULDER_P")->p() - robot->rootLink()->p());
        rarmLink->setName("RARMUPPERSMALL");
        rootLink->appendChild(rarmLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateSphere(0.5/*radius*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.2,1.2,1.2)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(0,-0.45,0.3);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.3,0,0.1);
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
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateSphere(0.55/*radius*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.2,1.2,1.2)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(0,0.4,0);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.3,0,0.1);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        larmLink->setShape(group);
      }
      {
        cnoid::LinkPtr larmLink = new cnoid::Link();
        larmLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        larmLink->setOffsetTranslation(robot->link("LARM_SHOULDER_P")->p() - robot->rootLink()->p());
        larmLink->setName("LARMSMALL");
        rootLink->appendChild(larmLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateSphere(0.5/*radius*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.2,1.2,1.2)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(0,0.45,0);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.3,0,0.1);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        larmLink->setShape(group);
      }
      {
        cnoid::LinkPtr larmLink = new cnoid::Link();
        larmLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        larmLink->setOffsetTranslation(robot->link("LARM_SHOULDER_P")->p() - robot->rootLink()->p());
        larmLink->setName("LARMUPPER");
        rootLink->appendChild(larmLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateSphere(0.55/*radius*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.2,1.2,1.2)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(0,0.4,0.25);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.3,0,0.1);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        larmLink->setShape(group);
      }
      {
        cnoid::LinkPtr larmLink = new cnoid::Link();
        larmLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        larmLink->setOffsetTranslation(robot->link("LARM_SHOULDER_P")->p() - robot->rootLink()->p());
        larmLink->setName("LARMUPPERSMALL");
        rootLink->appendChild(larmLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateSphere(0.5/*radius*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.2,1.2,1.2)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(0,0.45,0.3);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.3,0,0.1);
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        larmLink->setShape(group);
      }
      abstractRobot->setRootLink(rootLink);
    }
    abstractRobot->rootLink()->T() = robot->rootLink()->T();
    abstractRobot->calcForwardKinematics();
    abstractRobot->calcCenterOfMass();
    assocs.clear();
    assocs.push_back(std::make_pair<cnoid::LinkPtr, cnoid::LinkPtr>(robot->rootLink(), abstractRobot->rootLink()));

    horizontalRobot = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      rootLink->setJointType(cnoid::Link::JointType::FREE_JOINT);
      {
        cnoid::LinkPtr rlegLink = new cnoid::Link();
        rlegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rlegLink->setOffsetTranslation(robot->link("RLEG_HIP_P")->p() - robot->rootLink()->p());
        rlegLink->setName("RLEGSTAND");
        rootLink->appendChild(rlegLink);
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateCylinder(0.15/*radius*/, 0.4/*height*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,-0.1,-0.55);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rlegLink->setShape(group);
      }
      {
        cnoid::LinkPtr rlegLink = new cnoid::Link();
        rlegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rlegLink->setOffsetTranslation(robot->link("RLEG_HIP_P")->p() - robot->rootLink()->p());
        rlegLink->setName("RLEGSTANDSMALL");
        rootLink->appendChild(rlegLink);
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateCylinder(0.1/*radius*/, 0.2/*height*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,-0.1,-0.55);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rlegLink->setShape(group);
      }
      {
        cnoid::LinkPtr llegLink = new cnoid::Link();
        llegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        llegLink->setOffsetTranslation(robot->link("LLEG_HIP_P")->p() - robot->rootLink()->p());
        llegLink->setName("LLEGSTAND");
        rootLink->appendChild(llegLink);
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateCylinder(0.15/*radius*/, 0.4/*height*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,+0.1,-0.55);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        llegLink->setShape(group);
      }
      {
        cnoid::LinkPtr llegLink = new cnoid::Link();
        llegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        llegLink->setOffsetTranslation(robot->link("LLEG_HIP_P")->p() - robot->rootLink()->p());
        llegLink->setName("LLEGSTANDSMALL");
        rootLink->appendChild(llegLink);
        cnoid::SgShapePtr shape = new cnoid::SgShape();
        shape->setMesh(meshGenerator.generateCylinder(0.1/*radius*/, 0.2/*height*/));
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0,+0.1,-0.55);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        llegLink->setShape(group);
      }
      {
        cnoid::LinkPtr rlegLink = new cnoid::Link();
        rlegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rlegLink->setOffsetTranslation(robot->link("RLEG_HIP_P")->p() - robot->rootLink()->p());
        rlegLink->setName("RLEGBACK");
        rootLink->appendChild(rlegLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateCylinder(0.5/*radius*/, 0.6/*height*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,1.0,0.5)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(-0.4,0,-0.1);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.2,0,-0.4);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rlegLink->setShape(group);
      }
      {
        cnoid::LinkPtr rlegLink = new cnoid::Link();
        rlegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rlegLink->setOffsetTranslation(robot->link("RLEG_HIP_P")->p() - robot->rootLink()->p());
        rlegLink->setName("RLEGBACKSMALL");
        rootLink->appendChild(rlegLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateCylinder(0.45/*radius*/, 0.5/*height*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,1.0,0.4)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(-0.45,0,-0.15);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.2,0,-0.4);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rlegLink->setShape(group);
      }
      {
        cnoid::LinkPtr llegLink = new cnoid::Link();
        llegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        llegLink->setOffsetTranslation(robot->link("LLEG_HIP_P")->p() - robot->rootLink()->p());
        llegLink->setName("LLEGBACK");
        rootLink->appendChild(llegLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateCylinder(0.5/*radius*/, 0.6/*height*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,1.0,0.5)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(-0.4,0,0.1);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.2,0,-0.4);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        llegLink->setShape(group);
      }
      {
        cnoid::LinkPtr llegLink = new cnoid::Link();
        llegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        llegLink->setOffsetTranslation(robot->link("LLEG_HIP_P")->p() - robot->rootLink()->p());
        llegLink->setName("LLEGBACKSMALL");
        rootLink->appendChild(llegLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateCylinder(0.45/*radius*/, 0.5/*height*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,1.0,0.4)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(-0.45,0,0.15);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.2,0,-0.4);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        llegLink->setShape(group);
      }
      {
        cnoid::LinkPtr rlegLink = new cnoid::Link();
        rlegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rlegLink->setOffsetTranslation(robot->link("RLEG_HIP_P")->p() - robot->rootLink()->p());
        rlegLink->setName("RLEG");
        rootLink->appendChild(rlegLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateCylinder(0.5/*radius*/, 0.6/*height*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,1.0,0.5)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(-0.2,0,-0.1);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.2,0,-0.4);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rlegLink->setShape(group);
      }
      {
        cnoid::LinkPtr rlegLink = new cnoid::Link();
        rlegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        rlegLink->setOffsetTranslation(robot->link("RLEG_HIP_P")->p() - robot->rootLink()->p());
        rlegLink->setName("RLEGSMALL");
        rootLink->appendChild(rlegLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateCylinder(0.45/*radius*/, 0.5/*height*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,1.0,0.4)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(-0.25,0,-0.15);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.2,0,-0.4);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        rlegLink->setShape(group);
      }
      {
        cnoid::LinkPtr llegLink = new cnoid::Link();
        llegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        llegLink->setOffsetTranslation(robot->link("LLEG_HIP_P")->p() - robot->rootLink()->p());
        llegLink->setName("LLEG");
        rootLink->appendChild(llegLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateCylinder(0.5/*radius*/, 0.6/*height*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,1.0,0.5)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(-0.2,0,0.1);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.2,0,-0.4);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
        posTransform->addChild(shape);
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        group->addChild(posTransform);
        llegLink->setShape(group);
      }
      {
        cnoid::LinkPtr llegLink = new cnoid::Link();
        llegLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
        llegLink->setOffsetTranslation(robot->link("LLEG_HIP_P")->p() - robot->rootLink()->p());
        llegLink->setName("LLEGSMALL");
        rootLink->appendChild(llegLink);
        cnoid::SgShapePtr shape;
        {
          cnoid::SgShapePtr shape1 = new cnoid::SgShape();
          shape1->setMesh(meshGenerator.generateCylinder(0.45/*radius*/, 0.5/*height*/));
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1 = choreonoid_qhull::meshToEigen(shape1);
          cnoid::SgShapePtr shape2 = new cnoid::SgShape();
          shape2->setMesh(meshGenerator.generateBox(cnoid::Vector3(1.0,1.0,0.4)));
          cnoid::Position trans = cnoid::Position::Identity(); trans.translation() = cnoid::Vector3(-0.25,0,0.15);
          Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2 = trans * choreonoid_qhull::meshToEigen(shape2);
          Eigen::MatrixXd intersection;
          convex_polyhedron_intersection::intersection(vertices1, vertices2, intersection);
          shape = choreonoid_qhull::generateMeshFromConvexHull(intersection);
        }
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0.8);
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->translation() = cnoid::Vector3(0.2,0,-0.4);
        posTransform->rotation() = cnoid::AngleAxis(-M_PI/2, cnoid::Vector3::UnitX()).toRotationMatrix();
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

    horizontals.clear();
    horizontals.push_back(std::make_pair<cnoid::LinkPtr, cnoid::LinkPtr>(abstractRobot->rootLink(), horizontalRobot->rootLink()));

    endEffectors.clear();
    {
      std::shared_ptr<multicontact_locomotion_planner::EndEffector> endEffector = std::make_shared<multicontact_locomotion_planner::EndEffector>();
      endEffector->name = "rtoe";
      endEffector->parentLink = robot->link("RLEG_ANKLE_R");
      endEffector->localPose.setIdentity();
      endEffector->localPose.translation() = cnoid::Vector3(0.15,0.0,-0.045);
      endEffector->environmentType = multicontact_locomotion_planner::EndEffector::EnvironmentType::SMALLSURFACE;
      endEffector->ikConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
      endEffector->ikConstraint->A_link() = endEffector->parentLink;
      endEffector->ikConstraint->A_localpos() = endEffector->localPose;
      endEffector->ikConstraint->B_link() = nullptr;
      endEffector->ikConstraint->B_localpos() = endEffector->parentLink->T() * endEffector->localPose;
      endEffector->preContactOffset.setIdentity();
      endEffector->preContactOffset.translation()[2] = 0.05;
      endEffector->ignoreLinks.clear();
      endEffector->ignoreLinks.insert(robot->link("RLEG_ANKLE_R"));
      endEffector->ignoreLinks.insert(robot->link("RLEG_ANKLE_P"));
      endEffector->ignoreLinks.insert(robot->link("RLEG_KNEE"));
      endEffector->ignoreBoundingBox.parentLink = endEffector->parentLink;
      endEffector->ignoreBoundingBox.localPose = endEffector->localPose;
      endEffector->ignoreBoundingBox.dimensions = cnoid::Vector3(0.6,0.6,0.6);
      endEffector->preContactAngles.clear();
      endEffector->contactAngles.clear();
      endEffector->limbLinks.clear();
      endEffector->limbLinks.insert(robot->link("RLEG_ANKLE_R"));
      endEffector->limbLinks.insert(robot->link("RLEG_ANKLE_P"));
      endEffector->limbLinks.insert(robot->link("RLEG_KNEE"));
      endEffector->limbLinks.insert(robot->link("RLEG_HIP_Y"));
      endEffector->limbLinks.insert(robot->link("RLEG_HIP_P"));
      endEffector->limbLinks.insert(robot->link("RLEG_HIP_R"));
      endEffector->dl = Eigen::VectorXd::Zero(11); endEffector->C.resize(11,6); endEffector->du = 1e10 * Eigen::VectorXd::Ones(11);
      endEffector->C.insert(0,2) = 1.0; endEffector->du[0] = 2000.0;
      endEffector->C.insert(1,0) = 1.0; endEffector->C.insert(1,2) = 0.2;
      endEffector->C.insert(2,0) = -1.0; endEffector->C.insert(2,2) = 0.2;
      endEffector->C.insert(3,1) = 1.0; endEffector->C.insert(3,2) = 0.2;
      endEffector->C.insert(4,1) = -1.0; endEffector->C.insert(4,2) = 0.2;
      endEffector->C.insert(5,2) = 0.0; endEffector->C.insert(5,3) = 1.0;
      endEffector->C.insert(6,2) = 0.0; endEffector->C.insert(6,3) = -1.0;
      endEffector->C.insert(7,2) = 0.0; endEffector->C.insert(7,4) = 1.0;
      endEffector->C.insert(8,2) = 0.0; endEffector->C.insert(8,4) = -1.0;
      endEffector->C.insert(9,2) = 0.0; endEffector->C.insert(9,5) = 1.0;
      endEffector->C.insert(10,2) = 0.0; endEffector->C.insert(10,5) = -1.0;
      endEffectors[endEffector->name] = endEffector;
    }
    {
      std::shared_ptr<multicontact_locomotion_planner::EndEffector> endEffector = std::make_shared<multicontact_locomotion_planner::EndEffector>();
      endEffector->name = "ltoe";
      endEffector->parentLink = robot->link("LLEG_ANKLE_R");
      endEffector->localPose.setIdentity();
      endEffector->localPose.translation() = cnoid::Vector3(0.15,0.0,-0.045);
      endEffector->environmentType = multicontact_locomotion_planner::EndEffector::EnvironmentType::SMALLSURFACE;
      endEffector->ikConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
      endEffector->ikConstraint->A_link() = endEffector->parentLink;
      endEffector->ikConstraint->A_localpos() = endEffector->localPose;
      endEffector->ikConstraint->B_link() = nullptr;
      endEffector->ikConstraint->B_localpos() = endEffector->parentLink->T() * endEffector->localPose;
      endEffector->preContactOffset.setIdentity();
      endEffector->preContactOffset.translation()[2] = 0.05;
      endEffector->ignoreLinks.clear();
      endEffector->ignoreLinks.insert(robot->link("LLEG_ANKLE_R"));
      endEffector->ignoreLinks.insert(robot->link("LLEG_ANKLE_P"));
      endEffector->ignoreLinks.insert(robot->link("LLEG_KNEE"));
      endEffector->ignoreBoundingBox.parentLink = endEffector->parentLink;
      endEffector->ignoreBoundingBox.localPose = endEffector->localPose;
      endEffector->ignoreBoundingBox.dimensions = cnoid::Vector3(0.6,0.6,0.6);
      endEffector->preContactAngles.clear();
      endEffector->contactAngles.clear();
      endEffector->limbLinks.clear();
      endEffector->limbLinks.insert(robot->link("LLEG_ANKLE_R"));
      endEffector->limbLinks.insert(robot->link("LLEG_ANKLE_P"));
      endEffector->limbLinks.insert(robot->link("LLEG_KNEE"));
      endEffector->limbLinks.insert(robot->link("LLEG_HIP_Y"));
      endEffector->limbLinks.insert(robot->link("LLEG_HIP_P"));
      endEffector->limbLinks.insert(robot->link("LLEG_HIP_R"));
      endEffector->dl = Eigen::VectorXd::Zero(11); endEffector->C.resize(11,6); endEffector->du = 1e10 * Eigen::VectorXd::Ones(11);
      endEffector->C.insert(0,2) = 1.0; endEffector->du[0] = 2000.0;
      endEffector->C.insert(1,0) = 1.0; endEffector->C.insert(1,2) = 0.5;
      endEffector->C.insert(2,0) = -1.0; endEffector->C.insert(2,2) = 0.5;
      endEffector->C.insert(3,1) = 1.0; endEffector->C.insert(3,2) = 0.5;
      endEffector->C.insert(4,1) = -1.0; endEffector->C.insert(4,2) = 0.5;
      endEffector->C.insert(5,2) = 0.0; endEffector->C.insert(5,3) = 1.0;
      endEffector->C.insert(6,2) = 0.0; endEffector->C.insert(6,3) = -1.0;
      endEffector->C.insert(7,2) = 0.0; endEffector->C.insert(7,4) = 1.0;
      endEffector->C.insert(8,2) = 0.0; endEffector->C.insert(8,4) = -1.0;
      endEffector->C.insert(9,2) = 0.0; endEffector->C.insert(9,5) = 1.0;
      endEffector->C.insert(10,2) = 0.0; endEffector->C.insert(10,5) = -1.0;
      endEffectors[endEffector->name] = endEffector;
    }
    {
      std::shared_ptr<multicontact_locomotion_planner::EndEffector> endEffector = std::make_shared<multicontact_locomotion_planner::EndEffector>();
      endEffector->name = "rleg";
      endEffector->parentLink = robot->link("RLEG_ANKLE_R");
      endEffector->localPose.setIdentity();
      endEffector->localPose.translation() = cnoid::Vector3(0.0,0.0,-0.045);
      endEffector->environmentType = multicontact_locomotion_planner::EndEffector::EnvironmentType::LARGESURFACE;
      endEffector->ikConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
      endEffector->ikConstraint->A_link() = endEffector->parentLink;
      endEffector->ikConstraint->A_localpos() = endEffector->localPose;
      endEffector->ikConstraint->B_link() = nullptr;
      endEffector->ikConstraint->B_localpos() = endEffector->parentLink->T() * endEffector->localPose;
      endEffector->preContactOffset.setIdentity();
      endEffector->preContactOffset.translation()[2] = 0.05;
      endEffector->ignoreLinks.clear();
      endEffector->ignoreLinks.insert(robot->link("RLEG_ANKLE_R"));
      endEffector->ignoreLinks.insert(robot->link("RLEG_ANKLE_P"));
      endEffector->ignoreLinks.insert(robot->link("RLEG_KNEE"));
      endEffector->ignoreBoundingBox.parentLink = endEffector->parentLink;
      endEffector->ignoreBoundingBox.localPose = endEffector->localPose;
      endEffector->ignoreBoundingBox.dimensions = cnoid::Vector3(0.4,0.4,0.4);
      endEffector->preContactAngles.clear();
      endEffector->contactAngles.clear();
      endEffector->limbLinks.clear();
      endEffector->limbLinks.insert(robot->link("RLEG_ANKLE_R"));
      endEffector->limbLinks.insert(robot->link("RLEG_ANKLE_P"));
      endEffector->limbLinks.insert(robot->link("RLEG_KNEE"));
      endEffector->limbLinks.insert(robot->link("RLEG_HIP_Y"));
      endEffector->limbLinks.insert(robot->link("RLEG_HIP_P"));
      endEffector->limbLinks.insert(robot->link("RLEG_HIP_R"));
      endEffector->dl = Eigen::VectorXd::Zero(11); endEffector->C.resize(11,6); endEffector->du = 1e10 * Eigen::VectorXd::Ones(11);
      endEffector->C.insert(0,2) = 1.0; endEffector->du[0] = 2000.0;
      endEffector->C.insert(1,0) = 1.0; endEffector->C.insert(1,2) = 0.5;
      endEffector->C.insert(2,0) = -1.0; endEffector->C.insert(2,2) = 0.5;
      endEffector->C.insert(3,1) = 1.0; endEffector->C.insert(3,2) = 0.5;
      endEffector->C.insert(4,1) = -1.0; endEffector->C.insert(4,2) = 0.5;
      endEffector->C.insert(5,2) = 0.05; endEffector->C.insert(5,3) = 1.0;
      endEffector->C.insert(6,2) = 0.05; endEffector->C.insert(6,3) = -1.0;
      endEffector->C.insert(7,2) = 0.12; endEffector->C.insert(7,4) = 1.0;
      endEffector->C.insert(8,2) = 0.09; endEffector->C.insert(8,4) = -1.0;
      endEffector->C.insert(9,2) = 0.005; endEffector->C.insert(9,5) = 1.0;
      endEffector->C.insert(10,2) = 0.005; endEffector->C.insert(10,5) = -1.0;
      endEffectors[endEffector->name] = endEffector;
    }
    {
      std::shared_ptr<multicontact_locomotion_planner::EndEffector> endEffector = std::make_shared<multicontact_locomotion_planner::EndEffector>();
      endEffector->name = "lleg";
      endEffector->parentLink = robot->link("LLEG_ANKLE_R");
      endEffector->localPose.setIdentity();
      endEffector->localPose.translation() = cnoid::Vector3(0.0,0.0,-0.045);
      endEffector->environmentType = multicontact_locomotion_planner::EndEffector::EnvironmentType::LARGESURFACE;
      endEffector->ikConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
      endEffector->ikConstraint->A_link() = endEffector->parentLink;
      endEffector->ikConstraint->A_localpos() = endEffector->localPose;
      endEffector->ikConstraint->B_link() = nullptr;
      endEffector->ikConstraint->B_localpos() = endEffector->parentLink->T() * endEffector->localPose;
      endEffector->preContactOffset.setIdentity();
      endEffector->preContactOffset.translation()[2] = 0.05;
      endEffector->ignoreLinks.clear();
      endEffector->ignoreLinks.insert(robot->link("LLEG_ANKLE_R"));
      endEffector->ignoreLinks.insert(robot->link("LLEG_ANKLE_P"));
      endEffector->ignoreLinks.insert(robot->link("LLEG_KNEE"));
      endEffector->ignoreBoundingBox.parentLink = endEffector->parentLink;
      endEffector->ignoreBoundingBox.localPose = endEffector->localPose;
      endEffector->ignoreBoundingBox.dimensions = cnoid::Vector3(0.4,0.4,0.4);
      endEffector->preContactAngles.clear();
      endEffector->contactAngles.clear();
      endEffector->limbLinks.clear();
      endEffector->limbLinks.insert(robot->link("LLEG_ANKLE_R"));
      endEffector->limbLinks.insert(robot->link("LLEG_ANKLE_P"));
      endEffector->limbLinks.insert(robot->link("LLEG_KNEE"));
      endEffector->limbLinks.insert(robot->link("LLEG_HIP_Y"));
      endEffector->limbLinks.insert(robot->link("LLEG_HIP_P"));
      endEffector->limbLinks.insert(robot->link("LLEG_HIP_R"));
      endEffector->dl = Eigen::VectorXd::Zero(11); endEffector->C.resize(11,6); endEffector->du = 1e10 * Eigen::VectorXd::Ones(11);
      endEffector->C.insert(0,2) = 1.0; endEffector->du[0] = 2000.0;
      endEffector->C.insert(1,0) = 1.0; endEffector->C.insert(1,2) = 0.2;
      endEffector->C.insert(2,0) = -1.0; endEffector->C.insert(2,2) = 0.2;
      endEffector->C.insert(3,1) = 1.0; endEffector->C.insert(3,2) = 0.2;
      endEffector->C.insert(4,1) = -1.0; endEffector->C.insert(4,2) = 0.2;
      endEffector->C.insert(5,2) = 0.05; endEffector->C.insert(5,3) = 1.0;
      endEffector->C.insert(6,2) = 0.05; endEffector->C.insert(6,3) = -1.0;
      endEffector->C.insert(7,2) = 0.12; endEffector->C.insert(7,4) = 1.0;
      endEffector->C.insert(8,2) = 0.09; endEffector->C.insert(8,4) = -1.0;
      endEffector->C.insert(9,2) = 0.005; endEffector->C.insert(9,5) = 1.0;
      endEffector->C.insert(10,2) = 0.005; endEffector->C.insert(10,5) = -1.0;
      endEffectors[endEffector->name] = endEffector;
    }
    {
      std::shared_ptr<multicontact_locomotion_planner::EndEffector> endEffector = std::make_shared<multicontact_locomotion_planner::EndEffector>();
      endEffector->name = "rarm";
      endEffector->parentLink = robot->link("RARM_WRIST_R");
      endEffector->localPose.setIdentity();
      endEffector->localPose.translation() = cnoid::Vector3(0.0,0.0,-0.2);
      endEffector->environmentType = multicontact_locomotion_planner::EndEffector::EnvironmentType::SMALLSURFACE;
      endEffector->ikConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
      endEffector->ikConstraint->A_link() = endEffector->parentLink;
      endEffector->ikConstraint->A_localpos() = endEffector->localPose;
      endEffector->ikConstraint->B_link() = nullptr;
      endEffector->ikConstraint->B_localpos() = endEffector->parentLink->T() * endEffector->localPose;
      endEffector->preContactOffset.setIdentity();
      endEffector->preContactOffset.translation()[2] = 0.05;
      endEffector->ignoreLinks.clear();
      endEffector->ignoreLinks.insert(robot->link("RARM_WRIST_R"));
      endEffector->ignoreLinks.insert(robot->link("RARM_WRIST_P"));
      endEffector->ignoreLinks.insert(robot->link("RARM_WRIST_Y"));
      endEffector->ignoreLinks.insert(robot->link("RARM_ELBOW"));
      endEffector->ignoreBoundingBox.parentLink = endEffector->parentLink;
      endEffector->ignoreBoundingBox.localPose = endEffector->localPose;
      endEffector->ignoreBoundingBox.dimensions = cnoid::Vector3(0.3,0.3,0.3);
      endEffector->preContactAngles.clear();
      endEffector->contactAngles.clear();
      endEffector->limbLinks.clear();
      endEffector->limbLinks.insert(robot->link("RARM_WRIST_R"));
      endEffector->limbLinks.insert(robot->link("RARM_WRIST_P"));
      endEffector->limbLinks.insert(robot->link("RARM_WRIST_Y"));
      endEffector->limbLinks.insert(robot->link("RARM_ELBOW"));
      endEffector->limbLinks.insert(robot->link("RARM_SHOULDER_Y"));
      endEffector->limbLinks.insert(robot->link("RARM_SHOULDER_R"));
      endEffector->limbLinks.insert(robot->link("RARM_SHOULDER_P"));
      endEffector->dl = Eigen::VectorXd::Zero(11); endEffector->C.resize(11,6); endEffector->du = 1e10 * Eigen::VectorXd::Ones(11);
      endEffector->C.insert(0,2) = 1.0; endEffector->du[0] = 2000.0;
      endEffector->C.insert(1,0) = 1.0; endEffector->C.insert(1,2) = 0.2;
      endEffector->C.insert(2,0) = -1.0; endEffector->C.insert(2,2) = 0.2;
      endEffector->C.insert(3,1) = 1.0; endEffector->C.insert(3,2) = 0.2;
      endEffector->C.insert(4,1) = -1.0; endEffector->C.insert(4,2) = 0.2;
      endEffector->C.insert(5,2) = 0.0; endEffector->C.insert(5,3) = 1.0;
      endEffector->C.insert(6,2) = 0.0; endEffector->C.insert(6,3) = -1.0;
      endEffector->C.insert(7,2) = 0.0; endEffector->C.insert(7,4) = 1.0;
      endEffector->C.insert(8,2) = 0.0; endEffector->C.insert(8,4) = -1.0;
      endEffector->C.insert(9,2) = 0.0; endEffector->C.insert(9,5) = 1.0;
      endEffector->C.insert(10,2) = 0.0; endEffector->C.insert(10,5) = -1.0;
      endEffectors[endEffector->name] = endEffector;
    }
    {
      std::shared_ptr<multicontact_locomotion_planner::EndEffector> endEffector = std::make_shared<multicontact_locomotion_planner::EndEffector>();
      endEffector->name = "larm";
      endEffector->parentLink = robot->link("LARM_WRIST_R");
      endEffector->localPose.setIdentity();
      endEffector->localPose.translation() = cnoid::Vector3(0.0,0.0,-0.2);
      endEffector->environmentType = multicontact_locomotion_planner::EndEffector::EnvironmentType::SMALLSURFACE;
      endEffector->ikConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
      endEffector->ikConstraint->A_link() = endEffector->parentLink;
      endEffector->ikConstraint->A_localpos() = endEffector->localPose;
      endEffector->ikConstraint->B_link() = nullptr;
      endEffector->ikConstraint->B_localpos() = endEffector->parentLink->T() * endEffector->localPose;
      endEffector->preContactOffset.setIdentity();
      endEffector->preContactOffset.translation()[2] = 0.05;
      endEffector->ignoreLinks.clear();
      endEffector->ignoreLinks.insert(robot->link("LARM_WRIST_R"));
      endEffector->ignoreLinks.insert(robot->link("LARM_WRIST_P"));
      endEffector->ignoreLinks.insert(robot->link("LARM_WRIST_Y"));
      endEffector->ignoreLinks.insert(robot->link("LARM_ELBOW"));
      endEffector->ignoreBoundingBox.parentLink = endEffector->parentLink;
      endEffector->ignoreBoundingBox.localPose = endEffector->localPose;
      endEffector->ignoreBoundingBox.dimensions = cnoid::Vector3(0.3,0.3,0.3);
      endEffector->preContactAngles.clear();
      endEffector->contactAngles.clear();
      endEffector->limbLinks.clear();
      endEffector->limbLinks.insert(robot->link("LARM_WRIST_R"));
      endEffector->limbLinks.insert(robot->link("LARM_WRIST_P"));
      endEffector->limbLinks.insert(robot->link("LARM_WRIST_Y"));
      endEffector->limbLinks.insert(robot->link("LARM_ELBOW"));
      endEffector->limbLinks.insert(robot->link("LARM_SHOULDER_Y"));
      endEffector->limbLinks.insert(robot->link("LARM_SHOULDER_R"));
      endEffector->limbLinks.insert(robot->link("LARM_SHOULDER_P"));
      endEffector->dl = Eigen::VectorXd::Zero(11); endEffector->C.resize(11,6); endEffector->du = 1e10 * Eigen::VectorXd::Ones(11);
      endEffector->C.insert(0,2) = 1.0; endEffector->du[0] = 2000.0;
      endEffector->C.insert(1,0) = 1.0; endEffector->C.insert(1,2) = 0.2;
      endEffector->C.insert(2,0) = -1.0; endEffector->C.insert(2,2) = 0.2;
      endEffector->C.insert(3,1) = 1.0; endEffector->C.insert(3,2) = 0.2;
      endEffector->C.insert(4,1) = -1.0; endEffector->C.insert(4,2) = 0.2;
      endEffector->C.insert(5,2) = 0.0; endEffector->C.insert(5,3) = 1.0;
      endEffector->C.insert(6,2) = 0.0; endEffector->C.insert(6,3) = -1.0;
      endEffector->C.insert(7,2) = 0.0; endEffector->C.insert(7,4) = 1.0;
      endEffector->C.insert(8,2) = 0.0; endEffector->C.insert(8,4) = -1.0;
      endEffector->C.insert(9,2) = 0.0; endEffector->C.insert(9,5) = 1.0;
      endEffector->C.insert(10,2) = 0.0; endEffector->C.insert(10,5) = -1.0;
      endEffectors[endEffector->name] = endEffector;
    }
    {
      std::shared_ptr<multicontact_locomotion_planner::EndEffector> endEffector = std::make_shared<multicontact_locomotion_planner::EndEffector>();
      endEffector->name = "rhand";
      endEffector->parentLink = robot->link("RARM_WRIST_R");
      endEffector->localPose.setIdentity();
      endEffector->localPose.translation() = cnoid::Vector3(0.0,0.0,-0.2);
      endEffector->environmentType = multicontact_locomotion_planner::EndEffector::EnvironmentType::GRASP;
      endEffector->ikConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
      endEffector->ikConstraint->A_link() = endEffector->parentLink;
      endEffector->ikConstraint->A_localpos() = endEffector->localPose;
      endEffector->ikConstraint->B_link() = nullptr;
      endEffector->ikConstraint->B_localpos() = endEffector->parentLink->T() * endEffector->localPose;
      endEffector->preContactOffset.setIdentity();
      endEffector->preContactOffset.translation()[2] = 0.1;
      endEffector->ignoreLinks.clear();
      endEffector->ignoreLinks.insert(robot->link("RARM_WRIST_R"));
      endEffector->ignoreLinks.insert(robot->link("RARM_WRIST_P"));
      endEffector->ignoreLinks.insert(robot->link("RARM_WRIST_Y"));
      endEffector->ignoreLinks.insert(robot->link("RARM_ELBOW"));
      endEffector->ignoreBoundingBox.parentLink = endEffector->parentLink;
      endEffector->ignoreBoundingBox.localPose = endEffector->localPose;
      endEffector->ignoreBoundingBox.dimensions = cnoid::Vector3(0.3,0.3,0.3);
      endEffector->preContactAngles.clear();
      endEffector->contactAngles.clear();
      endEffector->limbLinks.clear();
      endEffector->limbLinks.insert(robot->link("RARM_WRIST_R"));
      endEffector->limbLinks.insert(robot->link("RARM_WRIST_P"));
      endEffector->limbLinks.insert(robot->link("RARM_WRIST_Y"));
      endEffector->limbLinks.insert(robot->link("RARM_ELBOW"));
      endEffector->limbLinks.insert(robot->link("RARM_SHOULDER_Y"));
      endEffector->limbLinks.insert(robot->link("RARM_SHOULDER_R"));
      endEffector->limbLinks.insert(robot->link("RARM_SHOULDER_P"));
      endEffector->dl.resize(6); endEffector->C.resize(6,6); endEffector->du.resize(6);
      endEffector->dl[0] = -500.0; endEffector->C.insert(0,0) = 1.0; endEffector->du[0] = 500.0;
      endEffector->dl[1] = -500.0; endEffector->C.insert(1,1) = 1.0; endEffector->du[1] = 500.0;
      endEffector->dl[2] = -500.0; endEffector->C.insert(2,2) = 1.0; endEffector->du[2] = 500.0;
      endEffector->dl[3] = -10.0; endEffector->C.insert(3,3) = 1.0; endEffector->du[3] = 10.0;
      endEffector->dl[4] = -10.0; endEffector->C.insert(4,4) = 1.0; endEffector->du[4] = 10.0;
      endEffector->dl[5] = -10.0; endEffector->C.insert(5,5) = 1.0; endEffector->du[5] = 10.0;
      endEffectors[endEffector->name] = endEffector;
    }
    {
      std::shared_ptr<multicontact_locomotion_planner::EndEffector> endEffector = std::make_shared<multicontact_locomotion_planner::EndEffector>();
      endEffector->name = "lhand";
      endEffector->parentLink = robot->link("LARM_WRIST_R");
      endEffector->localPose.setIdentity();
      endEffector->localPose.translation() = cnoid::Vector3(0.0,0.0,-0.2);
      endEffector->environmentType = multicontact_locomotion_planner::EndEffector::EnvironmentType::GRASP;
      endEffector->ikConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
      endEffector->ikConstraint->A_link() = endEffector->parentLink;
      endEffector->ikConstraint->A_localpos() = endEffector->localPose;
      endEffector->ikConstraint->B_link() = nullptr;
      endEffector->ikConstraint->B_localpos() = endEffector->parentLink->T() * endEffector->localPose;
      endEffector->preContactOffset.setIdentity();
      endEffector->preContactOffset.translation()[2] = 0.1;
      endEffector->ignoreLinks.clear();
      endEffector->ignoreLinks.insert(robot->link("LARM_WRIST_R"));
      endEffector->ignoreLinks.insert(robot->link("LARM_WRIST_P"));
      endEffector->ignoreLinks.insert(robot->link("LARM_WRIST_Y"));
      endEffector->ignoreLinks.insert(robot->link("LARM_ELBOW"));
      endEffector->ignoreBoundingBox.parentLink = endEffector->parentLink;
      endEffector->ignoreBoundingBox.localPose = endEffector->localPose;
      endEffector->ignoreBoundingBox.dimensions = cnoid::Vector3(0.3,0.3,0.3);
      endEffector->preContactAngles.clear();
      endEffector->contactAngles.clear();
      endEffector->limbLinks.clear();
      endEffector->limbLinks.insert(robot->link("LARM_WRIST_R"));
      endEffector->limbLinks.insert(robot->link("LARM_WRIST_P"));
      endEffector->limbLinks.insert(robot->link("LARM_WRIST_Y"));
      endEffector->limbLinks.insert(robot->link("LARM_ELBOW"));
      endEffector->limbLinks.insert(robot->link("LARM_SHOULDER_Y"));
      endEffector->limbLinks.insert(robot->link("LARM_SHOULDER_R"));
      endEffector->limbLinks.insert(robot->link("LARM_SHOULDER_P"));
      endEffector->dl.resize(6); endEffector->C.resize(6,6); endEffector->du.resize(6);
      endEffector->dl[0] = -500.0; endEffector->C.insert(0,0) = 1.0; endEffector->du[0] = 500.0;
      endEffector->dl[1] = -500.0; endEffector->C.insert(1,1) = 1.0; endEffector->du[1] = 500.0;
      endEffector->dl[2] = -500.0; endEffector->C.insert(2,2) = 1.0; endEffector->du[2] = 500.0;
      endEffector->dl[3] = -10.0; endEffector->C.insert(3,3) = 1.0; endEffector->du[3] = 10.0;
      endEffector->dl[4] = -10.0; endEffector->C.insert(4,4) = 1.0; endEffector->du[4] = 10.0;
      endEffector->dl[5] = -10.0; endEffector->C.insert(5,5) = 1.0; endEffector->du[5] = 10.0;
      endEffectors[endEffector->name] = endEffector;
    }

    modes.clear();
    {
      std::shared_ptr<multicontact_locomotion_planner::Mode> mode = std::make_shared<multicontact_locomotion_planner::Mode>();
      mode->name = "biped";
      mode->score = 10.0;
      {
        mode->eefs.push_back("rleg");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEGSTAND");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEGSTANDSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("lleg");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEGSTAND");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEGSTANDSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        // root collision
        std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
        constraint->A_link() = abstractRobot->rootLink();
        constraint->field() = field;
        constraint->tolerance() = 0.06; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
        constraint->precision() = 0.05; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする
        constraint->ignoreDistance() = 0.5; // rbrttは大きく動くので、ignoreも大きくする必要がある
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        mode->rootConstraints.push_back(constraint);
      }
      modes[mode->name]=mode;
    }


    {
      std::shared_ptr<multicontact_locomotion_planner::Mode> mode = std::make_shared<multicontact_locomotion_planner::Mode>();
      mode->name = "quadruped";
      mode->score = 5.0;
      {
        mode->eefs.push_back("rtoe");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEGBACK");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEGBACKSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("rarm");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("RARMUPPER");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("RARMUPPERSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("ltoe");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEGBACK");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEGBACKSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("larm");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("LARMUPPER");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("LARMUPPERSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        // root collision
        std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
        constraint->A_link() = abstractRobot->rootLink();
        constraint->field() = field;
        constraint->tolerance() = 0.06; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
        constraint->precision() = 0.05; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする
        constraint->ignoreDistance() = 0.5; // rbrttは大きく動くので、ignoreも大きくする必要がある
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        mode->rootConstraints.push_back(constraint);
      }
      modes[mode->name]=mode;
    }

    {
      std::shared_ptr<multicontact_locomotion_planner::Mode> mode = std::make_shared<multicontact_locomotion_planner::Mode>();
      mode->name = "quadruped_large";
      mode->score = 9.0;
      {
        mode->eefs.push_back("rleg");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEGBACK");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEGBACKSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("rarm");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("RARMUPPER");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("RARMUPPERSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("lleg");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEGBACK");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEGBACKSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("larm");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("LARMUPPER");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("LARMUPPERSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        // root collision
        std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
        constraint->A_link() = abstractRobot->rootLink();
        constraint->field() = field;
        constraint->tolerance() = 0.06; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
        constraint->precision() = 0.05; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする
        constraint->ignoreDistance() = 0.5; // rbrttは大きく動くので、ignoreも大きくする必要がある
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        mode->rootConstraints.push_back(constraint);
      }
      modes[mode->name]=mode;
    }


    {
      std::shared_ptr<multicontact_locomotion_planner::Mode> mode = std::make_shared<multicontact_locomotion_planner::Mode>();
      mode->name = "grasp";
      mode->score = 7.0;
      {
        mode->eefs.push_back("rtoe");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEG");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEGSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("rhand");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("RARM");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("RARMSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("ltoe");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEG");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEGSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("lhand");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("LARM");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("LARMSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        // root collision
        std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
        constraint->A_link() = abstractRobot->rootLink();
        constraint->field() = field;
        constraint->tolerance() = 0.06; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
        constraint->precision() = 0.05; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする
        constraint->ignoreDistance() = 0.5; // rbrttは大きく動くので、ignoreも大きくする必要がある
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        mode->rootConstraints.push_back(constraint);
      }
      modes[mode->name]=mode;
    }

    {
      std::shared_ptr<multicontact_locomotion_planner::Mode> mode = std::make_shared<multicontact_locomotion_planner::Mode>();
      mode->name = "grasp_large";
      mode->score = 8.0;
      {
        mode->eefs.push_back("rleg");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEG");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("RLEGSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("rhand");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("RARM");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("RARMSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("lleg");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEG");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = horizontalRobot->link("LLEGSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        mode->eefs.push_back("lhand");
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("LARM");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsLarge.push_back(constraint);
        }
        {
          std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> constraint = std::make_shared<ik_constraint2_bullet::BulletKeepCollisionConstraint>();
          constraint->A_link() = abstractRobot->link("LARMSMALL");
          constraint->B_link() = constraint->A_link(); // dummy
          constraint->precision() = 0.01; // rbrrtの結果が微妙に誤差があるので、0.01では小さすぎる
          constraint->A_FACE_C().resize(1); constraint->A_FACE_dl().resize(1); constraint->A_FACE_du().resize(1);
          choreonoid_cddlib::convertToFACEExpression(constraint->A_link()->collisionShape(),
                                                     constraint->A_FACE_C()[0],
                                                     constraint->A_FACE_dl()[0],
                                                     constraint->A_FACE_du()[0]);
          constraint->B_FACE_C() = constraint->A_FACE_C();
          constraint->B_FACE_dl() = constraint->A_FACE_dl();
          constraint->B_FACE_du() = constraint->A_FACE_du();
          constraint->debugLevel() = 0;
          constraint->updateBounds(); // キャッシュを内部に作る.
          mode->reachabilityConstraintsSmall.push_back(constraint);
        }
      }
      {
        // root collision
        std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
        constraint->A_link() = abstractRobot->rootLink();
        constraint->field() = field;
        constraint->tolerance() = 0.06; // ちょうど干渉すると法線ベクトルが変になることがあるので, 1回のiterationで動きうる距離よりも大きくせよ.
        constraint->precision() = 0.05; // 角で不正確になりがちなので, toleranceを大きくしてprecisionも大きくして、best effort的にする. precisionはdistanceFieldのサイズの倍数より大きくする
        constraint->ignoreDistance() = 0.5; // rbrttは大きく動くので、ignoreも大きくする必要がある
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        mode->rootConstraints.push_back(constraint);
      }
      modes[mode->name]=mode;
    }


    robotIKInfo = std::make_shared<multicontact_locomotion_planner::RobotIKInfo>();
    robotIKInfo->robot = robot;
    {
      // task: joint limit
      for(int i=0;i<robot->numJoints();i++){
        std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
        constraint->joint() = robot->joint(i);
        robotIKInfo->selfConstraints.push_back(constraint);
      }

      // task: self collision
      std::vector<std::string> rarm{"RARM_SHOULDER_R", "RARM_ELBOW", "RARM_WRIST_R"};
      std::vector<std::string> larm{"LARM_SHOULDER_R", "LARM_ELBOW", "LARM_WRIST_R"};
      std::vector<std::string> rleg{"RLEG_HIP_Y", "RLEG_KNEE", "RLEG_ANKLE_R"};
      std::vector<std::string> lleg{"LLEG_HIP_Y", "LLEG_KNEE", "LLEG_ANKLE_R"};
      std::vector<std::string> torso{"WAIST", "WAIST_R", "CHEST"};

      std::vector<std::vector<std::string> > pairs;
      for(int i=0;i<rarm.size();i++) for(int j=0;j<larm.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],larm[j]});
      for(int i=0;i<rarm.size();i++) for(int j=0;j<rleg.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],rleg[j]});
      for(int i=0;i<rarm.size();i++) for(int j=0;j<lleg.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],lleg[j]});
      for(int i=0;i<rarm.size();i++) for(int j=0;j<torso.size();j++) pairs.push_back(std::vector<std::string>{rarm[i],torso[j]});
      for(int i=0;i<larm.size();i++) for(int j=0;j<rleg.size();j++) pairs.push_back(std::vector<std::string>{larm[i],rleg[j]});
      for(int i=0;i<larm.size();i++) for(int j=0;j<lleg.size();j++) pairs.push_back(std::vector<std::string>{larm[i],lleg[j]});
      for(int i=0;i<larm.size();i++) for(int j=0;j<torso.size();j++) pairs.push_back(std::vector<std::string>{larm[i],torso[j]});
      for(int i=0;i<rleg.size();i++) for(int j=0;j<lleg.size();j++) pairs.push_back(std::vector<std::string>{rleg[i],lleg[j]});

      for(int i=0;i<pairs.size();i++){
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = robot->link(pairs[i][0]);
        constraint->B_link() = robot->link(pairs[i][1]);
        constraint->tolerance() = 0.01;
        robotIKInfo->selfConstraints.push_back(constraint);
      }
    }

    {
      // task: env collision
      for(int i=0;i<robot->numLinks();i++){
        std::shared_ptr<ik_constraint2_distance_field::DistanceFieldCollisionConstraint> constraint = std::make_shared<ik_constraint2_distance_field::DistanceFieldCollisionConstraint>();
        constraint->A_link() = robot->link(i);
        constraint->field() = field;
        constraint->tolerance() = 0.05; // fieldのresolutionが分解能. 一回のiterationで動く最大量よりも大きいと安全. 0.03だと小さくてためにめりこむ?
        constraint->updateBounds(); // キャッシュを内部に作る. キャッシュを作ったあと、10スレッドぶんコピーする方が速い
        robotIKInfo->envConstraints.push_back(constraint);
      }
    }

    {
      // task: nominal constairnt

      for(int i=0;i<robot->numJoints();i++){
        std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
        constraint->joint() = robot->joint(i);
        constraint->targetq() = reset_manip_pose[i];
        constraint->precision() = 1e10; // always satisfied
        robotIKInfo->nominalConstraints.push_back(constraint);
      }
    }

  }


};
