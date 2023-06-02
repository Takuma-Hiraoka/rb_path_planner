#include "world_common.h"

#include <cnoid/MeshGenerator>
#include <choreonoid_bullet/choreonoid_bullet.h>
#include <choreonoid_qhull/choreonoid_qhull.h>

namespace multicontact_locomotion_planner_sample{
  void generateStepWorld(cnoid::BodyPtr& obstacle, // for visual
                         std::shared_ptr<multicontact_locomotion_planner::Environment>& environment
                         ){
    cnoid::MeshGenerator meshGenerator;
    obstacle = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(4,2,0.1)));
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
          posTransform->translation() = cnoid::Vector3(1.5,0,0.35);
          posTransform->addChild(shape);
          group->addChild(posTransform);
        }
        rootLink->setShape(group);
      }
      obstacle->setRootLink(rootLink);
    }

    environment = std::make_shared<multicontact_locomotion_planner::Environment>();
    {
      // collision world
      environment->obstacles = std::make_shared<distance_field::PropagationDistanceField>(5,//size_x
                                                                                          5,//size_y
                                                                                          5,//size_z
                                                                                          0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
                                                                                          -2.5,//origin_x
                                                                                          -2.5,//origin_y
                                                                                          -2.5,//origin_z
                                                                                          0.5, // max_distance
                                                                                          true// propagate_negative_distances
                                                                                          );
      EigenSTL::vector_Vector3d vertices;
      for(int i=0;i<obstacle->numLinks();i++){
        std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.01);
        for(int j=0;j<vertices_.size();j++){
          vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
        }
      }
      environment->obstacles->addPointsToField(vertices);
    }

    {
      // Support polygon
      // 厚みをもたせて3D形状とせずに、2Dpolygonのままで扱ったほうが、IKの制約がコンパクトになるので速い
      cnoid::SgGroupPtr largeSurfaceGroup = new cnoid::SgGroup();
      environment->largeSurfacesBody->rootLink()->setShape(largeSurfaceGroup);
      environment->largeSurfacesBody->setRootLink(environment->largeSurfacesBody->rootLink());
      cnoid::SgGroupPtr smallSurfaceGroup = new cnoid::SgGroup();
      environment->smallSurfacesBody->rootLink()->setShape(smallSurfaceGroup);
      environment->smallSurfacesBody->setRootLink(environment->smallSurfacesBody->rootLink());
      cnoid::SgGroupPtr graspGroup = new cnoid::SgGroup();
      environment->graspsBody->rootLink()->setShape(graspGroup);
      environment->graspsBody->setRootLink(environment->graspsBody->rootLink());
      {
        multicontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(0.0,0.0,0.0);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+1.9,+0.9,0.0);
        region.shape.col(1) = cnoid::Vector3(-1.9,+0.9,0.0);
        region.shape.col(2) = cnoid::Vector3(-1.9,-0.9,0.0);
        region.shape.col(3) = cnoid::Vector3(+1.9,-0.9,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        region.weightR = cnoid::Vector3(1,1,0);
        environment->largeSurfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        largeSurfaceGroup->addChild(posTransform);

      }
      {
        multicontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(0.0,0.0,0.0);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+1.98,+0.98,0.0);
        region.shape.col(1) = cnoid::Vector3(-1.98,+0.98,0.0);
        region.shape.col(2) = cnoid::Vector3(-1.98,-0.98,0.0);
        region.shape.col(3) = cnoid::Vector3(+1.98,-0.98,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        region.weightR = cnoid::Vector3(1,1,0);
        environment->smallSurfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(0.0,0.0,1.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        smallSurfaceGroup->addChild(posTransform);

      }
      {
        multicontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(1.5,0.0,0.4);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+0.4,+0.4,0.0);
        region.shape.col(1) = cnoid::Vector3(-0.4,+0.4,0.0);
        region.shape.col(2) = cnoid::Vector3(-0.4,-0.4,0.0);
        region.shape.col(3) = cnoid::Vector3(+0.4,-0.4,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        region.weightR = cnoid::Vector3(1,1,0);
        environment->largeSurfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        largeSurfaceGroup->addChild(posTransform);

      }
      {
        multicontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(1.5,0.0,0.4);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+0.48,+0.48,0.0);
        region.shape.col(1) = cnoid::Vector3(-0.48,+0.48,0.0);
        region.shape.col(2) = cnoid::Vector3(-0.48,-0.48,0.0);
        region.shape.col(3) = cnoid::Vector3(+0.48,-0.48,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        region.weightR = cnoid::Vector3(1,1,0);
        environment->smallSurfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(0.0,0.0,1.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        smallSurfaceGroup->addChild(posTransform);

      }

      // bulletModelの原点の位置が違うので, 上とはべつで再度変換する必要がある
      environment->largeSurfacesBulletModel = choreonoid_bullet::convertToBulletModels(environment->largeSurfacesBody->rootLink()->collisionShape());
      environment->smallSurfacesBulletModel = choreonoid_bullet::convertToBulletModels(environment->smallSurfacesBody->rootLink()->collisionShape());
      environment->graspsBulletModel = choreonoid_bullet::convertToBulletModels(environment->graspsBody->rootLink()->collisionShape());
    }

  }


  void generateLadderWorld(cnoid::BodyPtr& obstacle, // for visual
                           std::shared_ptr<multicontact_locomotion_planner::Environment>& environment
                           ){
    cnoid::MeshGenerator meshGenerator;
    obstacle = new cnoid::Body();
    {
      cnoid::LinkPtr rootLink = new cnoid::Link();
      {
        cnoid::SgGroupPtr group = new cnoid::SgGroup();
        {
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(4,2,0.1)));
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
          posTransform->translation() = cnoid::Vector3(1.5,0,1.8-0.05);
          posTransform->addChild(shape);
          group->addChild(posTransform);
        }
        for(int i=1;i<7;i++){
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.05,1,0.05)));
          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->translation() = cnoid::Vector3(0.9,0,-0.025+i*0.3);
          posTransform->addChild(shape);
          group->addChild(posTransform);
        }
        rootLink->setShape(group);
      }
      obstacle->setRootLink(rootLink);
    }

    environment = std::make_shared<multicontact_locomotion_planner::Environment>();
    {
      // collision world
      environment->obstacles = std::make_shared<distance_field::PropagationDistanceField>(5,//size_x
                                                                                          5,//size_y
                                                                                          5,//size_z
                                                                                          0.02,//resolution // constratintのtoleranceよりも小さい必要がある.
                                                                                          -2.5,//origin_x
                                                                                          -2.5,//origin_y
                                                                                          -2.5,//origin_z
                                                                                          0.5, // max_distance
                                                                                          false// propagate_negative_distances
                                                                                          );
      EigenSTL::vector_Vector3d vertices;
      for(int i=0;i<obstacle->numLinks();i++){
        std::vector<Eigen::Vector3f> vertices_ = ik_constraint2_distance_field::getSurfaceVertices(obstacle->link(i), 0.01);
        for(int j=0;j<vertices_.size();j++){
          vertices.push_back(obstacle->link(i)->T() * vertices_[j].cast<double>());
        }
      }
      environment->obstacles->addPointsToField(vertices);
    }

    {
      // Support polygon
      // 厚みをもたせて3D形状とせずに、2Dpolygonのままで扱ったほうが、IKの制約がコンパクトになるので速い
      cnoid::SgGroupPtr largeSurfaceGroup = new cnoid::SgGroup();
      environment->largeSurfacesBody->rootLink()->setShape(largeSurfaceGroup);
      environment->largeSurfacesBody->setRootLink(environment->largeSurfacesBody->rootLink());
      cnoid::SgGroupPtr smallSurfaceGroup = new cnoid::SgGroup();
      environment->smallSurfacesBody->rootLink()->setShape(smallSurfaceGroup);
      environment->smallSurfacesBody->setRootLink(environment->smallSurfacesBody->rootLink());
      cnoid::SgGroupPtr graspGroup = new cnoid::SgGroup();
      environment->graspsBody->rootLink()->setShape(graspGroup);
      environment->graspsBody->setRootLink(environment->graspsBody->rootLink());
      {
        multicontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(0.0,0.0,0.0);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+1.9,+0.9,0.0);
        region.shape.col(1) = cnoid::Vector3(-1.9,+0.9,0.0);
        region.shape.col(2) = cnoid::Vector3(-1.9,-0.9,0.0);
        region.shape.col(3) = cnoid::Vector3(+1.9,-0.9,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        region.weightR = cnoid::Vector3(1,1,0);
        environment->largeSurfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        largeSurfaceGroup->addChild(posTransform);
      }
      {
        multicontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(0.0,0.0,0.0);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+1.98,+0.98,0.0);
        region.shape.col(1) = cnoid::Vector3(-1.98,+0.98,0.0);
        region.shape.col(2) = cnoid::Vector3(-1.98,-0.98,0.0);
        region.shape.col(3) = cnoid::Vector3(+1.98,-0.98,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        region.weightR = cnoid::Vector3(1,1,0);
        environment->smallSurfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(0.0,0.0,1.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        smallSurfaceGroup->addChild(posTransform);
      }
      {
        multicontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(1.5,0.0,1.8);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+0.4,+0.4,0.0);
        region.shape.col(1) = cnoid::Vector3(-0.4,+0.4,0.0);
        region.shape.col(2) = cnoid::Vector3(-0.4,-0.4,0.0);
        region.shape.col(3) = cnoid::Vector3(+0.4,-0.4,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        region.weightR = cnoid::Vector3(1,1,0);
        environment->largeSurfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        largeSurfaceGroup->addChild(posTransform);
      }
      {
        multicontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(1.5,0.0,1.8);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+0.48,+0.48,0.0);
        region.shape.col(1) = cnoid::Vector3(-0.48,+0.48,0.0);
        region.shape.col(2) = cnoid::Vector3(-0.48,-0.48,0.0);
        region.shape.col(3) = cnoid::Vector3(+0.48,-0.48,0.0);

        cnoid::SgShapePtr shape = new cnoid::SgShape();
        cnoid::MeshGenerator::Extrusion extrusion;
        for(int i=0;i<region.shape.cols();i++){
          extrusion.crossSection.push_back(region.shape.col(i).head<2>());
        }
        extrusion.spine.push_back(cnoid::Vector3(0,0,-0.005));
        extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        extrusion.scale.push_back(cnoid::Vector2(1,1));
        shape->setMesh(meshGenerator.generateExtrusion(extrusion));

        region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
        region.weightR = cnoid::Vector3(1,1,0);
        environment->smallSurfaces.push_back(region);

        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(0.0,0.0,1.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        smallSurfaceGroup->addChild(posTransform);
      }
      for(int i=1;i<7;i++){
        {
          multicontact_locomotion_planner::ContactableRegion region;
          region.pose.translation() = cnoid::Vector3(0.8,0.0,i*0.3);
          region.shape.resize(3,4);
          region.shape.col(0) = cnoid::Vector3(+0.005,+0.48,0.0);
          region.shape.col(1) = cnoid::Vector3(-0.005,+0.48,0.0);
          region.shape.col(2) = cnoid::Vector3(-0.005,-0.48,0.0);
          region.shape.col(3) = cnoid::Vector3(+0.005,-0.48,0.0);

          cnoid::SgShapePtr shape = new cnoid::SgShape();
          cnoid::MeshGenerator::Extrusion extrusion;
          for(int i=0;i<region.shape.cols();i++){
            extrusion.crossSection.push_back(region.shape.col(i).head<2>());
          }
          extrusion.spine.push_back(cnoid::Vector3(0,0,-0.001));
          extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          shape->setMesh(meshGenerator.generateExtrusion(extrusion));

          region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
          region.weightR = cnoid::Vector3(1,1,0);
          environment->smallSurfaces.push_back(region);

          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.0,0.0,1.0));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->T() = region.pose;
          posTransform->addChild(shape);
          smallSurfaceGroup->addChild(posTransform);
        }
        {
          multicontact_locomotion_planner::ContactableRegion region;
          region.pose.translation() = cnoid::Vector3(0.9,0.0,-0.025 + i*0.3);
          region.pose.linear() = (cnoid::Matrix3() <<
                                       0, 0, -1,
                                       1, 0, 0,
                                       0, -1, 0).finished();
          region.shape.resize(3,4);
          region.shape.col(0) = cnoid::Vector3(+0.48,+0.005,0.0);
          region.shape.col(1) = cnoid::Vector3(-0.48,+0.005,0.0);
          region.shape.col(2) = cnoid::Vector3(-0.48,-0.005,0.0);
          region.shape.col(3) = cnoid::Vector3(+0.48,-0.005,0.0);

          cnoid::SgShapePtr shape = new cnoid::SgShape();
          cnoid::MeshGenerator::Extrusion extrusion;
          for(int i=0;i<region.shape.cols();i++){
            extrusion.crossSection.push_back(region.shape.col(i).head<2>());
          }
          extrusion.spine.push_back(cnoid::Vector3(0,0,-0.001));
          extrusion.spine.push_back(cnoid::Vector3(0,0,0.0));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.orientation.push_back(cnoid::AngleAxis(0, cnoid::Vector3::UnitX()));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          extrusion.scale.push_back(cnoid::Vector2(1,1));
          shape->setMesh(meshGenerator.generateExtrusion(extrusion));

          region.bulletModel = choreonoid_bullet::convertToBulletModel(shape);
          region.weightR = cnoid::Vector3(0,1,1);
          environment->grasps.push_back(region);

          cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
          material->setTransparency(0);
          material->setDiffuseColor(cnoid::Vector3f(0.0,1.0,0.0));
          shape->setMaterial(material);
          cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
          posTransform->T() = region.pose;
          posTransform->addChild(shape);
          smallSurfaceGroup->addChild(posTransform);
          graspGroup->addChild(posTransform);

          region.pose.linear() *= -1;
          environment->grasps.push_back(region); // X軸反転
        }
      }

      // bulletModelの原点の位置が違うので, 上とはべつで再度変換する必要がある
      environment->largeSurfacesBulletModel = choreonoid_bullet::convertToBulletModels(environment->largeSurfacesBody->rootLink()->collisionShape());
      environment->smallSurfacesBulletModel = choreonoid_bullet::convertToBulletModels(environment->smallSurfacesBody->rootLink()->collisionShape());
      environment->graspsBulletModel = choreonoid_bullet::convertToBulletModels(environment->graspsBody->rootLink()->collisionShape());
    }


  }

};
