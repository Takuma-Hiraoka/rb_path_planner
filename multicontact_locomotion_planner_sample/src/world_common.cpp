#include "world_common.h"

#include <cnoid/MeshGenerator>

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
      environment->obstacles->addPointsToField(vertices);
    }

    {
      // Support polygon
      // 厚みをもたせて3D形状とせずに、2Dpolygonのままで扱ったほうが、IKの制約がコンパクトになるので速い
      cnoid::SgGroupPtr group = new cnoid::SgGroup();
      environment->largeSurfacesBody->rootLink()->setShape(group);
      environment->largeSurfacesBody->setRootLink(environment->largeSurfacesBody->rootLink());
      {
        multicontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(0.0,0.0,0.0);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+1.9,+0.9,0.0);
        region.shape.col(1) = cnoid::Vector3(-1.9,+0.9,0.0);
        region.shape.col(2) = cnoid::Vector3(-1.9,-0.9,0.0);
        region.shape.col(3) = cnoid::Vector3(+1.9,-0.9,0.0);
        region.weightR = cnoid::Vector3(1,1,0);
        environment->largeSurfaces.push_back(region);

        cnoid::LinkPtr link = new cnoid::Link();
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
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        group->addChild(posTransform);
      }
      {
        multicontact_locomotion_planner::ContactableRegion region;
        region.pose.translation() = cnoid::Vector3(1.5,0.0,0.4);
        region.shape.resize(3,4);
        region.shape.col(0) = cnoid::Vector3(+0.4,+0.4,0.0);
        region.shape.col(1) = cnoid::Vector3(-0.4,+0.4,0.0);
        region.shape.col(2) = cnoid::Vector3(-0.4,-0.4,0.0);
        region.shape.col(3) = cnoid::Vector3(+0.4,-0.4,0.0);
        region.weightR = cnoid::Vector3(1,1,0);
        environment->largeSurfaces.push_back(region);

        cnoid::LinkPtr link = new cnoid::Link();
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
        cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
        material->setTransparency(0);
        material->setDiffuseColor(cnoid::Vector3f(1.0,0.0,0.0));
        shape->setMaterial(material);
        cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
        posTransform->T() = region.pose;
        posTransform->addChild(shape);
        group->addChild(posTransform);
      }
    }

  }

};
