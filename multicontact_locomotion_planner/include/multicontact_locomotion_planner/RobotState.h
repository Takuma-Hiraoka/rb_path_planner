#ifndef MULTICONTACT_LOCOMOTION_PLANNER_ROBOTSTATE_H
#define MULTICONTACT_LOCOMOTION_PLANNER_ROBOTSTATE_H

#include <unordered_map>
#include <vector>
#include <string>
#include <unordered_set>
#include <Eigen/Eigen>
#include <moveit/distance_field/propagation_distance_field.h>
#include <cnoid/Body>

namespace multicontact_locomotion_planner{

  class EndEffector {
  public:
    std::string name;

    cnoid::LinkPtr parentLink;
    cnoid::Position localpose;
    enum class EnvironmentType {LARGESURFACE, SMALLSURFACE, GRASP } environmemtType = EnvironmentType::LARGESURFACE;
    Eigen::SparseMatrix<double,Eigen::RowMajor> C{0,6}; // localPose frame/origin. endeffectorが受ける力に関する接触力制約. 列は6. C, ld, udの行数は同じ.
    cnoid::VectorX ld;
    cnoid::VectorX ud;

    cnoid::Vector3 preContactOffset = cnoid::Vector3::Zero(); // 接触の前後で、実際に接触する位置姿勢から、エンドエフェクタ座標系でこの値だけoffsetした位置に、まず移動する. ここから接触までの間は、直線的に移動しcollisionを許容する.
    // ignore boundingbox
    std::vector<cnoid::LinkPtr> ignoreLinks; // 干渉を許容しうるリンク
    cnoid::Position ignoreBoundingBoxLocalPose = cnoid::Position::Identity(); // endeffector frame.
    cnoid::Vector3 ignoreBoundingBoxDimensions = cnoid::Vector3::Zero(); // ignoreLinksのignoreBoundingBox内にaある部分は、preContact~contactまでの間はobstacleとの干渉を許容する.
    std::vector<std::pair<cnoid::LinkPtr, double> > preContactAngles; // preContactから接触までの間のangle
    std::vector<std::pair<cnoid::LinkPtr, double> > contactAngles; // 接触してからのangle

    std::unordered_set<cnoid::LinkPtr> limbLinks; // 同一limbにあるlink. これらのどれかが接触している場合は、このEEFを接触させるまえにまずそれをbreakさせる必要がある.
  };

  class Contact {
  public:
    std::string name; // エンドエフェクタの名前の場合、そのエンドエフェクタがworldに接触していることを意味し、効率的に扱える

    cnoid::LinkPtr link1 = nullptr; // nullptrならworld
    cnoid::Position localPose1 = cnoid::Position::Identity();
    cnoid::LinkPtr link2 = nullptr; // nullptrならworld
    cnoid::Position localPose2 = cnoid::Position::Identity();

    Eigen::SparseMatrix<double,Eigen::RowMajor> C{0,6}; // localPose1 frame/origin. link1がlink2から受ける力に関する接触力制約. 列は6. C, ld, udの行数は同じ.
    cnoid::VectorX ld;
    cnoid::VectorX ud;
  };

  class RobotState {
  public:
    std::vector<double> jointAngle; // freejointはx y z qx qy qz qwの順
    std::unordered_map<std::string, std::shared_ptr<Contact> > contacts;
  };

  class ContactableRegion {
    cnoid::Position pose;
    std::vector<cnoid::Vector3> shape; // pose frame.
    enum class Type { SURFACE, GRASP } type = Type::SURFACE;
    /*
      2D surface polygonの場合
          shapeのZ座標は0. Z座標+の方向が法線方向
      graspable lineの場合
          shapeのYZ座標は0. Z座標+の方向からapproachする.
     */
  };

  class Environment {
  public:
    std::shared_ptr<distance_field::PropagationDistanceField> obstacles = std::make_shared<distance_field::PropagationDistanceField>(5,//size_x
                                                                                                                                     5,//size_y
                                                                                                                                     5,//size_z
                                                                                                                                     0.04,//resolution
                                                                                                                                     -2.5,//origin_x
                                                                                                                                     -2.5,//origin_y
                                                                                                                                     -2.5,//origin_z
                                                                                                                                     0.5, // max_distance
                                                                                                                                     false// propagate_negative_distances
                                                                                                                                     );
    std::vector<ContactableRegion> largeSurfaces;
    std::vector<ContactableRegion> smallSurfaces;
    std::vector<ContactableRegion> grasps;
  };

  class Mode {
  public:
    std::string name;

    std::vector<std::string> eefs;
  };
};

#endif
