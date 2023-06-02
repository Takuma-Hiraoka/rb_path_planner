#ifndef MULTICONTACT_LOCOMOTION_PLANNER_ROBOTSTATE_H
#define MULTICONTACT_LOCOMOTION_PLANNER_ROBOTSTATE_H

#include <unordered_map>
#include <vector>
#include <string>
#include <unordered_set>
#include <Eigen/Eigen>
#include <ik_constraint2_distance_field/ik_constraint2_distance_field.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <cnoid/Body>
#include <ik_constraint2/ik_constraint2.h>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>
#include <ik_constraint2_bullet/ik_constraint2_bullet.h>
#include <rb_rrt_solver/rb_rrt_solver.h>

namespace multicontact_locomotion_planner{

  class Contact {
  public:
    std::string name; // エンドエフェクタの名前の場合、そのエンドエフェクタがworldに接触していることを意味し、効率的に扱える

    cnoid::LinkPtr link1 = nullptr; // nullptrならworld
    cnoid::Position localPose1 = cnoid::Position::Identity();
    cnoid::LinkPtr link2 = nullptr; // nullptrならworld
    cnoid::Position localPose2 = cnoid::Position::Identity();

    std::shared_ptr<ik_constraint2::PositionConstraint> ikConstraint = std::make_shared<ik_constraint2::PositionConstraint>(); // A_localposeを変えないで、B_localposeを変更する

    Eigen::SparseMatrix<double,Eigen::RowMajor> C{0,6}; // localPose1 frame/origin. link1がlink2から受ける力に関する接触力制約. 列は6. C, ld, udの行数は同じ.
    cnoid::VectorX dl;
    cnoid::VectorX du;

    cnoid::Position preContactOffset = cnoid::Position::Identity(); // 接触の前後で、実際に接触する位置姿勢から、localpose1座標系でこの値だけoffsetした位置に、localpose1がまず移動する. ここから接触までの間は、直線的に移動しcollisionを許容する.
    // ignore boundingbox
    std::unordered_set<cnoid::LinkPtr> ignoreLinks; // 干渉を許容しうるリンク
    ik_constraint2_distance_field::DistanceFieldCollisionConstraint::BoundingBox ignoreBoundingBox; // ignoreLinksのignoreBoundingBox内にaある部分は、preContact~contactまでの間はobstacleとの干渉を許容する.
    std::vector<std::pair<cnoid::LinkPtr, double> > preContactAngles; // preContactから接触までの間のangle

  };

  class EndEffector {
  public:
    std::string name;

    cnoid::LinkPtr parentLink;
    cnoid::Position localPose;
    enum class EnvironmentType {LARGESURFACE, SMALLSURFACE, GRASP } environmentType = EnvironmentType::LARGESURFACE;
    Eigen::SparseMatrix<double,Eigen::RowMajor> C{0,6}; // localPose frame/origin. endeffectorが受ける力に関する接触力制約. 列は6. C, ld, udの行数は同じ.
    cnoid::VectorX dl;
    cnoid::VectorX du;

    std::shared_ptr<ik_constraint2::PositionConstraint> ikConstraint = std::make_shared<ik_constraint2::PositionConstraint>(); // A_linkがこのEEF

    cnoid::Position preContactOffset = cnoid::Position::Identity(); // 接触の前後で、実際に接触する位置姿勢から、エンドエフェクタ座標系でこの値だけoffsetした位置に、まず移動する. ここから接触までの間は、直線的に移動しcollisionを許容する.
    // ignore boundingbox
    std::unordered_set<cnoid::LinkPtr> ignoreLinks; // 干渉を許容しうるリンク
    ik_constraint2_distance_field::DistanceFieldCollisionConstraint::BoundingBox ignoreBoundingBox; // ignoreLinksのignoreBoundingBox内にaある部分は、preContact~contactまでの間はobstacleとの干渉を許容する.

    std::vector<std::pair<cnoid::LinkPtr, double> > preContactAngles; // preContactから接触までの間のangle
    std::vector<std::pair<cnoid::LinkPtr, double> > contactAngles; // 接触してからのangle

    std::unordered_set<cnoid::LinkPtr> limbLinks; // 同一limbにあるlink. これらのどれかが接触している場合は、このEEFを接触させるまえにまずそれをbreakさせる必要がある.

  public:
    std::shared_ptr<Contact> generateContact();
  };



  class RobotState {
  public:
    std::vector<double> jointAngle; // freejointはx y z qx qy qz qwの順
    std::unordered_map<std::string, std::shared_ptr<Contact> > contacts;
  };

  class ContactableRegion {
  public:
    cnoid::Position pose = cnoid::Position::Identity();
    Eigen::Matrix<double, 3, Eigen::Dynamic> shape; // pose frame. 3xX [v1, v2, v3 ...] の凸形状
    std::shared_ptr<btConvexShape> bulletModel;
    cnoid::Vector3 weightR = cnoid::Vector3::Ones(); // pose frame
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
    cnoid::BodyPtr largeSurfacesBody = new cnoid::Body();
    std::vector<std::shared_ptr<btConvexShape> > largeSurfacesBulletModel; // rootLinkに対応
    std::vector<ContactableRegion> smallSurfaces;
    cnoid::BodyPtr smallSurfacesBody = new cnoid::Body();
    std::vector<std::shared_ptr<btConvexShape> > smallSurfacesBulletModel; // rootLinkに対応
    std::vector<ContactableRegion> grasps;
    cnoid::BodyPtr graspsBody = new cnoid::Body();
    std::vector<std::shared_ptr<btConvexShape> > graspsBulletModel; // rootLinkに対応

  };

  class Mode {
  public:
    std::string name;
    double score = 1.0; // 大きい方を好む

    std::vector<std::string> eefs;
    std::vector<std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> > reachabilityConstraintsSmall; // サイズと順番はeefsと同じ. A_linkがreachability. B_linkはplannerがセットする.
    std::vector<std::shared_ptr<ik_constraint2_bullet::BulletKeepCollisionConstraint> > reachabilityConstraintsLarge; // サイズと順番はeefsと同じ. A_linkがreachability. B_linkはplannerがセットする.

    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rootConstraints; // root obstacle collision.

  public:
    std::shared_ptr<ik_constraint2::IKConstraint> generateCondition(const std::unordered_map<std::string, std::shared_ptr<EndEffector> >& endEffectors, const std::shared_ptr<Environment>& environment);

    bool isContactSatisfied(const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                            bool isLarge,
                            std::vector<std::string>& moveEEF,
                            std::vector<std::string>& newEEF,
                            std::vector<std::string>& excessContact);
  };
};

#endif
