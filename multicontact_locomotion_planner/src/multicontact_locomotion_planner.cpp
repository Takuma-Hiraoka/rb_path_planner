#include <multicontact_locomotion_planner/multicontact_locomotion_planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <cnoid/MeshGenerator>
#include <convex_polyhedron_intersection/convex_polyhedron_intersection.h>
#include <choreonoid_qhull/choreonoid_qhull.h>

namespace multicontact_locomotion_planner{

  bool RobotIKInfo::solveFullbodyIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                                    const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                                    const std::unordered_map<std::string, std::shared_ptr<Contact> >& nearContacts,
                                    const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& targetConstraints,
                                    std::shared_ptr<std::vector<std::vector<double> > >& path
                                    ){

    // 自己干渉、関節上下限
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0 = this->selfConstraints;

    // 環境干渉
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
    for(int i=0;i<this->envConstraints.size();i++){
      this->envConstraints[i]->ignoreBoundingBox().clear();
      for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it = currentContacts.begin();it!=currentContacts.end();it++){
        if(it->second->ignoreLinks.find(this->envConstraints[i]->A_link()) != it->second->ignoreLinks.end()){
          this->envConstraints[i]->ignoreBoundingBox().push_back(it->second->ignoreBoundingBox);
        }
      }
      for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it = nearContacts.begin();it!=nearContacts.end();it++){
        if(it->second->ignoreLinks.find(this->envConstraints[i]->A_link()) != it->second->ignoreLinks.end()){
          this->envConstraints[i]->ignoreBoundingBox().push_back(it->second->ignoreBoundingBox);
        }
      }
      constraints1.push_back(this->envConstraints[i]);
    }

    // 重心実行可能領域, 現在の接触位置
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2;
    for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it = currentContacts.begin();it!=currentContacts.end();it++){
      it->second->ikConstraint->A_link() = it->second->link1;
      it->second->ikConstraint->A_localpos() = it->second->localPose1;
      it->second->ikConstraint->B_link() = it->second->link2;
      it->second->ikConstraint->B_localpos() = it->second->localPose2;
      constraints2.push_back(it->second->ikConstraint);
    }

    // 重心実行可能領域TODO

    // target task
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints3 = targetConstraints;

    // nominal task
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints4 = this->nominalConstraints;

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1,constraints2,constraints3,constraints4};
    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                      constraints,
                                                                      this->tasks,
                                                                      this->pikParam);

    for ( int i=0; i<constraints0.size(); i++ ) {
      std::cerr << "constraints0: "<< constraints0[i]->isSatisfied() << std::endl;
    }
    for ( int i=0; i<constraints1.size(); i++ ) {
      std::cerr << "constraints1: "<< constraints1[i]->isSatisfied() << std::endl;
    }
    for ( int i=0; i<constraints2.size(); i++ ) {
      std::cerr << "constraints2: "<< constraints2[i]->isSatisfied() << std::endl;
    }
    for ( int i=0; i<constraints3.size(); i++ ) {
      constraints3[i]->debugLevel() = 2;
      constraints3[i]->updateBounds();
      std::cerr << "constraints3: "<< constraints3[i]->isSatisfied() << std::endl;
    }
    for ( int i=0; i<constraints4.size(); i++ ) {
      std::cerr << "constraints4: "<< constraints4[i]->isSatisfied() << std::endl;
    }

    return solved;
  }

  bool RobotIKInfo::solveGlobalIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                                  const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                                  const std::unordered_map<std::string, std::shared_ptr<Contact> >& nearContacts,
                                  const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& targetConstraints,
                                  const cnoid::LinkPtr& projectLink,
                                  const cnoid::Position& projectLocalPose,
                                  std::shared_ptr<std::vector<std::vector<double> > >& path
                                  ){

    // 自己干渉、関節上下限
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0 = this->selfConstraints;

    // 環境干渉
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
    for(int i=0;i<this->envConstraints.size();i++){
      this->envConstraints[i]->ignoreBoundingBox().clear();
      for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it = currentContacts.begin();it!=currentContacts.end();it++){
        if(it->second->ignoreLinks.find(this->envConstraints[i]->A_link()) != it->second->ignoreLinks.end()){
          this->envConstraints[i]->ignoreBoundingBox().push_back(it->second->ignoreBoundingBox);
        }
      }
      for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it = nearContacts.begin();it!=nearContacts.end();it++){
        if(it->second->ignoreLinks.find(this->envConstraints[i]->A_link()) != it->second->ignoreLinks.end()){
          this->envConstraints[i]->ignoreBoundingBox().push_back(it->second->ignoreBoundingBox);
        }
      }
      constraints1.push_back(this->envConstraints[i]);
    }

    // 重心実行可能領域
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2;//TODO

    // target task
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goals = targetConstraints;

    // nominal task
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals = this->nominalConstraints;

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1,constraints2};

    this->gikParam.projectLink.resize(1);
    this->gikParam.projectLink[0] = projectLink;
    this->gikParam.projectLocalPose = projectLocalPose;

    bool solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                             constraints,
                                                             goals,
                                                             nominals,
                                                             this->gikParam,
                                                             path);
    return solved;
  }


  bool solveMLP(const cnoid::BodyPtr currentRobot,
                const std::shared_ptr<Environment>& environment,
                const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                const std::string& prevSwingEEF,
                const std::vector<std::pair<std::vector<double>, std::string> >& targetRootPath, // angle, mode
                const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                std::vector<RobotState>& outputPath, // variablesの次元に対応

                const MLPParam& param){

    if(param.debugLevel >= 3){
      if(param.viewer){
        param.viewer->drawObjects();
      }
      std::cerr << "start MSL. Press ENTER:" << std::endl;
      getchar();
    }

    std::vector<double> currentAngle;
    link2Frame(variables, currentAngle);

    // 今のroot位置から、target root pathのsubgoal点を見つける.
    int subGoalIdx = -1;
    bool subGoalFound = false;
    for(int i=targetRootPath.size()-1;i>=0;i--){
      cnoid::Vector3 targetp(targetRootPath[i].first[0],targetRootPath[i].first[1],targetRootPath[i].first[2]);
      cnoid::Quaternion targetR(targetRootPath[i].first[6],
                                targetRootPath[i].first[3],
                                targetRootPath[i].first[4],
                                targetRootPath[i].first[5]);
      double dist = std::sqrt((currentRobot->rootLink()->p()-targetp).squaredNorm() + std::pow(cnoid::AngleAxis(currentRobot->rootLink()->R().transpose()*targetR).angle(),2));
      if(dist <= param.subGoalDistanceFar) {
        subGoalIdx = i;
        subGoalFound = true;
        break;
      }
    }
    if(!subGoalFound){
      std::cerr << "[" << __FUNCTION__ << "] subGoal is not found" << std::endl;
      return false;
    }
    if(param.debugLevel>=2){
      std::cerr << "subGoalFar: " << subGoalIdx << " [" << targetRootPath[subGoalIdx].first[0] << " " << targetRootPath[subGoalIdx].first[1] << " " << targetRootPath[subGoalIdx].first[2] << " " << targetRootPath[subGoalIdx].first[3] << " " << targetRootPath[subGoalIdx].first[4] << " " << targetRootPath[subGoalIdx].first[5] << " " << targetRootPath[subGoalIdx].first[6] << "]" << std::endl;
    }

    // 今のままsubgoalに到達しないか見る. するならrootを動かして終わり. (angle-vectorが出てくる.)
    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
    {
      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = param.robot->rootLink();
      constraint->B_link() = nullptr;
      constraint->B_localpos().translation()[0] = targetRootPath[subGoalIdx].first[0];
      constraint->B_localpos().translation()[1] = targetRootPath[subGoalIdx].first[1];
      constraint->B_localpos().translation()[2] = targetRootPath[subGoalIdx].first[2];
      constraint->B_localpos().linear() = cnoid::Quaternion(targetRootPath[subGoalIdx].first[6],
                                                            targetRootPath[subGoalIdx].first[3],
                                                            targetRootPath[subGoalIdx].first[4],
                                                            targetRootPath[subGoalIdx].first[5]).toRotationMatrix();
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > targetConstraints{constraint};
      bool solved = param.robotIKInfo->solveFullbodyIK(variables,
                                                       currentContacts,
                                                       std::unordered_map<std::string, std::shared_ptr<Contact> >(),
                                                       targetConstraints,
                                                       path);
      if(solved){
        outputPath.resize(path->size());
        for(int i=0;i<path->size();i++){
          outputPath[i].jointAngle = path->at(i);
          outputPath[i].contacts = currentContacts;
        }
        return true;
      }
    }

    if(param.debugLevel >= 3){
      if(param.viewer){
        param.viewer->drawObjects();
      }
      std::cerr << "check subGoal. Press ENTER:" << std::endl;
      getchar();
    }

    // subGoalに可能な限り近づけたところで、再度今のroot位置から、target root pathのsubgoal点を見つける. (subGoalが前回のiterationよりも前に進んでいることが期待される)
    subGoalIdx = -1;
    subGoalFound = false;
    for(int i=targetRootPath.size()-1;i>=0;i--){
      cnoid::Vector3 targetp(targetRootPath[i].first[0],targetRootPath[i].first[1],targetRootPath[i].first[2]);
      cnoid::Quaternion targetR(targetRootPath[i].first[6],
                                targetRootPath[i].first[3],
                                targetRootPath[i].first[4],
                                targetRootPath[i].first[5]);
      double dist = std::sqrt((currentRobot->rootLink()->p()-targetp).squaredNorm() + std::pow(cnoid::AngleAxis(currentRobot->rootLink()->R().transpose()*targetR).angle(),2));
      if(dist <= param.subGoalDistanceNear) {
        subGoalIdx = i;
        subGoalFound = true;
        break;
      }
    }
    if(!subGoalFound){
      std::cerr << "[" << __FUNCTION__ << "] new subGoal is not found" << std::endl;
      frame2Link(currentAngle, variables);
      param.robot->calcForwardKinematics(false);
      multicontact_locomotion_planner::calcAssoc(param.assocs);
      param.abstractRobot->calcForwardKinematics(false);
      multicontact_locomotion_planner::calcHorizontal(param.horizontals);
      param.horizontalRobot->calcForwardKinematics(false);
      return false;
    }

    if(param.debugLevel>=2){
      std::cerr << "subGoalNear: " << subGoalIdx << " [" << targetRootPath[subGoalIdx].first[0] << " " << targetRootPath[subGoalIdx].first[1] << " " << targetRootPath[subGoalIdx].first[2] << " " << targetRootPath[subGoalIdx].first[3] << " " << targetRootPath[subGoalIdx].first[4] << " " << targetRootPath[subGoalIdx].first[5] << " " << targetRootPath[subGoalIdx].first[6] << "]" << std::endl;
    }

    std::shared_ptr<Mode> targetMode = param.modes.find(targetRootPath[subGoalIdx].second)->second;
    frame2Link(targetRootPath[subGoalIdx].first, std::vector<cnoid::LinkPtr>{param.abstractRobot->rootLink()});
    param.robot->calcForwardKinematics(false);
    multicontact_locomotion_planner::calcAssoc(param.assocs);
    param.abstractRobot->calcForwardKinematics(false);
    multicontact_locomotion_planner::calcHorizontal(param.horizontals);
    param.horizontalRobot->calcForwardKinematics(false);

    // 次にswingするeefを決める
    //   まず、targetModeにあって、currentContactsに無いEEFを探して、makeさせる. このとき、同一limbに別のcurrentContactがあるなら、それをbreakする.
    std::unordered_set<std::string> unBreakableContacts; // TODO

    bool swingEEFFound = false;
    std::shared_ptr<Contact> breakContact = nullptr;
    std::unordered_map<std::string, std::shared_ptr<Contact> > swingContacts = currentContacts; // 遊脚期のcontact
    std::shared_ptr<EndEffector> targetEEF = nullptr; // 次に接触させるEEF
    std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> targetReachabilityConstraint;
    for(int i=0;i<targetMode->eefs.size();i++){
      if(currentContacts.find(targetMode->eefs[i]) == currentContacts.end()){
        targetEEF = param.endEffectors.find(targetMode->eefs[i])->second;
        targetReachabilityConstraint = targetMode->reachabilityConstraints[i];
        for(std::unordered_map<std::string, std::shared_ptr<Contact> >::iterator it=swingContacts.begin(); it!=swingContacts.end();){
          if(targetEEF->limbLinks.find(it->second->link1) != targetEEF->limbLinks.end() ||
             targetEEF->limbLinks.find(it->second->link2) != targetEEF->limbLinks.end()){
            breakContact = it->second;
            it = swingContacts.erase(it);
          }else{
            it++;
          }
        }
        swingEEFFound = true;
        break;
      }
    }
    //   次に、targetModeに無くて、currentContactsにあるEEFを探して、breakさせる
    if(!swingEEFFound){
      for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it=currentContacts.begin(); it!=currentContacts.end();it++){
        bool found = false;
        for(int i=0;i<targetMode->eefs.size();i++){
          if(it->first == targetMode->eefs[i]) {
            found = true;
            break;
          }
        }
        if(!found){
          breakContact = it->second;
          swingContacts.erase(it->first);
          swingEEFFound = true;
          break;
        }
      }
    }
    //   最後に、prevSwingEEFの次のEEFをswingする
    if(!swingEEFFound){

      int nextEEFIdxInit=0;
      for(int i=0;i<targetMode->eefs.size();i++){
        if(prevSwingEEF == targetMode->eefs[i]) {
          nextEEFIdxInit = (i+1==targetMode->eefs.size()) ? 0 : i+1;
        }
      }

      for(int i=0;i<targetMode->eefs.size();i++){
        int nextEEFIdx = nextEEFIdxInit + i;
        if(nextEEFIdx >= targetMode->eefs.size()) nextEEFIdx -= targetMode->eefs.size();

        // 候補点に今の接触位置が含まれているなら、別のlimbへ
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> reachabilityConstraint = targetMode->reachabilityConstraints[nextEEFIdx];
        {
          cnoid::MeshGenerator meshGenerator;
          cnoid::LinkPtr tmpLink = new cnoid::Link();
          tmpLink->setJointType(cnoid::Link::JointType::FIXED_JOINT);
          cnoid::SgShapePtr shape = new cnoid::SgShape();
          shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.01,0.01,0.01)));
          cnoid::SgGroupPtr group = new cnoid::SgGroup();
          group->addChild(shape);
          tmpLink->setShape(group);
          tmpLink->T() = param.endEffectors.find(targetMode->eefs[nextEEFIdx])->second->parentLink->T() * param.endEffectors.find(targetMode->eefs[nextEEFIdx])->second->localPose;

          reachabilityConstraint->B_link() = tmpLink;
        }
        reachabilityConstraint->updateBounds();
        if(!reachabilityConstraint->isSatisfied()){
          continue;
        }

        targetEEF = param.endEffectors.find(targetMode->eefs[nextEEFIdx])->second;
        targetReachabilityConstraint = targetMode->reachabilityConstraints[i];
        breakContact = currentContacts.find(targetMode->eefs[nextEEFIdx])->second;
        swingContacts.erase(targetMode->eefs[nextEEFIdx]);
        swingEEFFound = true;
        break;
      }
    }

    if(!swingEEFFound){
      std::cerr << "[" << __FUNCTION__ << "] swing eef is not found" << std::endl;
      frame2Link(currentAngle, variables);
      param.robot->calcForwardKinematics(false);
      multicontact_locomotion_planner::calcAssoc(param.assocs);
      param.abstractRobot->calcForwardKinematics(false);
      multicontact_locomotion_planner::calcHorizontal(param.horizontals);
      param.horizontalRobot->calcForwardKinematics(false);

      return false;
    }

    frame2Link(currentAngle, variables);
    param.robot->calcForwardKinematics(false);
    multicontact_locomotion_planner::calcAssoc(param.assocs);
    param.abstractRobot->calcForwardKinematics(false);
    multicontact_locomotion_planner::calcHorizontal(param.horizontals);
    param.horizontalRobot->calcForwardKinematics(false);

    // breakするなら、
    std::shared_ptr<std::vector<std::vector<double> > > breakPath1 = std::make_shared<std::vector<std::vector<double> > >();
    std::shared_ptr<std::vector<std::vector<double> > > breakPath2 = std::make_shared<std::vector<std::vector<double> > >();
    std::unordered_map<std::string, std::shared_ptr<Contact> > currentContactsAfterBreak = currentContacts;
    if(breakContact){
      {
        if(param.debugLevel>=2){
          std::cerr << "breakContact: " << breakContact->name << std::endl;
        }

        // contact breakability check
        // TODO
        std::shared_ptr<ik_constraint2::COMConstraint> constraint = std::make_shared<ik_constraint2::COMConstraint>();
        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > targetConstraints{};
        bool solved = param.robotIKInfo->solveFullbodyIK(variables,
                                                         currentContacts,
                                                         std::unordered_map<std::string, std::shared_ptr<Contact> >(),
                                                         targetConstraints,
                                                         breakPath1);
        if(!solved){
          std::cerr << "[" << __FUNCTION__ << "] breakPath is not found" << std::endl;
          frame2Link(currentAngle, variables);
          param.robot->calcForwardKinematics(false);
          multicontact_locomotion_planner::calcAssoc(param.assocs);
          param.abstractRobot->calcForwardKinematics(false);
          multicontact_locomotion_planner::calcHorizontal(param.horizontals);
          param.horizontalRobot->calcForwardKinematics(false);

          return false;
        }

        if(param.debugLevel >= 3){
          if(param.viewer){
            param.viewer->drawObjects();
          }
          std::cerr << "break contact. Press ENTER:" << std::endl;
          getchar();
        }

      }

      currentContactsAfterBreak.erase(breakContact->name);

      {
        // pre contact pose
        std::vector<cnoid::LinkPtr> variables_near = variables;
        for(int i=0;i<breakContact->preContactAngles.size();i++){
          breakContact->preContactAngles[i].first->q() = breakContact->preContactAngles[i].second;

          std::vector<cnoid::LinkPtr>::iterator it = std::remove(variables_near.begin(), variables_near.end(), breakContact->preContactAngles[i].first);
          variables_near.erase(it, variables_near.end());
        }
        param.robot->calcForwardKinematics();

        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = breakContact->link1;
        constraint->A_localpos() = breakContact->localPose1;
        constraint->B_link() = breakContact->link2;
        constraint->B_localpos() = breakContact->localPose2 * breakContact->preContactOffset;

        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > targetConstraints{constraint};
        std::unordered_map<std::string, std::shared_ptr<Contact> > nearContacts;
        nearContacts[breakContact->name] = breakContact;
        bool solved = param.robotIKInfo->solveFullbodyIK(variables,
                                                         currentContactsAfterBreak,
                                                         nearContacts,
                                                         targetConstraints,
                                                         breakPath2);

        if(!solved){
          std::cerr << "[" << __FUNCTION__ << "] post contact Path is not found" << std::endl;
          frame2Link(currentAngle, variables);
          param.robot->calcForwardKinematics(false);
          multicontact_locomotion_planner::calcAssoc(param.assocs);
          param.abstractRobot->calcForwardKinematics(false);
          multicontact_locomotion_planner::calcHorizontal(param.horizontals);
          param.horizontalRobot->calcForwardKinematics(false);

          return false;
        }


        if(param.debugLevel >= 3){
          if(param.viewer){
            param.viewer->drawObjects();
          }
          std::cerr << "post contact. Press ENTER:" << std::endl;
          getchar();
        }

      }
    }

    // makeするなら、subGoalのroot位置のreachability内で環境接触候補点を絞り、そのどこかに接触させる軌道を生成する. (goalのnominal root poseを与える)
    std::shared_ptr<std::vector<std::vector<double> > > makePath1 = std::make_shared<std::vector<std::vector<double> > >();
    std::unordered_map<std::string, std::shared_ptr<Contact> > currentContactsAfterMake = currentContactsAfterBreak;
    if(targetEEF){
      if(param.debugLevel>=2){
        std::cerr << "makeContact: " << targetEEF->name << std::endl;
      }

      // subGoalのroot位置のreachability内で環境接触候補点を絞る
      Eigen::Matrix<double, 3, Eigen::Dynamic> region = choreonoid_qhull::meshToEigen(targetReachabilityConstraint->A_link()->collisionShape());
      const std::vector<ContactableRegion>& candidate =
        (targetEEF->environmentType==EndEffector::EnvironmentType::LARGESURFACE) ? environment->largeSurfaces :
        (targetEEF->environmentType==EndEffector::EnvironmentType::SMALLSURFACE) ? environment->smallSurfaces :
        environment->grasps;
      std::vector<ContactableRegion> targetRegion;
      for(int i=0;i<candidate.size();i++){
        Eigen::Matrix<double, 3, Eigen::Dynamic> shape = candidate[i].pose * candidate[i].shape;
        Eigen::MatrixXd intersection;
        bool solved = convex_polyhedron_intersection::intersection(shape, region, intersection);
        if(solved && intersection.cols() > 0){
          Eigen::Matrix<double, 3, Eigen::Dynamic> intersectionLocal = candidate[i].pose.inverse() * Eigen::Matrix<double, 3, Eigen::Dynamic>(intersection);
          targetRegion.push_back(candidate[i]);
          targetRegion.back().pose = targetRegion.back().pose * targetEEF->preContactOffset; // offsetだけずらす
          targetRegion.back().shape = intersectionLocal;
        }
      }

      if(!solveSwingTrajectory(variables,
                               swingContacts,
                               targetRootPath[subGoalIdx].first,
                               targetEEF,
                               makePath1
                               )){
        std::cerr << "[" << __FUNCTION__ << "] makePath is not found" << std::endl;
        frame2Link(currentAngle, variables);
        param.robot->calcForwardKinematics(false);
        multicontact_locomotion_planner::calcAssoc(param.assocs);
        param.abstractRobot->calcForwardKinematics(false);
        multicontact_locomotion_planner::calcHorizontal(param.horizontals);
        param.horizontalRobot->calcForwardKinematics(false);

        return false;
      }

      if(param.debugLevel >= 3){
        if(param.viewer){
          param.viewer->drawObjects();
        }
        std::cerr << "swing. Press ENTER:" << std::endl;
        getchar();
      }

    }


    // break->make指令およびangle-vectorが出てくる. (stateが別れている.)


    return true;
  }

  bool solveSwingTrajectory(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                            const std::unordered_map<std::string, std::shared_ptr<Contact> >& swingContacts,
                            const std::vector<double>& subGoal,
                            const std::shared_ptr<EndEffector>& targetEEF,
                            std::shared_ptr<std::vector<std::vector<double> > >& path
                            ){
    return true;
  }


};
