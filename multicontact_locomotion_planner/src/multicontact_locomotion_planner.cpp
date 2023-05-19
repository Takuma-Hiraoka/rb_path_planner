#include <multicontact_locomotion_planner/multicontact_locomotion_planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <cnoid/MeshGenerator>
#include <convex_polyhedron_intersection/convex_polyhedron_intersection.h>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <ik_constraint2_region_cdd/ik_constraint2_region_cdd.h>
#include <scfr_solver/scfr_solver.h>

namespace multicontact_locomotion_planner{

  bool RobotIKInfo::solveFullbodyIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                                    const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                                    const std::unordered_map<std::string, std::shared_ptr<Contact> >& nearContacts,
                                    const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& targetConstraints,
                                    const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& bestEffortConstraints,
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
    this->updateCOMConstraint(currentContacts, this->comConstraint);
    constraints2.push_back(this->comConstraint);

    // target task
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints3 = targetConstraints;

    // nominal task
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints4 = this->nominalConstraints;
    std::copy(bestEffortConstraints.begin(), bestEffortConstraints.end(), std::back_inserter(constraints4));

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1,constraints2,constraints3,constraints4};
    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                      constraints,
                                                                      this->tasks,
                                                                      this->pikParam,
                                                                      path);

    // for ( int i=0; i<constraints0.size(); i++ ) {
    //   std::cerr << "constraints0: "<< constraints0[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints1.size(); i++ ) {
    //   std::cerr << "constraints1: "<< constraints1[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints2.size(); i++ ) {
    //   std::cerr << "constraints2: "<< constraints2[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints3.size(); i++ ) {
    //   constraints3[i]->debugLevel() = 2;
    //   constraints3[i]->updateBounds();
    //   std::cerr << "constraints3: "<< constraints3[i]->isSatisfied() << std::endl;
    // }
    // for ( int i=0; i<constraints4.size(); i++ ) {
    //   std::cerr << "constraints4: "<< constraints4[i]->isSatisfied() << std::endl;
    // }

    return solved;
  }

  bool RobotIKInfo::solveGlobalIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                                  const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                                  const std::unordered_map<std::string, std::shared_ptr<Contact> >& nearContacts,
                                  const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& targetConstraints,
                                  const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& bestEffortConstraints,
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

    // 重心実行可能領域, 現在の接触位置
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2;
    for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it = currentContacts.begin();it!=currentContacts.end();it++){
      it->second->ikConstraint->A_link() = it->second->link1;
      it->second->ikConstraint->A_localpos() = it->second->localPose1;
      it->second->ikConstraint->B_link() = it->second->link2;
      it->second->ikConstraint->B_localpos() = it->second->localPose2;
      constraints2.push_back(it->second->ikConstraint);
    }
    this->updateCOMConstraint(currentContacts, this->comConstraint);
    constraints2.push_back(this->comConstraint);

    // target task
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goals = targetConstraints;

    // nominal task
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals = this->nominalConstraints;
    std::copy(bestEffortConstraints.begin(), bestEffortConstraints.end(), std::back_inserter(nominals));

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1,constraints2};

    this->gikParam.projectLink.resize(1);
    this->gikParam.projectLink[0] = projectLink;
    this->gikParam.projectLocalPose = projectLocalPose;

    // 関節角度上下限を厳密に満たしていないと、omplのstart stateがエラーになるので
    for(int i=0;i<variables.size();i++){
      if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()) {
        variables[i]->q() = std::max(std::min(variables[i]->q(),variables[i]->q_upper()),variables[i]->q_lower());
      }
    }

    bool solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                             constraints,
                                                             goals,
                                                             nominals,
                                                             this->gikParam,
                                                             path);

    return solved;
  }

  void RobotIKInfo::updateCOMConstraint(const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts, std::shared_ptr<ik_constraint2::COMConstraint>& constraint){
    std::vector<cnoid::Position> poses;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
    std::vector<cnoid::VectorX> bs;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
    std::vector<cnoid::VectorX> dls;
    std::vector<cnoid::VectorX> dus;
    for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it=currentContacts.begin();it!=currentContacts.end();it++){
      if(((it->second->link1!=nullptr) && (it->second->link1->body() == this->robot)) &&
         !((it->second->link2!=nullptr) && (it->second->link2->body() == this->robot))){
        poses.push_back(it->second->link1->T() * it->second->localPose1);
        As.emplace_back(0,6);
        bs.emplace_back(0);
        Cs.push_back(it->second->C);
        dls.push_back(it->second->dl);
        dus.push_back(it->second->du);
      } else if(!((it->second->link1!=nullptr) && (it->second->link1->body() == this->robot)) &&
                ((it->second->link2!=nullptr) && (it->second->link2->body() == this->robot))){
        poses.push_back(it->second->link1 ? (it->second->link1->T() * it->second->localPose1) : it->second->localPose1);
        As.emplace_back(0,6);
        bs.emplace_back(0);
        Cs.push_back(-it->second->C);
        dls.push_back(it->second->dl);
        dus.push_back(it->second->du);
      }
    }
    Eigen::SparseMatrix<double,Eigen::RowMajor> M(0,2);
    Eigen::VectorXd l;
    Eigen::VectorXd u;
    std::vector<Eigen::Vector2d> vertices;
    scfr_solver::calcSCFR(poses,
                          As,
                          bs,
                          Cs,
                          dls,
                          dus,
                          this->robot->mass(),
                          M,
                          l,
                          u,
                          vertices
                          );

    Eigen::SparseMatrix<double,Eigen::ColMajor> C(M.rows(),3);
    C.leftCols<2>() = M;

    constraint->A_robot() = this->robot;
    constraint->weight().setZero();
    constraint->C() = C;
    constraint->dl() = l;
    constraint->du() = u;
  }

  bool solveMLP(const cnoid::BodyPtr currentRobot,
                const std::shared_ptr<Environment>& environment,
                const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                const std::string& prevSwingEEF,
                const std::vector<std::pair<std::vector<double>, std::string> >& targetRootPath, // angle, mode
                const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                std::vector<RobotState>& outputPath, // variablesの次元に対応
                std::string& swingEEF,
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

    std::vector<cnoid::LinkPtr> rootPathVariables;
    rootPathVariables.push_back(param.abstractRobot->rootLink());

    // 今のcontactから、target root pathのsubgoal点を見つける.
    std::vector<std::string> moveEEF, newEEF, excessContact;
    double currentIdx = -1;
    for(int i=targetRootPath.size()-1;i>=0;i--){

      frame2Link(targetRootPath[i].first, rootPathVariables);
      param.abstractRobot->calcForwardKinematics(false);
      multicontact_locomotion_planner::calcHorizontal(param.horizontals);
      param.horizontalRobot->calcForwardKinematics(false);

      if(param.modes.find(targetRootPath[i].second)->second->isContactSatisfied(currentContacts, true, moveEEF, newEEF, excessContact)){
        currentIdx = i;
        break;
      }
    }

    if(currentIdx == -1){
      std::cerr << "[" << __FUNCTION__ << "] subGoal is not found" << std::endl;
      return false;
    }


    if(param.debugLevel>=2){
      std::cerr << "current: " << currentIdx << " " << targetRootPath[currentIdx].second << " [" << targetRootPath[currentIdx].first[0] << " " << targetRootPath[currentIdx].first[1] << " " << targetRootPath[currentIdx].first[2] << " " << targetRootPath[currentIdx].first[3] << " " << targetRootPath[currentIdx].first[4] << " " << targetRootPath[currentIdx].first[5] << " " << targetRootPath[currentIdx].first[6] << "]" << std::endl;
      std::cerr << " newEEF";
      for(int i=0;i<newEEF.size();i++) std::cerr << " " << newEEF[i];
      std::cerr << " excessContact";
      for(int i=0;i<excessContact.size();i++) std::cerr << " " << excessContact[i];
      std::cerr << " moveEEF";
      for(int i=0;i<moveEEF.size();i++) std::cerr << " " << moveEEF[i];
      std::cerr << std::endl;
    }

    bool swingEEFFound = false;
    std::shared_ptr<Contact> breakContact = nullptr;
    std::shared_ptr<EndEffector> targetEEF = nullptr; // 次に接触させるEEF
    std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> targetReachabilityConstraint; // large
    int subGoalIdx = -1;
    std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> currentReachabilityConstraint; // large

    if(newEEF.size() > 0){ // 足りないcontactをmakeする
      targetEEF = param.endEffectors.find(newEEF[0])->second;
      std::shared_ptr<Mode> targetMode = param.modes.find(targetRootPath[currentIdx].second)->second;
      targetReachabilityConstraint = targetMode->reachabilityConstraintsLarge[std::find(targetMode->eefs.begin(),targetMode->eefs.end(),newEEF[0]) - targetMode->eefs.begin()];
      subGoalIdx = currentIdx;
      for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it=currentContacts.begin(); it!=currentContacts.end();it++){
        if(targetEEF->limbLinks.find(it->second->link1) != targetEEF->limbLinks.end() ||
           targetEEF->limbLinks.find(it->second->link2) != targetEEF->limbLinks.end()){
          breakContact = it->second;
          break;
        }
      }
      swingEEFFound = true;

    }else if(excessContact.size() > 0){
      // 不要なContactをbreakさせる
      breakContact = currentContacts.find(excessContact[0])->second;
      subGoalIdx = currentIdx;
      swingEEFFound = true;

    }else if(moveEEF.size() > 0){ // 届かないcontactをswingする
      breakContact = currentContacts.find(moveEEF[0])->second;
      targetEEF = param.endEffectors.find(moveEEF[0])->second;
      std::shared_ptr<Mode> targetMode = param.modes.find(targetRootPath[currentIdx].second)->second;
      targetReachabilityConstraint = targetMode->reachabilityConstraintsLarge[std::find(targetMode->eefs.begin(),targetMode->eefs.end(),moveEEF[0]) - targetMode->eefs.begin()];
      subGoalIdx = currentIdx;
      swingEEFFound = true;

    }else{ // last or 次idxでmove/fail/excessが2以上
      if(currentIdx+1 == targetRootPath.size()){
        std::cerr << "solved without contact transition" << param.robot->rootLink()->p().transpose() << std::endl;
        return true;
      }

      std::vector<std::string> moveEEFnext, newEEFnext, excessContactnext;

      frame2Link(targetRootPath[currentIdx+1].first, rootPathVariables);
      param.abstractRobot->calcForwardKinematics(false);
      multicontact_locomotion_planner::calcHorizontal(param.horizontals);
      param.horizontalRobot->calcForwardKinematics(false);

      param.modes.find(targetRootPath[currentIdx+1].second)->second->isContactSatisfied(currentContacts, true, moveEEFnext, newEEFnext, excessContactnext);

      if(param.debugLevel>=2){
        std::cerr << " newEEFnext";
        for(int i=0;i<newEEFnext.size();i++) std::cerr << " " << newEEFnext[i];
        std::cerr << " excessContactnext";
        for(int i=0;i<excessContactnext.size();i++) std::cerr << " " << excessContactnext[i];
        std::cerr << " moveEEFnext";
        for(int i=0;i<moveEEFnext.size();i++) std::cerr << " " << moveEEFnext[i];
        std::cerr << std::endl;
      }

      if(moveEEFnext.size() > 0){
        int idx = 0;
        for(;idx+1<moveEEFnext.size();idx++){
          if(moveEEFnext[idx] != prevSwingEEF) break;
        }

        // moveEEFnextをnextのregionとcurrentのregionの共通部分に移せば、次はmoveEEFnextが一つ減る.
        breakContact = currentContacts.find(moveEEFnext[idx])->second;
        targetEEF = param.endEffectors.find(moveEEFnext[idx])->second;
        std::shared_ptr<Mode> targetMode = param.modes.find(targetRootPath[currentIdx+1].second)->second;
        targetReachabilityConstraint = targetMode->reachabilityConstraintsLarge[std::find(targetMode->eefs.begin(),targetMode->eefs.end(),moveEEFnext[idx]) - targetMode->eefs.begin()];
        subGoalIdx = currentIdx + 1;
        std::shared_ptr<Mode> currentMode = param.modes.find(targetRootPath[currentIdx].second)->second;
        currentReachabilityConstraint = currentMode->reachabilityConstraintsLarge[std::find(currentMode->eefs.begin(),currentMode->eefs.end(),moveEEFnext[idx]) - currentMode->eefs.begin()];
        swingEEFFound = true;

        // rootpathの定義より、newEEFNext <=2 && excessContactnext <= 2. newEEFnextがあるなら、excessContactnextはあったとしてもlimblinkに含まれる
      }else if(newEEFnext.size() > 0){
        // nextのregionに接触させる.次はnextへ進む
        targetEEF = param.endEffectors.find(newEEFnext[0])->second;
        std::shared_ptr<Mode> targetMode = param.modes.find(targetRootPath[currentIdx+1].second)->second;
        targetReachabilityConstraint = targetMode->reachabilityConstraintsLarge[std::find(targetMode->eefs.begin(),targetMode->eefs.end(),newEEFnext[0]) - targetMode->eefs.begin()];
        subGoalIdx = currentIdx + 1;
        for(std::unordered_map<std::string, std::shared_ptr<Contact> >::const_iterator it=currentContacts.begin(); it!=currentContacts.end();it++){
          if(targetEEF->limbLinks.find(it->second->link1) != targetEEF->limbLinks.end() ||
             targetEEF->limbLinks.find(it->second->link2) != targetEEF->limbLinks.end()){
            breakContact = it->second;
            break;
          }
        }
        swingEEFFound = true;

      }else{
        // excesssContactnextをbreakする. 次はnextへ進む
        breakContact = currentContacts.find(excessContactnext[0])->second;
        subGoalIdx = currentIdx;
        swingEEFFound = true;
      }
    }

    if(!swingEEFFound){
      std::cerr << "[" << __FUNCTION__ << "] new subGoal is not found" << std::endl;
      frame2Link(currentAngle, variables);
      param.robot->calcForwardKinematics(false);
      param.robot->calcCenterOfMass();
      multicontact_locomotion_planner::calcAssoc(param.assocs);
      param.abstractRobot->calcForwardKinematics(false);
      multicontact_locomotion_planner::calcHorizontal(param.horizontals);
      param.horizontalRobot->calcForwardKinematics(false);
      return false;
    }

    if(param.debugLevel>=2){
      std::cerr << "subGoal: " << subGoalIdx << " " << targetRootPath[subGoalIdx].second << " [" << targetRootPath[subGoalIdx].first[0] << " " << targetRootPath[subGoalIdx].first[1] << " " << targetRootPath[subGoalIdx].first[2] << " " << targetRootPath[subGoalIdx].first[3] << " " << targetRootPath[subGoalIdx].first[4] << " " << targetRootPath[subGoalIdx].first[5] << " " << targetRootPath[subGoalIdx].first[6] << "]" << std::endl;
    }

    std::shared_ptr<ik_constraint2::PositionConstraint> subGoalConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
    subGoalConstraint->A_link() = param.robot->rootLink();
    subGoalConstraint->B_link() = nullptr;
    subGoalConstraint->B_localpos().translation()[0] = targetRootPath[currentIdx].first[0];
    subGoalConstraint->B_localpos().translation()[1] = targetRootPath[currentIdx].first[1];
    subGoalConstraint->B_localpos().translation()[2] = targetRootPath[currentIdx].first[2];
    subGoalConstraint->B_localpos().linear() = cnoid::Quaternion(targetRootPath[currentIdx].first[6],
                                                          targetRootPath[currentIdx].first[3],
                                                          targetRootPath[currentIdx].first[4],
                                                          targetRootPath[currentIdx].first[5]).toRotationMatrix();
    subGoalConstraint->weight().head<3>() = /*3 **/ cnoid::Vector3::Ones();
    subGoalConstraint->weight().tail<3>() = /*3 **/ cnoid::Vector3::Ones();
    subGoalConstraint->precision() = 1e10; // always success.

    // robotを初期姿勢に戻す.
    frame2Link(currentAngle, variables);
    param.robot->calcForwardKinematics(false);
    param.robot->calcCenterOfMass();

    // abstractRobot, horizontalRobotは、subGoalの位置にする
    multicontact_locomotion_planner::frame2Link(targetRootPath[subGoalIdx].first, std::vector<cnoid::LinkPtr>{param.abstractRobot->rootLink()});
    param.abstractRobot->calcForwardKinematics(false);
    multicontact_locomotion_planner::calcHorizontal(param.horizontals);
    param.horizontalRobot->calcForwardKinematics(false);

    if(param.debugLevel >= 3){
      if(param.viewer){
        param.viewer->drawObjects();
      }
      std::cerr << "check subGoal2. Press ENTER:" << std::endl;
      getchar();
    }

    if(targetEEF) swingEEF = targetEEF->name; // ここでセットしておくことで、break, makeで失敗した場合に次のeefに移れる

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
        std::shared_ptr<ik_constraint2::COMConstraint> constraint = std::make_shared<ik_constraint2::COMConstraint>();
        std::unordered_map<std::string, std::shared_ptr<Contact> > targetContacts = currentContacts;
        targetContacts.erase(breakContact->name);
        param.robotIKInfo->updateCOMConstraint(targetContacts, constraint);

        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > targetConstraints{constraint};
        bool solved = param.robotIKInfo->solveFullbodyIK(variables,
                                                         currentContacts,
                                                         std::unordered_map<std::string, std::shared_ptr<Contact> >(),
                                                         targetConstraints,
                                                         std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >{subGoalConstraint},
                                                         breakPath1);
        if(false/*!solved*/){ // 不正確でも良いので緩和. TODO緩和してはいけない
          std::cerr << "[" << __FUNCTION__ << "] breakPath is not found" << std::endl;
          frame2Link(currentAngle, variables);
          param.robot->calcForwardKinematics(false);
          param.robot->calcCenterOfMass();
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
        param.robot->calcCenterOfMass();

        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = breakContact->link1;
        constraint->A_localpos() = breakContact->localPose1 * breakContact->preContactOffset.inverse();
        constraint->B_link() = breakContact->link2;
        constraint->B_localpos() = breakContact->localPose2;

        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > targetConstraints{constraint};
        std::unordered_map<std::string, std::shared_ptr<Contact> > nearContacts;
        nearContacts[breakContact->name] = breakContact;
        bool solved = param.robotIKInfo->solveFullbodyIK(variables,
                                                         currentContactsAfterBreak,
                                                         nearContacts,
                                                         targetConstraints,
                                                         std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >{subGoalConstraint},
                                                         breakPath2);

        if(false/*!solved*/){ // always solved. 多少ずれていてもいいので
          std::cerr << "[" << __FUNCTION__ << "] post contact Path is not found" << std::endl;
          frame2Link(currentAngle, variables);
          param.robot->calcForwardKinematics(false);
          param.robot->calcCenterOfMass();
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
    std::shared_ptr<std::vector<std::vector<double> > > makePath2 = std::make_shared<std::vector<std::vector<double> > >();
    std::unordered_map<std::string, std::shared_ptr<Contact> > currentContactsAfterMake = currentContactsAfterBreak;
    if(targetEEF){
      if(param.debugLevel>=2){
        std::cerr << "makeContact: " << std::endl;
        std::cerr << "makeContact: " << targetEEF->name << std::endl;
      }

      // subGoalのroot位置のreachability内で環境接触候補点を絞る
      Eigen::Matrix<double, 3, Eigen::Dynamic> region = targetReachabilityConstraint->A_link()->T() * choreonoid_qhull::meshToEigen(targetReachabilityConstraint->A_link()->collisionShape());

      if(currentReachabilityConstraint){ // modeが違うとregionの形状も違う
        multicontact_locomotion_planner::frame2Link(targetRootPath[currentIdx].first, std::vector<cnoid::LinkPtr>{param.abstractRobot->rootLink()});
        param.abstractRobot->calcForwardKinematics(false);
        multicontact_locomotion_planner::calcHorizontal(param.horizontals);
        param.horizontalRobot->calcForwardKinematics(false);
        Eigen::Matrix<double, 3, Eigen::Dynamic> region2 = currentReachabilityConstraint->A_link()->T() * choreonoid_qhull::meshToEigen(currentReachabilityConstraint->A_link()->collisionShape());
        Eigen::MatrixXd intersection;
        convex_polyhedron_intersection::intersection(region, region2, intersection);
        region = intersection;

        multicontact_locomotion_planner::frame2Link(targetRootPath[subGoalIdx].first, std::vector<cnoid::LinkPtr>{param.abstractRobot->rootLink()});
        param.abstractRobot->calcForwardKinematics(false);
        multicontact_locomotion_planner::calcHorizontal(param.horizontals);
        param.horizontalRobot->calcForwardKinematics(false);
      }
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
          targetRegion.back().pose = targetRegion.back().pose;
          targetRegion.back().shape = intersectionLocal;
        }
      }
      if(targetRegion.size()==0){
        std::cerr << "[" << __FUNCTION__ << "] targetRegion is not found" << std::endl;
        frame2Link(currentAngle, variables);
        param.robot->calcForwardKinematics(false);
        param.robot->calcCenterOfMass();
        multicontact_locomotion_planner::calcAssoc(param.assocs);
        param.abstractRobot->calcForwardKinematics(false);
        multicontact_locomotion_planner::calcHorizontal(param.horizontals);
        param.horizontalRobot->calcForwardKinematics(false);

        return false;
      }

      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goals;
      for(int i=0;i<targetRegion.size();i++){
        std::shared_ptr<ik_constraint2_region_cdd::CddRegionConstraint> constraint = std::make_shared<ik_constraint2_region_cdd::CddRegionConstraint>();
        constraint->A_link() = targetEEF->parentLink;
        constraint->A_localpos() = targetEEF->localPose * targetEEF->preContactOffset.inverse();
        constraint->B_link() = nullptr;
        constraint->B_localpos() = targetRegion[i].pose;
        constraint->eval_link() = nullptr;
        constraint->eval_localR() = targetRegion[i].pose.linear();
        constraint->weightR() = targetRegion[i].weightR;
        constraint->setVertices(targetRegion[i].shape);
        goals.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >{constraint});
      }

      if(!param.robotIKInfo->solveGlobalIK(variables,
                                           currentContactsAfterBreak,
                                           std::unordered_map<std::string, std::shared_ptr<Contact> >(),
                                           goals,
                                           std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >{subGoalConstraint},
                                           targetEEF->parentLink,
                                           targetEEF->localPose,
                                           makePath1
                                           )){
        std::cerr << "[" << __FUNCTION__ << "] swing Path is not found" << std::endl;
        frame2Link(currentAngle, variables);
        param.robot->calcForwardKinematics(false);
        param.robot->calcCenterOfMass();
        multicontact_locomotion_planner::calcAssoc(param.assocs);
        param.abstractRobot->calcForwardKinematics(false);
        multicontact_locomotion_planner::calcHorizontal(param.horizontals);
        param.horizontalRobot->calcForwardKinematics(false);

        return false;
      }

      frame2Link(makePath1->back(), variables);
      param.robot->calcForwardKinematics(false);
      param.robot->calcCenterOfMass();

      if(param.debugLevel >= 3){
        if(param.viewer){
          param.viewer->drawObjects();
        }
        std::cerr << "swing. Press ENTER:" << std::endl;
        getchar();
      }


      // pre contact pose
      std::vector<cnoid::LinkPtr> variables_near = variables;
      for(int i=0;i<targetEEF->preContactAngles.size();i++){
        targetEEF->preContactAngles[i].first->q() = targetEEF->preContactAngles[i].second;

        std::vector<cnoid::LinkPtr>::iterator it = std::remove(variables_near.begin(), variables_near.end(), targetEEF->preContactAngles[i].first);
        variables_near.erase(it, variables_near.end());
      }
      param.robot->calcForwardKinematics();
      param.robot->calcCenterOfMass();

      std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
      constraint->A_link() = targetEEF->parentLink;
      constraint->A_localpos() = targetEEF->localPose;
      constraint->B_link() = nullptr;
      constraint->B_localpos() = targetEEF->parentLink->T() * targetEEF->localPose * targetEEF->preContactOffset.inverse();

      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > targetConstraints{constraint};
      std::unordered_map<std::string, std::shared_ptr<Contact> > nearContacts;
      nearContacts[targetEEF->name] = targetEEF->generateContact();
      bool solved = param.robotIKInfo->solveFullbodyIK(variables,
                                                       currentContactsAfterBreak,
                                                       nearContacts,
                                                       targetConstraints,
                                                       std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >{subGoalConstraint},
                                                       makePath2);

      if(false/*!solved*/){
        std::cerr << "[" << __FUNCTION__ << "] contact Path is not found" << std::endl;
        frame2Link(currentAngle, variables);
        param.robot->calcForwardKinematics(false);
        param.robot->calcCenterOfMass();
        multicontact_locomotion_planner::calcAssoc(param.assocs);
        param.abstractRobot->calcForwardKinematics(false);
        multicontact_locomotion_planner::calcHorizontal(param.horizontals);
        param.horizontalRobot->calcForwardKinematics(false);

        return false;
      }

      {
        for(int i=0;i<targetEEF->contactAngles.size();i++){
          targetEEF->contactAngles[i].first->q() = targetEEF->contactAngles[i].second;
        }
        std::vector<double> frame;
        link2Frame(variables,frame);
        makePath2->push_back(frame);
      }
      param.robot->calcForwardKinematics(false);
      param.robot->calcCenterOfMass();

      currentContactsAfterMake[targetEEF->name] = targetEEF->generateContact();

      if(param.debugLevel >= 3){
        if(param.viewer){
          param.viewer->drawObjects();
        }
        std::cerr << "make contact. Press ENTER:" << std::endl;
        getchar();
      }

    }


    if(param.debugLevel>=2){
      std::cerr << "solved " << std::endl;
    }


    // break->make指令およびangle-vectorが出てくる. (stateが別れている.)
    outputPath.clear();
    for(int i=0;i<breakPath1->size();i++){
      outputPath.resize(outputPath.size()+1);
      outputPath.back().jointAngle = breakPath1->at(i);
      outputPath.back().contacts = currentContacts;
    }
    for(int i=0;i<breakPath2->size();i++){
      outputPath.resize(outputPath.size()+1);
      outputPath.back().jointAngle = breakPath2->at(i);
      outputPath.back().contacts = currentContactsAfterBreak;
    }
    for(int i=0;i<makePath1->size();i++){
      outputPath.resize(outputPath.size()+1);
      outputPath.back().jointAngle = makePath1->at(i);
      outputPath.back().contacts = currentContactsAfterBreak;
    }
    for(int i=0;i<makePath2->size();i++){
      outputPath.resize(outputPath.size()+1);
      outputPath.back().jointAngle = makePath2->at(i);
      outputPath.back().contacts = currentContactsAfterBreak;
    }
    {
      outputPath.push_back(outputPath.back());
      outputPath.back().contacts = currentContactsAfterMake;
    }

    multicontact_locomotion_planner::calcAssoc(param.assocs);
    param.abstractRobot->calcForwardKinematics(false);
    multicontact_locomotion_planner::calcHorizontal(param.horizontals);
    param.horizontalRobot->calcForwardKinematics(false);

    return true;
  }

  bool solveRBRRT(const std::shared_ptr<Environment>& environment,
                  const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                  const cnoid::Position goal,
                  const MLPParam& param,
                  std::vector<std::pair<std::vector<double>, std::string> >& outputRootPath // angle, mode
                  ){

    std::vector<cnoid::LinkPtr> variables;
    variables.push_back(param.abstractRobot->rootLink());

    std::shared_ptr<rb_rrt_solver::ConditionOR> conditions = std::make_shared<rb_rrt_solver::ConditionOR>();
    for(std::unordered_map<std::string, std::shared_ptr<Mode> >::const_iterator it=param.modes.begin(); it!=param.modes.end(); it++){
      conditions->children.push_back(it->second->generateCondition(param.endEffectors, environment));
    }

    std::vector<double> goals;
    {
      cnoid::Position org = param.abstractRobot->rootLink()->T();
      param.abstractRobot->rootLink()->T() = goal;
      rb_rrt_solver::link2Frame(variables, goals);
      param.abstractRobot->rootLink()->T() = org;
    }


    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
    if(!rb_rrt_solver::solveRBRRT(variables,
                                  param.horizontals,
                                  conditions,
                                  goals,
                                  path,
                                  param.rbrrtParam
                                  )){
      std::cerr << "solveRBRRT failed" << std::endl;
      return false;
    }

    outputRootPath.resize(path->size());
    std::string prevMode = "";
    for(int i=0;i<path->size();i++){
      outputRootPath[i].first = path->at(i);
      frame2Link(path->at(i), variables);
      param.abstractRobot->calcForwardKinematics(false);
      multicontact_locomotion_planner::calcHorizontal(param.horizontals);
      param.horizontalRobot->calcForwardKinematics(false);

      double maxScore = -1;
      int idx = 0;
      std::vector<std::string> moveEEF, newEEF, excessContact;
      for(std::unordered_map<std::string, std::shared_ptr<Mode> >::const_iterator it=param.modes.begin(); it!=param.modes.end(); it++){
        if((it->second->score > maxScore || prevMode == it->first) &&
           conditions->children[idx]->isValid()){
          if(i!=0 || param.modes.find(it->first)->second->isContactSatisfied(currentContacts, true, moveEEF, newEEF, excessContact)) { // i = 0の場合、初期姿勢を満たしている必要がある.
            if(prevMode == it->first) {
              maxScore = it->second->score;
              outputRootPath[i].second = prevMode = it->first;
              break;
            }else{
              maxScore = it->second->score;
              outputRootPath[i].second = prevMode = it->first;
            }
          }
        }
        idx++;
      }
      if(maxScore == -1){
        std::cerr << __FUNCTION__ << "outputRooPath not found" << std::endl;
        return false;
      }
    }

    // 連続化
    for(int i=0;i+1<outputRootPath.size();i++){
      if(outputRootPath[i].second == "biped" && outputRootPath[i+1].second == "quadruped"){
        outputRootPath.insert(outputRootPath.begin()+i+1, std::pair<std::vector<double>,std::string>(outputRootPath[i].first,"quadruped_large"));
      }else if(outputRootPath[i].second == "biped" && outputRootPath[i+1].second == "grasp"){
        outputRootPath.insert(outputRootPath.begin()+i+1, std::pair<std::vector<double>,std::string>(outputRootPath[i].first,"grasp_large"));
      }else if(outputRootPath[i].second == "quadruped" && outputRootPath[i+1].second == "biped"){
        outputRootPath.insert(outputRootPath.begin()+i+1, std::pair<std::vector<double>,std::string>(outputRootPath[i+1].first,"quadruped_large"));
      }else if(outputRootPath[i].second == "quadruped" && outputRootPath[i+1].second == "grasp_large"){
        outputRootPath.insert(outputRootPath.begin()+i+1, std::pair<std::vector<double>,std::string>(outputRootPath[i+1].first,"quadruped_large"));
      }else if(outputRootPath[i].second == "grasp" && outputRootPath[i+1].second == "biped"){
        outputRootPath.insert(outputRootPath.begin()+i+1, std::pair<std::vector<double>,std::string>(outputRootPath[i+1].first,"grasp_large"));
      }else if(outputRootPath[i].second == "grasp" && outputRootPath[i+1].second == "quadruped_large"){
        outputRootPath.insert(outputRootPath.begin()+i+1, std::pair<std::vector<double>,std::string>(outputRootPath[i+1].first,"grasp_large"));
      }else if(outputRootPath[i].second == "quadruped_large" && outputRootPath[i+1].second == "grasp"){
        outputRootPath.insert(outputRootPath.begin()+i+1, std::pair<std::vector<double>,std::string>(outputRootPath[i].first,"grasp_large"));
      }else if(outputRootPath[i].second == "grasp_large" && outputRootPath[i+1].second == "quadruped"){
        outputRootPath.insert(outputRootPath.begin()+i+1, std::pair<std::vector<double>,std::string>(outputRootPath[i].first,"quadruped_large"));
      }

    }

    return true;
  }
};
