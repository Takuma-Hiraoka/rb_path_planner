#include <multicontact_locomotion_planner/multicontact_locomotion_planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace multicontact_locomotion_planner{

  bool RobotIKInfo::solveFullbodyIK(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                                    const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
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
          ik_constraint2_distance_field::DistanceFieldCollisionConstraint::BoundingBox bbx;
          bbx.localPose = it->second->ignoreBoundingBoxLocalPose;
          bbx.parentLink = it->second->ignoreBoundingBoxParentLink;
          bbx.dimensions = it->second->ignoreBoundingBoxDimensions;
          this->envConstraints[i]->ignoreBoundingBox().push_back(bbx);
        }
        constraints1.push_back(this->envConstraints[i]);
      }
    }

    // 重心実行可能領域
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints2;//TODO

    // target task
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints3 = targetConstraints;

    // nominal task
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints4 = this->nominalConstraints;

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1,constraints2,constraints3,constraints4};
    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables,
                                                                      constraints,
                                                                      this->tasks,
                                                                      this->pikParam);
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

    std::vector<double> currentAngle;
    body2Frame(currentRobot, currentAngle);

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
      if(dist <= param.subGoalDistance) {
        subGoalIdx = i;
        subGoalFound = true;
      }
    }
    if(!subGoalFound){
      std::cerr << "[" << __FUNCTION__ << "] subGoal is not found" << std::endl;
      return false;
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


    // 再度今のroot位置から、target root pathのsubgoal点を見つける. (subGoalが前回のiterationよりも前に進んでいることが期待される)
    subGoalIdx = -1;
    subGoalFound = false;
    for(int i=targetRootPath.size()-1;i>=0;i--){
      cnoid::Vector3 targetp(targetRootPath[i].first[0],targetRootPath[i].first[1],targetRootPath[i].first[2]);
      cnoid::Quaternion targetR(targetRootPath[i].first[6],
                                targetRootPath[i].first[3],
                                targetRootPath[i].first[4],
                                targetRootPath[i].first[5]);
      double dist = std::sqrt((currentRobot->rootLink()->p()-targetp).squaredNorm() + std::pow(cnoid::AngleAxis(currentRobot->rootLink()->R().transpose()*targetR).angle(),2));
      if(dist <= param.subGoalDistance) {
        subGoalIdx = i;
        subGoalFound = true;
      }
    }
    if(!subGoalFound){
      std::cerr << "[" << __FUNCTION__ << "] new subGoal is not found" << std::endl;
      frame2Body(currentAngle, currentRobot);
      return false;
    }

    std::shared_ptr<Mode> targetMode = param.modes.find(targetRootPath[subGoalIdx].second)->second;

    // 次にswingするeefを決める
    //   まず、targetModeにあって、currentContactsに無いEEFを探して、makeさせる. このとき、同一limbに別のcurrentContactがあるなら、それをbreakする.
    std::unordered_set<std::string> unBreakableContacts; // TODO

    bool swingEEFFound = false;
    std::shared_ptr<Contact> breakContact = nullptr;
    std::unordered_map<std::string, std::shared_ptr<Contact> > swingContacts = currentContacts; // 遊脚期のcontact
    std::shared_ptr<EndEffector> targetEEF = nullptr; // 次に接触させるEEF
    for(int i=0;i<targetMode->eefs.size();i++){
      if(currentContacts.find(targetMode->eefs[i]) == currentContacts.end()){
        targetEEF = param.endEffectors.find(targetMode->eefs[i])->second;
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
      for(int i=0;i<targetMode->eefs.size();i++){
        if(prevSwingEEF == targetMode->eefs[i]) {
          int nextEEFIdx = (i+1==targetMode->eefs.size()) ? 0 : i+1;

          // 候補点に今の接触位置が含まれているなら、別のlimbへ
          if(isInsideReachiability(targetRootPath[subGoalIdx].first,
                                   param.endEffectors.find(targetMode->eefs[nextEEFIdx])->second,
                                   currentContacts.find(targetMode->eefs[nextEEFIdx])->second)){
            continue;
          }

          targetEEF = param.endEffectors.find(targetMode->eefs[nextEEFIdx])->second;
          breakContact = currentContacts.find(targetMode->eefs[nextEEFIdx])->second;
          swingContacts.erase(targetMode->eefs[nextEEFIdx]);
          swingEEFFound = true;
          break;
        }
      }
    }
    if(!swingEEFFound){
      std::cerr << "[" << __FUNCTION__ << "] swing eef is not found" << std::endl;
      frame2Body(currentAngle, currentRobot);
      return false;
    }


    frame2Body(currentAngle, currentRobot);


    // breakするなら、contact breakability checkして、だめならnext limb
    std::shared_ptr<std::vector<std::vector<double> > > breakPath = std::make_shared<std::vector<std::vector<double> > >();
    if(breakContact){
      if(!solveContactBreakabilityIK(variables,
                                     currentContacts,
                                     swingContacts,
                                     breakContact,
                                     breakPath
                                     )){
        std::cerr << "[" << __FUNCTION__ << "] breakPath is not found" << std::endl;
        frame2Body(currentAngle, currentRobot);
        return false;
      }
    }

    // makeするなら、subGoalのroot位置のreachibility内で環境接触候補点を絞り、そのどこかに接触させる軌道を生成する. (goalのnominal root poseを与える)
    std::shared_ptr<std::vector<std::vector<double> > > makePath = std::make_shared<std::vector<std::vector<double> > >();
    if(targetEEF){
      if(!solveSwingTrajectory(variables,
                               swingContacts,
                               targetRootPath[subGoalIdx].first,
                               targetEEF,
                               makePath
                               )){
        std::cerr << "[" << __FUNCTION__ << "] makePath is not found" << std::endl;
        frame2Body(currentAngle, currentRobot);
        return false;
      }
    }


    // break->make指令およびangle-vectorが出てくる. (stateが別れている.)


    return true;
  }


};
