#include <multicontact_locomotion_planner/multicontact_locomotion_planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace multicontact_locomotion_planner{

  bool solveMLP(const cnoid::BodyPtr currentRobot,
                const std::unordered_map<std::string, std::shared_ptr<Contact> >& currentContacts,
                const std::string& prevSwingEEF,
                const std::vector<std::pair<std::vector<double>, std::string> >& targetRootPath, // angle, mode
                const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                std::vector<RobotState>& outputPath, // variablesの次元に対応

                const MLPParam& param){

    std::vector<double> currentAngle;
    body2Frame(currentRobot, currentAngle);

    // // 今のjoint angle, contact stateから、今のmodeを把握する
    // //   各mode - 1なら、そのmodeのbreak直後とみなせる. 各mode間が2以上離れているので、重複はない.
    // std::shared_ptr<Mode> currentMode = nullptr;
    // for(std::unordered_map<std::string, std::shared_ptr<Mode> >::const_iterator it=param.modes.begin(); it!=param.modes.end();it++){
    //   if(it->second->isBelongTo(currentState)) {
    //     currentMode = it->second;
    //     break;
    //   }
    // }
    // if(!currentMode){
    //   std::cerr << "[" << __FUNCTION__ << "] current mode is not found" << std::endl;
    //   return false;
    // }

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
    if(solveRootIK(variables,
                   targetRootPath[subGoalIdx].first,
                   path
                   )){
      outputPath.resize(path->size());
      for(int i=0;i<path->size();i++){
        outputPath[i].jointAngle = path->at(i);
        outputPath[i].contacts = currentContacts;
      }
      return true;
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
    bool swingEEFFound = false;
    std::unordered_map<std::string, std::shared_ptr<Contact> > swingContacts = currentContacts; // 遊脚期のcontact
    std::shared_ptr<EndEffector> targetEEF = nullptr; // 次に接触させるEEF
    for(int i=0;i<targetMode->eefs.size();i++){
      if(currentContacts.find(targetMode->eefs[i]) == currentContacts.end()){
        targetEEF = param.endEffectors.find(targetMode->eefs[i])->second;
        for(std::unordered_map<std::string, std::shared_ptr<Contact> >::iterator it=swingContacts.begin(); it!=swingContacts.end();){
          if(targetEEF->limbLinks.find(it->second->link1) != targetEEF->limbLinks.end() ||
             targetEEF->limbLinks.find(it->second->link2) != targetEEF->limbLinks.end()){
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
          if(it->first == targetMode->eefs[i]) found = true;
        }
        if(!found){
          swingContacts.erase(it->first);
          swingEEFFound = true;
          break;
        }
      }
    }
    //   最後に、prevSwingEEFの次のEEFをswingする



    frame2Body(currentAngle, currentRobot);


    

    // breakするなら、contact breakability checkして、だめならnext limb


    // makeするなら、swing期で、rootを可能な限りcmd_vel方向に進めるIKをして、限界まで進んだ位置からさらに+Xしたroot位置を用いて、reachibilityで環境接触候補点を絞る


    // 候補点に今の接触位置が含まれているなら、next limb


    // 含まれてないなら、それらに対してompl (goalのnominal root poseを与える). ダメならnext limb


    // break->make指令およびangle-vectorが出てくる. (stateが別れている.)


    return true;
  }
};
