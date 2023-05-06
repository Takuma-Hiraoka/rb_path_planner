#include <rb_rrt_solver/rb_rrt_solver.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace rb_rrt_solver{

  bool solveRBRRT(const std::vector<cnoid::LinkPtr>& variables, // 0: variables
                  const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& constraints, // 0: constraints
                  const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& inverseConstraints, // 0: constraints
                  const std::vector<double>& goal, // 0: angles.
                  std::shared_ptr<std::vector<std::vector<double> > > path, // 0: states. 1: angles
                  const RBRRTParam& param
                  ){
    std::vector<std::vector<cnoid::LinkPtr> > variabless{variables};
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraintss{constraints};
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > inverseConstraintss{inverseConstraints};
    std::shared_ptr<UintQueue> modelQueue = std::make_shared<UintQueue>();
    modelQueue->push(0);

    if(param.threads >= 2){
      std::set<cnoid::BodyPtr> bodies = getBodies(variables);
      for(int i=1;i<param.threads;i++){
        modelQueue->push(i);
        std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
        for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
          modelMap[*it] = (*it)->clone();
        }
        variabless.push_back(std::vector<cnoid::LinkPtr>(variables.size()));
        for(int v=0;v<variables.size();v++){
          variabless.back()[v] = modelMap[variables[v]->body()]->link(variables[v]->index());
        }
        constraintss.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(constraints.size()));
        for(int j=0;j<constraints.size();j++){
          constraintss.back()[j] = constraints[j]->clone(modelMap);
        }
        inverseConstraintss.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(inverseConstraints.size()));
        for(int j=0;j<inverseConstraints.size();j++){
          inverseConstraintss.back()[j] = inverseConstraints[j]->clone(modelMap);
        }
      }
    }

    return solveRBRRT(variabless,
                      constraintss,
                      inverseConstraintss,
                      goal,
                      modelQueue,
                      path,
                      param);

  }


  bool solveRBRRT(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                  const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints, // 0: modelQueue, 1: constraints
                  const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& inverseConstraints, // 0: modelQueue, 1: constraints
                  const std::vector<double>& goal, // 0: angles
                  std::shared_ptr<UintQueue> modelQueue,
                  std::shared_ptr<std::vector<std::vector<double> > > path, // 0: states. 1: angles
                  const RBRRTParam& param
                  ){
    if((variables.size() == 0) ||
       (variables.size() != constraints.size()) ||
       (constraints.size() != inverseConstraints.size())
       ){
      std::cerr << "[solveRBRRT] size mismatch" << std::endl;
      return false;
    }

    ompl::base::StateSpacePtr stateSpace = createAmbientSpace(variables[0], param.maxTranslation);
    ompl::base::SpaceInformationPtr spaceInformation = std::make_shared<ompl::base::SpaceInformation>(stateSpace);
    spaceInformation->setStateValidityCheckingResolution(param.stateValidityCheckingResolution);
    RBRRTStateValidityCheckerPtr stateValidityChecker = std::make_shared<RBRRTStateValidityChecker>(spaceInformation,
                                                                                                    modelQueue,
                                                                                                    constraints,
                                                                                                    inverseConstraints,
                                                                                                    variables);
    stateValidityChecker->viewer() = param.viewer;
    stateValidityChecker->drawLoop() = param.drawLoop;

    spaceInformation->setStateValidityChecker(stateValidityChecker);
    spaceInformation->setup(); // ここでsetupを呼ばないと、stateSpaceがsetupされないのでlink2State等ができない

    ompl::base::ProblemDefinitionPtr problemDefinition = std::make_shared<ompl::base::ProblemDefinition>(spaceInformation);
    ompl::base::ScopedState<> startState(stateSpace);
    link2State(variables[0], stateSpace, startState.get());
    problemDefinition->clearStartStates();
    problemDefinition->addStartState(startState);
    ompl::base::ScopedState<> goalState(stateSpace);
    frame2State(goal, stateSpace, goalState.get());
    problemDefinition->setGoalState(goalState);

    ompl::base::PlannerPtr planner;
    {
      std::shared_ptr<ompl::geometric::RRTConnect> planner_ = std::make_shared<ompl::geometric::RRTConnect>(spaceInformation, false);
      planner_->setRange(param.range);
      planner = planner_;
    }

    if(!spaceInformation->isSetup()) spaceInformation->setup();
    planner->setProblemDefinition(problemDefinition);
    if(!planner->isSetup()) planner->setup();

    ompl::base::PlannerStatus solved;
    {
      ompl::time::point start = ompl::time::now();
      solved = planner->solve(param.timeout);
      double planTime = ompl::time::seconds(ompl::time::now() - start);
      if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
        OMPL_INFORM("Solution found in %f seconds", planTime);
      else
        OMPL_INFORM("No solution found after %f seconds", planTime);
    }

    if(path != nullptr){
      ompl::geometric::PathGeometricPtr solutionPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(problemDefinition->getSolutionPath());
      if(solutionPath == nullptr) {
        path->resize(0);
      }else{
        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        ompl::geometric::PathSimplifierPtr pathSimplifier = std::make_shared<ompl::geometric::PathSimplifier>(spaceInformation);
        {
          ompl::time::point start = ompl::time::now();
          std::size_t numStates = solutionPath->getStateCount();
          stateValidityChecker->viewer() = nullptr; // simplifySolution()中は描画しない
          pathSimplifier->simplify(*solutionPath, param.timeout);
          stateValidityChecker->viewer() = param.viewer; // simplifySolution()中は描画しない
          double simplifyTime = ompl::time::seconds(ompl::time::now() - start);
          OMPL_INFORM("Path simplification took %f seconds and changed from %d to %d states",
                      simplifyTime, numStates, solutionPath->getStateCount());
        }

        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        solutionPath->interpolate();
        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        // 途中の軌道をpathに入れて返す
        path->resize(solutionPath->getStateCount());
        for(int j=0;j<solutionPath->getStateCount();j++){
          //stateSpace->getDimension()は,SO3StateSpaceが3を返してしまう(実際はquaternionで4)ので、使えない
          state2Frame(stateSpace, solutionPath->getState(j), path->at(j));
        }
      }
    }

    // goal stateをvariablesに反映して返す.
    if(problemDefinition->hasSolution()) {
      const ompl::geometric::PathGeometricPtr solutionPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(problemDefinition->getSolutionPath());
      state2Link(stateSpace, solutionPath->getState(solutionPath->getStateCount()-1), variables[0]);
      std::set<cnoid::BodyPtr> bodies = getBodies(variables[0]);
      for(std::set<cnoid::BodyPtr>::const_iterator it=bodies.begin(); it != bodies.end(); it++){
        (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
        (*it)->calcCenterOfMass();
      }
    }


    return solved == ompl::base::PlannerStatus::EXACT_SOLUTION;
  }


};
