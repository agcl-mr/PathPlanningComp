
#include "../Header Files/pre_built_algo.h"

algos::algos(void) {

}

void algos::updateObstacleMap(std::vector<Node>* node, int GRID_WIDTH, int GRID_HEIGHT, std::vector<Path>* paths) {
    map_width = GRID_WIDTH;
    map_height = GRID_HEIGHT;
    this->paths = paths;
    for (int i = 0; i < GRID_HEIGHT; i++) {
        map.push_back(std::vector<int>());
        for (int j = 0; j < GRID_WIDTH; j++) {
            Node cell = node->at(i * GRID_WIDTH + j);
            if (cell.empty) {
                map.at(i).push_back(0);
            }
            else {
                map.at(i).push_back(1);
            }
        }
    }
    std::cout << "image height = " << map_height << "\n";
    std::cout << "image width = " << map_width << "\n";

    // Create the state space
    // Specify the bounds for width and height
    double widthMin = 0.0;
    double widthMax = map_width * 1.0f;
    double heightMin = 0.0;
    double heightMax = map_height * 1.0f;

    // Create the state space (RealVectorStateSpace)
    space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, widthMin);
    bounds.setHigh(0, widthMax);
    bounds.setLow(1, heightMin);
    bounds.setHigh(1, heightMax);

    // Cast the state space pointer to RealVectorStateSpace
    auto* realVectorSpace = space->as<ob::RealVectorStateSpace>();

    // Set the bounds for each dimension
    realVectorSpace->setBounds(bounds);

    // Create a SpaceInformation object
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // Create a custom state validity checker with the obstacle map
    std::shared_ptr<MyStateValidityChecker> validityChecker = std::make_shared<MyStateValidityChecker>(map, si);

    // Set the StateValidityChecker for the SpaceInformation
    si->setStateValidityChecker(validityChecker);

    // Create a SimpleSetup object
    ss = std::make_shared<og::SimpleSetup>(si);
}

void algos::dijkstras(void) {

}

void algos::abitStar(void) {

    // Step 4: Set up the state space
    unsigned int DIMENSIONS = 2; // Example: 2D state space
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(DIMENSIONS));

    // Step 5: Create a space information instance
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

    // Step 6: Create a problem definition
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

    // Step 7: Set the start and goal states
    ompl::base::ScopedState<> start(space);
    // Set the start state
    // start[0] = ...
    // start[1] = ...
    // ...

    ompl::base::ScopedState<> goal(space);
    // Set the goal state
    // goal[0] = ...
    // goal[1] = ...
    // ...

    // Step 8: Create the planner and set its parameters
    ompl::geometric::ABITstar planner(si);
    planner.setPruneThresholdFraction(0.1);
    // Set other parameters as needed

    // Step 9: Set the problem definition for the planner
    planner.setProblemDefinition(pdef);

    // Step 10: Solve the planning problem
    double PLANNING_TIME = 5.0; // Example: 5 seconds planning time
    ompl::base::PlannerTerminationCondition ptc = ompl::base::timedPlannerTerminationCondition(PLANNING_TIME);
    ompl::base::PlannerStatus status = planner.solve(ptc);

    // Step 11: Extract the solution path
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
        ompl::base::PathPtr solutionPath = pdef->getSolutionPath();
        // Convert to PathGeometric
        ompl::geometric::PathGeometric geometricPath(si);
        geometricPath = *solutionPath->as<ompl::geometric::PathGeometric>();
        // Process or visualize the solution path
    }

    return;
}

void algos::abitStar2(int start_cell_index, int goal_cell_index, consolidated_result* result) {

    // Specify the start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = start_cell_index % map_width; // Set the start coordinates
    start[1] = start_cell_index / map_width;
    goal[0] = goal_cell_index % map_width; // Set the goal coordinates
    goal[1] = goal_cell_index / map_width;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal);

    // Set projection evaluation
    //auto projectionEvaluator = std::make_shared<MyProjectionEvaluator>(space);
    //ss->getStateSpace()->registerDefaultProjection(projectionEvaluator);

    // Create the planner (ABITstar*)
    auto planner(std::make_shared<og::ABITstar>(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    // Attempt to solve the problem within the given time
    ob::PlannerStatus solved = ss->solve(0.015);

    if (solved)
    {
        show_results(nullptr);
    }

    return;
}

void algos::prm(int start_cell_index, int goal_cell_index, consolidated_result* result) {

    // Specify the start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = start_cell_index % map_width; // Set the start coordinates
    start[1] = start_cell_index / map_width;
    goal[0] = goal_cell_index % map_width; // Set the goal coordinates
    goal[1] = goal_cell_index / map_width;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal);

    // Create the planner (PRM*)
    auto planner(std::make_shared<og::PRM>(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    // Attempt to solve the problem within the given time
    ob::PlannerStatus solved = ss->solve(0.5);

    if (solved)
    {
        show_results(&result->prm);
    }

    return;
}

void algos::fmt(int start_cell_index, int goal_cell_index, consolidated_result* result) {

    // Specify the start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = start_cell_index % map_width; // Set the start coordinates
    start[1] = start_cell_index / map_width;
    goal[0] = goal_cell_index % map_width; // Set the goal coordinates
    goal[1] = goal_cell_index / map_width;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal);

    // Create the planner (FMT*)
    auto planner(std::make_shared<og::FMT>(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    // Attempt to solve the problem within the given time
    ob::PlannerStatus solved = ss->solve(0.5);

    if (solved)
    {
        show_results(&result->fmt);
    }

    return;
}

void algos::est(int start_cell_index, int goal_cell_index, consolidated_result* result) {

    // Specify the start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = start_cell_index % map_width; // Set the start coordinates
    start[1] = start_cell_index / map_width;
    goal[0] = goal_cell_index % map_width; // Set the goal coordinates
    goal[1] = goal_cell_index / map_width;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal);

    // Create the planner (EST*)
    auto planner(std::make_shared<og::EST>(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    // Attempt to solve the problem within the given time
    ob::PlannerStatus solved = ss->solve(0.5);

    if (solved)
    {
        show_results(&result->est);
    }

    return;
}

void algos::rlrt(int start_cell_index, int goal_cell_index, consolidated_result* result) {

    // Specify the start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = start_cell_index % map_width; // Set the start coordinates
    start[1] = start_cell_index / map_width;
    goal[0] = goal_cell_index % map_width; // Set the goal coordinates
    goal[1] = goal_cell_index / map_width;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal);

    // Create the planner (RLRT*)
    auto planner(std::make_shared<og::RLRT>(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    // Attempt to solve the problem within the given time
    ob::PlannerStatus solved = ss->solve(0.5);

    if (solved)
    {
        show_results(&result->rlrt);
    }

    return;
}

void algos::sst(int start_cell_index, int goal_cell_index, consolidated_result* result) {

    // Specify the start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = start_cell_index % map_width; // Set the start coordinates
    start[1] = start_cell_index / map_width;
    goal[0] = goal_cell_index % map_width; // Set the goal coordinates
    goal[1] = goal_cell_index / map_width;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal);

    // Create the planner (SST*)
    auto planner(std::make_shared<og::SST>(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    // Attempt to solve the problem within the given time
    ob::PlannerStatus solved = ss->solve(0.5);

    if (solved)
    {
        show_results(&result->sst);
    }

    return;
}

void algos::xxl(int start_cell_index, int goal_cell_index, consolidated_result* result) {

    // Specify the start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = start_cell_index % map_width; // Set the start coordinates
    start[1] = start_cell_index / map_width;
    goal[0] = goal_cell_index % map_width; // Set the goal coordinates
    goal[1] = goal_cell_index / map_width;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal);

    // Create the planner (XXL*)
    auto planner(std::make_shared<og::XXL>(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    // Attempt to solve the problem within the given time
    ob::PlannerStatus solved = ss->solve(0.5);

    if (solved)
    {
        show_results(nullptr);
    }

    return;
}

void algos::stride(int start_cell_index, int goal_cell_index, consolidated_result* result) {

    // Specify the start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = start_cell_index % map_width; // Set the start coordinates
    start[1] = start_cell_index / map_width;
    goal[0] = goal_cell_index % map_width; // Set the goal coordinates
    goal[1] = goal_cell_index / map_width;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal);

    // Create the planner (STRIDE*)
    auto planner(std::make_shared<og::STRIDE>(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    // Attempt to solve the problem within the given time
    ob::PlannerStatus solved = ss->solve(0.5);

    if (solved)
    {
        show_results(&result->stride);
    }

    return;
}

void algos::bitStar(int start_cell_index, int goal_cell_index, consolidated_result* result) {

    // Specify the start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = start_cell_index % map_width; // Set the start coordinates
    start[1] = start_cell_index / map_width;
    goal[0] = goal_cell_index % map_width; // Set the goal coordinates
    goal[1] = goal_cell_index / map_width;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal);

    // Create the planner (BITstar*)
    auto planner(std::make_shared<og::BITstar>(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    // Attempt to solve the problem within the given time
    ob::PlannerStatus solved = ss->solve(0.5);

    if (solved)
    {
        show_results(nullptr);
    }

    return;
}

void algos::rrtStar(int start_cell_index, int goal_cell_index, consolidated_result* result) {

    // Specify the start and goal states
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = start_cell_index % map_width; // Set the start coordinates
    start[1] = start_cell_index / map_width;
    goal[0] = goal_cell_index % map_width; // Set the goal coordinates
    goal[1] = goal_cell_index / map_width;

    // Set the start and goal states
    ss->setStartAndGoalStates(start, goal);

    // Create the planner (RRT*)
    auto planner(std::make_shared<og::RRTstar>(ss->getSpaceInformation()));
    ss->setPlanner(planner);

    // Attempt to solve the problem within the given time
    ob::PlannerStatus solved = ss->solve(0.5);

    if (solved)
    {
        show_results(&result->rrtStar);
    }

    return;
}

void algos::show_results(algo_result* result) {
    // Retrieve the path
    og::PathGeometric path = ss->getSolutionPath();

    // Print the path
    //path.printAsMatrix(std::cout);

    for (std::size_t i = 1; i < path.getStateCount(); ++i)
    {
        const ob::State* state = path.getState(i);
        const auto* coordinates = state->as<ob::RealVectorStateSpace::StateType>()->values;

        // Access the x and y coordinates
        float x1 = static_cast<float>(coordinates[0]);
        float y1 = static_cast<float>(coordinates[1]);

        state = path.getState(i - 1);
        coordinates = state->as<ob::RealVectorStateSpace::StateType>()->values;

        // Access the x and y coordinates
        float x2 = static_cast<float>(coordinates[0]);
        float y2 = static_cast<float>(coordinates[1]);

        if (i == 1)
            paths->push_back(Path(x1, y1, x2, y2, Color(1.0, 0.0, 0.0)));
        else if (i == path.getStateCount() - 1)
            paths->push_back(Path(x1, y1, x2, y2, Color(0.0, 1.0, 0.0)));
        else
            paths->push_back(Path(x1, y1, x2, y2, Color(0.0, 0.0, 0.0)));
    }

    std::vector<std::vector<float>> solved_path;
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        const ob::State* state = path.getState(i);
        const auto* coordinates = state->as<ob::RealVectorStateSpace::StateType>()->values;

        // Access the x and y coordinates
        float x = static_cast<float>(coordinates[0]);
        float y = static_cast<float>(coordinates[1]);
        int index = solved_path.size();
        solved_path.push_back(std::vector<float>());
        solved_path.at(index).push_back(x);
        solved_path.at(index).push_back(y);
    }

    *result = algo_result(path_length(solved_path), 
        ss->getLastPlanComputationTime(), 
        stringify(solved_path));

    // get computation time info
    std::cout << "time info : " << ss->getLastPlanComputationTime() << std::endl;
}

float algos::path_length(std::vector<std::vector<float>> path) {
    float length = 0.0f;
    for (int i = 1; i < path.size(); i++) {
        length += distance(path.at(i - 1)[0], path.at(i - 1)[1], path.at(i)[0], path.at(i)[1]);
    }
    return length;
}

float algos::distance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x1 - x2, 2.0) + pow(y1 - y2, 2.0));
}

std::string algos::stringify(std::vector<std::vector<float>> path) {
    std::string order = "[";
    for (int i = 0; i < path.size(); i++) {
        order = order + " {" + std::to_string(path.at(i)[0]) + ", " + std::to_string(path.at(i)[1]) + "}, ";
    }
    order = order + "]";
    return order;
}