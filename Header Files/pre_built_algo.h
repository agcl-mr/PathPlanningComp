#ifndef PRE_BUILT_ALGO_CLASS_H
#define PRE_BUILT_ALGO_CLASS_H

#include <iostream>
#include <string>
#include "..\Header Files\mcc.h"
#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/rlrt/RLRT.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/xxl/XXL.h>

#include <memory> // Add this line for the smart pointer

namespace ob = ompl::base;
namespace og = ompl::geometric;

/*/ Custom projection evaluator for RealVectorStateSpace
class MyProjectionEvaluator : public ob::ProjectionEvaluator
{
public:
    MyProjectionEvaluator(const ob::StateSpacePtr& space) : ob::ProjectionEvaluator(space)
    {
        setDimension(2);
    }

    // Implement the project() function to project a state to a subset of its dimensions
    void project(const ob::State* state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Extract the values from the state and set the projection vector
        const auto* realState = state->as<ob::RealVectorStateSpace::StateType>();
        projection[0] = realState->values[0];
        projection[1] = realState->values[1];
    }
};*/

// Define a custom state validity checker
class MyStateValidityChecker : public ob::StateValidityChecker
{
public:
    MyStateValidityChecker(const std::vector<std::vector<int>>& obstacleMap, const ob::SpaceInformationPtr& si)
        : ob::StateValidityChecker(si), obstacleMap_(obstacleMap)
    {
        // Get the map dimensions
        mapWidth_ = obstacleMap_[0].size();
        mapHeight_ = obstacleMap_.size();
    }

    // Override the isValid() function
    bool isValid(const ob::State* state) const override
    {
        // Retrieve the coordinates from the state
        const double* coordinates = state->as<ob::RealVectorStateSpace::StateType>()->values;

        // Convert the coordinates to map indices
        int x = static_cast<int>(coordinates[0] * mapResolution_);
        int y = static_cast<int>(coordinates[1] * mapResolution_);

        // Check if the coordinates are within the map bounds
        if (x < 0 || x >= mapWidth_ || y < 0 || y >= mapHeight_)
        {
            return false;
        }
        // Check if the corresponding cell in the obstacle map is an obstacle
        return (obstacleMap_[y][x] == 0);
    }

private:
    const std::vector<std::vector<int>>& obstacleMap_;
    int mapWidth_ = 0;      // Width of the obstacle map
    int mapHeight_ = 0;     // Height of the obstacle map
    double mapResolution_ = 1.0;  // Resolution of the obstacle map
};

class algos {
public:
    std::vector<std::vector<int>> map;
    int map_width, map_height;

	algos(void);

    void updateObstacleMap(std::vector<Node>* node, int GRID_WIDTH, int GRID_HEIGHT, std::vector<Path>* paths);

	void dijkstras(void);

    void abitStar(void);

    void abitStar2(int start_cell_index, int goal_cell_index, consolidated_result* result);

    void prm(int start_cell_index, int goal_cell_index, consolidated_result* result);

    void fmt(int start_cell_index, int goal_cell_index, consolidated_result* result);

    void est(int start_cell_index, int goal_cell_index, consolidated_result* result);

    void rlrt(int start_cell_index, int goal_cell_index, consolidated_result* result);

    void sst(int start_cell_index, int goal_cell_index, consolidated_result* result);

    void xxl(int start_cell_index, int goal_cell_index, consolidated_result* result);

    void stride(int start_cell_index, int goal_cell_index, consolidated_result* result);

    void bitStar(int start_cell_index, int goal_cell_index, consolidated_result* result);

    void rrtStar(int start_cell_index, int goal_cell_index, consolidated_result* result);

private:
    og::SimpleSetupPtr ss;
    ob::StateSpacePtr space;
    std::vector<Path>* paths;

    void show_results(algo_result* result);

    float path_length(std::vector<std::vector<float>> path);

    float distance(float x1, float y1, float x2, float y2);

    std::string stringify(std::vector<std::vector<float>> path);
};

#endif