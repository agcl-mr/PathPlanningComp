#ifndef PRE_BUILT_ALGO_CLASS_H
#define PRE_BUILT_ALGO_CLASS_H

#include <iostream>
#include "..\Header Files\mcc.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
//#include <ompl/geometric/planners/rrt/Dijkstra.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <memory> // Add this line for the smart pointer

namespace ob = ompl::base;
namespace og = ompl::geometric;

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

class MyValidityChecker : public ompl::base::StateValidityChecker
{
public:
    MyValidityChecker(const ompl::base::SpaceInformationPtr& si) : ompl::base::StateValidityChecker(si) {}

    bool isValid(const ompl::base::State* state) const override
    {
        // Define your validity checking function here.
    }
};

class algos {
public:
    std::vector<std::vector<int>> map;
    int map_width, map_height;

	algos(void);

    void updateObstacleMap(std::vector<Node>* node, int GRID_WIDTH, int GRID_HEIGHT, std::vector<Path>* paths);

	void dijkstras(void);

	void abitStar(void);

    bool isStateValid(const ompl::base::State* state)
    {
        // Define your state validity checking function here
        // Return true if the state is valid, false otherwise
        return true;
    }

    void rrtStar(int start_cell_index, int goal_cell_index);

private:
    og::SimpleSetupPtr ss;
    ob::StateSpacePtr space;
    std::vector<Path>* paths;

    void show_results(void);
};

#endif