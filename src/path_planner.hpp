//
// Created by Timothy Smith on 2/4/18.
//

#ifndef PATH_PLANNING_PATH_PLANNER_HPP
#define PATH_PLANNING_PATH_PLANNER_HPP

#include <vector>

#include "map_data.hpp"


class Path
{
public:
    std::vector<double> x;
    std::vector<double> y;
};


class State
{
public:
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
};

class PathPlanner
{
public:
    PathPlanner(MapData mapData);

    void UpdateLocalisation(State state);

    Path PlanPath();


private:
    MapData mapData;
    State state;
};


#endif //PATH_PLANNING_PATH_PLANNER_HPP
