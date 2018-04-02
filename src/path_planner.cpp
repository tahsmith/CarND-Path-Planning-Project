//
// Created by Timothy Smith on 2/4/18.
//

#include <utility>
#include <cmath>

#include "path_planner.hpp"
#include "map_data.hpp"
#include "utilities.hpp"

PathPlanner::PathPlanner(MapData mapData) :
    mapData{std::move(mapData)}
{}

Path PathPlanner::PlanPath()
{
    Path path{};
    double dist_inc = 0.5;
    for (int i = 0; i < 50; i++)
    {
        path.x.push_back(
            state.x + (dist_inc * i) * cos(deg2rad(state.yaw)));
        path.y.push_back(
            state.y + (dist_inc * i) * sin(deg2rad(state.yaw)));
    }
    return path;
}

void PathPlanner::UpdateLocalisation(State state)
{
    this->state = state;
}
