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
    double pos_x;
    double pos_y;
    double angle;
    int path_size = previousPath.x.size();

    for (int i = 0; i < path_size; i++)
    {
        path.x.push_back(previousPath.x[i]);
        path.y.push_back(previousPath.y[i]);
    }

    if (path_size == 0)
    {
        pos_x = state.x;
        pos_y = state.y;
        angle = deg2rad(state.yaw);
    }
    else
    {
        pos_x = previousPath.x[path_size - 1];
        pos_y = previousPath.y[path_size - 1];

        double pos_x2 = previousPath.x[path_size - 2];
        double pos_y2 = previousPath.y[path_size - 2];
        angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
    }

    double dist_inc = 0.5;
    for (int i = 0; i < 50 - path_size; i++)
    {
        path.x
            .push_back(
                pos_x + (dist_inc) * cos(angle + (i + 1) * (pi() / 100)));
        path.y
            .push_back(
                pos_y + (dist_inc) * sin(angle + (i + 1) * (pi() / 100)));
        pos_x += (dist_inc) * cos(angle + (i + 1) * (pi() / 100));
        pos_y += (dist_inc) * sin(angle + (i + 1) * (pi() / 100));
    }
    return path;
}

void PathPlanner::UpdateLocalisation(State state)
{
    this->state = state;
}

void PathPlanner::UpdateSensorFusion(SensorFusionData)
{
    this->sensorFusionData = std::move(sensorFusionData);
}

void PathPlanner::UpdateHistory(Path previousPath)
{
    this->previousPath = std::move(previousPath);
}

