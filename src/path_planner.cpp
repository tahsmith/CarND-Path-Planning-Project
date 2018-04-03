//
// Created by Timothy Smith on 2/4/18.
//

#include <utility>
#include <cmath>

#include "path_planner.hpp"
#include "utilities.hpp"

PathPlanner::PathPlanner(MapData mapData) :
    mapData{std::move(mapData)}
{}

Path PathPlanner::PlanPath()
{
    Path path{};

    double d = 6;
    double dist_inc = 0.5;
    for (int i = 0; i < 50; i++)
    {
        double s = state.s + (i + 1) * dist_inc;
        auto xy = getXY(s, d, this->mapData.waypoints_s,
                        this->mapData.waypoints_x, this->mapData.waypoints_y);
        path.x.push_back(xy[0]);
        path.y.push_back(xy[1]);
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

