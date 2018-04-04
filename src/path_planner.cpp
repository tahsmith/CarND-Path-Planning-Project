//
// Created by Timothy Smith on 2/4/18.
//

#include <utility>
#include <cmath>

#include "path_planner.hpp"
#include "utilities.hpp"
#include "jerk_minimal_trajectory.hpp"

PathPlanner::PathPlanner(double dt, MapData mapData) :
    dt(dt),
    mapData{std::move(mapData)}
{}

Path PathPlanner::PlanPath()
{
    Path path{};

    double speed = 16;  // m s^-1

    double x_initial;
    if(!previousPath.x.empty()) {
        x_initial = previousPath.x[0];
//        x_initial = state.x;
    }
    else {
        x_initial = state.x;
    }

    double y_initial;
    if(!previousPath.y.empty()) {
        y_initial = previousPath.y[0];
//        y_initial = state.y;

    }
    else {
        y_initial = state.y;
    }
    double t_final = 2;
    double s_final = state.s + speed * t_final;
    double d_final = 6;

    auto xy_final = getXY(s_final, d_final, this->mapData.waypoints_s,
                    this->mapData.waypoints_x, this->mapData.waypoints_y);

    double x_final = xy_final[0];
    double y_final = xy_final[1];

    auto first_waypoint_i = ClosestWaypoint(x_initial, y_initial,
                                            mapData.waypoints_x,
                                            mapData.waypoints_y);
    auto last_waypoint_i = ClosestWaypoint(x_final, y_final, mapData.waypoints_x,
                                           mapData.waypoints_y);

//    double vx_initial;
//    double vy_initial;
//    if (!previousPath.x.empty())
//    {
//        vx_initial = (state.x - previousPath.x.back()) / dt;
//        vx_initial = fmin(speed, vx_initial);
//    }
//    else
//    {
//        vx_initial = 0;
//    }
//    if (!previousPath.y.empty())
//    {
//        vy_initial = (state.y - previousPath.y.back()) / dt;
//        vy_initial = fmin(speed, vy_initial);
//
//    }
//    else
//    {
//        vy_initial = 0;
//    }

    double vx_initial = -speed * mapData.waypoints_dy[first_waypoint_i];
    double vy_initial = speed * mapData.waypoints_dx[first_waypoint_i];

    auto vx_final = -speed * mapData.waypoints_dy[last_waypoint_i];
    auto vy_final = speed * mapData.waypoints_dx[last_waypoint_i];

    auto x_curve = JerkMinimalTrajectory({x_initial, vx_initial, 0},
                                         {x_final, vx_final, 0},
                                         t_final);

    auto y_curve = JerkMinimalTrajectory({y_initial, vy_initial, 0},
                                         {y_final, vy_final, 0},
                                         t_final);

    auto n_points = lround(floor(t_final / dt));

//    for (auto i = std::min(previousPath.x.size(), 2UL); i > 0; i--)
//    {
//        path.x.push_back(previousPath.x[previousPath.x.size() - i]);
//        path.y.push_back(previousPath.y[previousPath.x.size() - i]);
//    }
    for (int i = 1; i < n_points; i++)
    {
        double t = i * dt;
        double x = x_curve.Evaluate(t);
        double y = y_curve.Evaluate(t);
//        double x = state.x + t * vx;
//        double y = state.y + t * vy;

        path.x.push_back(x);
        path.y.push_back(y);
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

