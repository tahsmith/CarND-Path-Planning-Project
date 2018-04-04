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
    double t_final = 0.5;
    double s_final = state.s + speed * t_final;
    double d_final = 6;

    auto xy = getXY(s_final, d_final, this->mapData.waypoints_s,
                    this->mapData.waypoints_x, this->mapData.waypoints_y);

    auto last_waypoint_i = ClosestWaypoint(xy[0], xy[1], mapData.waypoints_x,
                                           mapData.waypoints_y);

    double vx_initial;
    double vy_initial;
    if (!previousPath.x.empty())
    {
        vx_initial = (state.x - previousPath.x.back()) / dt;
        vx_initial = fmin(speed, vx_initial);
    }
    else
    {
        vx_initial = 0;
    }
    if (!previousPath.y.empty())
    {
        vy_initial = (state.y - previousPath.y.back()) / dt;
        vy_initial = fmin(speed, vy_initial);

    }
    else
    {
        vy_initial = 0;
    }

    auto vx_final = -speed * mapData.waypoints_dy[last_waypoint_i];
    auto vy_final = speed * mapData.waypoints_dx[last_waypoint_i];

    auto x_curve = JerkMinimalTrajectory({state.x, vx_initial, 0},
                                         {xy[0], vx_final, 0},
                                         t_final);

    auto y_curve = JerkMinimalTrajectory({state.y, vy_initial, 0},
                                         {xy[1], vy_final, 0},
                                         t_final);

    auto first_waypoint_i = ClosestWaypoint(state.x, state.y,
                                            mapData.waypoints_x,
                                            mapData.waypoints_y);
    double vx = -speed * mapData.waypoints_dy[last_waypoint_i];
    double vy = speed * mapData.waypoints_dx[last_waypoint_i];
    auto n_points = lround(t_final / dt);

    for (auto i = std::min(previousPath.x.size(), 2UL); i > 0; i--)
    {
        path.x.push_back(previousPath.x[previousPath.x.size() - i]);
        path.y.push_back(previousPath.y[previousPath.x.size() - i]);
    }
    for (int i = 1; i < n_points; i++)
    {
        double t = i * dt;
//        double x = x_curve.Evaluate(t);
//        double y = y_curve.Evaluate(t);
        double x = state.x + t * vx;
        double y = state.y + t * vy;

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

