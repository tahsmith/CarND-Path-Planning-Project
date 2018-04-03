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

    double speed = 0.8;  // m s^-1
    double t_final = 5;
    double s_final = state.s + speed * t_final;
    double d_final = 6;

    auto xy = getXY(s_final, d_final, this->mapData.waypoints_s,
                    this->mapData.waypoints_x, this->mapData.waypoints_y);

    auto last_waypoint_i = ClosestWaypoint(xy[0], xy[1], mapData.waypoints_x,
                                           mapData.waypoints_y);

    auto vx_initial = state.speed * cos(state.yaw);
    auto vy_initial = state.speed * sin(state.yaw);

    auto vx_final = -speed * mapData.waypoints_dy[last_waypoint_i];
    auto vy_final = speed * mapData.waypoints_dx[last_waypoint_i];

    auto x_curve = JerkMinimalTrajectory({state.x, vx_initial, 0},
                                         {xy[0], vx_final, 0},
                                         t_final);

    auto y_curve = JerkMinimalTrajectory({state.y, vy_initial, 0},
                                         {xy[1], vy_final, 0},
                                         t_final);

    auto n_points = lround(t_final / dt);
    for (int i = 0; i < n_points; i++)
    {
        double t = i * dt;
        double x = x_curve.Evaluate(t);
        double y = y_curve.Evaluate(t);

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

