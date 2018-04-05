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

    double speed_limit = 21.90496;  // m s^-1 ~= 49 miles / hr

    double x_initial;
    if (!previousPath.x.empty())
    {
        x_initial = previousPath.x.front();
    }
    else
    {
        x_initial = state.x;
    }

    double y_initial;
    if (!previousPath.y.empty())
    {
        y_initial = previousPath.y.front();
    }
    else
    {
        y_initial = state.y;
    }

    double vx_initial;
    double vy_initial;
    double ax_initial;
    double ay_initial;
    if (previousPath.x.size() > 2)
    {
        vx_initial = (previousPath.x[1] - previousPath.x[0]) / dt;
        double vx2_initial =
            (previousPath.x[2] - previousPath.x[1]) / dt;
        ax_initial = (vx2_initial - vx_initial) / dt;
    }
    else
    {
        vx_initial = 0;
        ax_initial = 0;
    }
    if (previousPath.y.size() > 2)
    {
        vy_initial = (previousPath.y[1] - previousPath.y[0]) / dt;
        double vy2_initial =
            (previousPath.y[2] - previousPath.y[1]) / dt;
        ay_initial = (vy2_initial - vy_initial) / dt;
    }
    else
    {
        vy_initial = 0;
        ay_initial = 0;
    }

    double t_final = 1.0;
    double s_final = state.s + speed_limit * t_final;
    double d_final = 06;

    auto xy_final = getXY(s_final, d_final, this->mapData.waypoints_s,
                          this->mapData.waypoints_x, this->mapData.waypoints_y);

    double x_final = xy_final[0];
    double y_final = xy_final[1];

    auto last_waypoint_i = ClosestWaypoint(x_final, y_final,
                                           mapData.waypoints_x,
                                           mapData.waypoints_y);

    auto vx_final = -speed_limit * mapData.waypoints_dy[last_waypoint_i];
    auto vy_final = speed_limit * mapData.waypoints_dx[last_waypoint_i];

    auto x_curve = JerkMinimalTrajectory({x_initial, vx_initial, ax_initial},
                                         {x_final, vx_final, ay_initial},
                                         t_final);

    auto y_curve = JerkMinimalTrajectory({y_initial, vy_initial, 0},
                                         {y_final, vy_final, 0},
                                         t_final);

    auto n_points = lround(floor(t_final / dt));

    auto overlap = std::min(previousPath.x.size(), 1UL);
    for (auto i = 0; i < overlap; ++i)
    {
        path.x.push_back(previousPath.x[i]);
        path.y.push_back(previousPath.y[i]);
    }
    for (int i = overlap; i < n_points; i++)
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

