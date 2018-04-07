//
// Created by Timothy Smith on 2/4/18.
//

#ifndef PATH_PLANNING_PATH_PLANNER_HPP
#define PATH_PLANNING_PATH_PLANNER_HPP

#include <vector>

#include "map_data.hpp"
#include "sensor_fusion_data.hpp"
#include "jerk_minimal_trajectory.hpp"


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
    PathPlanner(double dt, MapData mapData);

    void UpdateLocalisation(State state);

    void UpdateHistory(Path previousPath);

    void UpdateSensorFusion(SensorFusionData);

    Path PlanPath();


private:
    const double speed_limit = 21.90496;  // m s^-1 ~= 49 miles / hr
    const double dt;
    const MapData mapData;
    State vehicle_state;
    uint8_t planner_state;
    double speed_target;
    double d_target;
    SensorFusionData sensorFusionData;
    Path previousPath;

    Path
    GenerateTrajectory(double t_final, double s_final, double d_final,
                           double speed_final) const;

    Path GenerateTrajectoryForState(uint8_t state) const;
    double CostForTrajectory(uint8_t state, const Path&) const;
    size_t FindCarToFollow() const;
};


#endif //PATH_PLANNING_PATH_PLANNER_HPP
