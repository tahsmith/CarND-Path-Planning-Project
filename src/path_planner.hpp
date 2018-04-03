//
// Created by Timothy Smith on 2/4/18.
//

#ifndef PATH_PLANNING_PATH_PLANNER_HPP
#define PATH_PLANNING_PATH_PLANNER_HPP

#include <vector>

#include "map_data.hpp"
#include "sensor_fusion_data.hpp"


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
    explicit PathPlanner(double dt, MapData mapData);

    void UpdateLocalisation(State state);

    void UpdateHistory(Path previousPath);

    void UpdateSensorFusion(SensorFusionData);

    Path PlanPath();


private:
    double dt;
    MapData mapData;
    State state;
    SensorFusionData sensorFusionData;
    Path previousPath;
};


#endif //PATH_PLANNING_PATH_PLANNER_HPP
