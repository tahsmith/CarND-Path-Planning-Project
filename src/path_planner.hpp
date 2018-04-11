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
    std::vector<double> vx;
    std::vector<double> vy;
};

class Plan
{
public:
    Path path;
    long lane_current;
    long lane_target;
    double speed_target;
};


class VehicleState
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

    void UpdateLocalisation(VehicleState state);

    void UpdateHistory(Path previousPath);

    void UpdateSensorFusion(SensorFusionData);

    Path PlanPath();


    const double speed_limit = 22.35 * 0.90;  //  22.35m s^-1 ~= 50 miles / hr
    const double dt;
    const MapData mapData;
    VehicleState vehicle_state;
    uint8_t planner_state;
    Plan current_plan;
    long lane_actual;
    SensorFusionData sensorFusionData;

    Path previousPath;

    Path
    GenerateTrajectory(double t_final, double s_final, double d_final,
                           double speed_final) const;
    Plan GeneratePlanForState(uint8_t state) const;
    double CostForTrajectory(const Plan&) const;

    size_t FindCarToFollow(long lane) const;

    double SafeSpeedForLane(long lane) const;
    double CarPotential(double x, double y,
                        double vx, double vy,
                        double car_x, double car_y,
                        double car_vx, double car_vy) const;

    double SoftCarPotential(double x, double y,
                            double vx, double vy,
                            double car_x, double car_y,
                            double car_vx, double car_vy) const;
    double CarAvoidanceCostPerCar(const Path& path, size_t i) const;
    double CarAvoidanceCost(const Path& path) const;


    Path GenerateTrajectory(double t_final, double s_final, double d_final,
                            double speed_final, double x_initial, double y_initial,
                            double vx_initial, double vy_initial, double ax_initial,
                            double ay_initial) const;
};


#endif //PATH_PLANNING_PATH_PLANNER_HPP
