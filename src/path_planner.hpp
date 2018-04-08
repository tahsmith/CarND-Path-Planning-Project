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

class Plan
{
public:
    Path path;
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


private:
    const double speed_limit = 22.35 * 0.95;  //  22.35m s^-1 ~= 40 miles / hr
    const double dt;
    const MapData mapData;
    VehicleState vehicle_state;
    uint8_t planner_state;
    double speed_target;
    long lane_current;
    size_t follow_id;
    SensorFusionData sensorFusionData;

    Path previousPath;

    Path
    GenerateTrajectory(double t_final, double s_final, double d_final,
                           double speed_final) const;
    Plan GeneratePlanForState(uint8_t state) const;
    double CostForTrajectory(uint8_t state, const Plan&) const;

    size_t FindCarToFollow(size_t lane) const;

    double SafeSpeedForLane(size_t lane) const;
    double CarPotential(double x, double y,
                        double car_x, double car_y,
                        double car_vx, double car_vy) const;
    double CarAvoidanceCostPerCar(const Path& path, size_t i) const;
    double CarAvoidanceCost(const Path& path) const;


    void GenerateTrajectory(double t_final, double s_final, double d_final,
                            double speed_final, double x_initial, double y_initial,
                            double vx_initial, double vy_initial, double ax_initial,
                            double ay_initial, Polynomial& x_curve,
                            Polynomial& y_curve) const;
};


#endif //PATH_PLANNING_PATH_PLANNER_HPP
