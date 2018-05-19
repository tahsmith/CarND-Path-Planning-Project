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
//    std::vector<double> ax;
//    std::vector<double> ay;
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
    PathPlanner(MapData mapData);
    void UpdateLocalisation(VehicleState state);
    void UpdateHistory(Path previousPath);
    void UpdateSensorFusion(SensorFusionData);
    Path PlanPath();
    Path FinalTrajectory(const Path& previous_path, const Plan& plan);
    Plan GeneratePlanForState(uint8_t state) const;
    Path
    GenerateTrajectoryFromCurrent(double lane_target, double speed_target) const;
    typedef std::map<std::string, std::tuple<double, double, double> > CostDebugInfo;
    double CostForTrajectory(const Plan& plan, CostDebugInfo& debug_info) const;
    void PrintDebugInfo(const CostDebugInfo&);
    size_t FindCarToFollow(long lane) const;
    double SafeSpeedForLane(long lane) const;
    void UpdateCarPaths();
    double CarPotential(double x, double y, double car_x, double car_y,
                            double car_vx, double car_vy) const;
    double CarAvoidanceCostPerCar(const Path& path, size_t i) const;
    double CarAvoidanceCost(const Path& path) const;
    double SoftCarPotential(double x, double y, double car_x, double car_y,
                                double car_vx, double car_vy) const;
    double SoftCarAvoidanceCostPerCar(const Path& path, size_t i) const;
    double SoftCarAvoidanceCost(const Path& path) const;
    Path GenerateTrajectory(double s_initial, double s_final,
                                double d_initial, double d_final,
                                double speed_initial, double speed_final,
                                double vy_initial, double ax_initial,
                                double ay_initial) const;
    Path InterpolatePath(double x_initial, double x_final, double y_initial,
                             double y_final, double vx_initial, double vx_final,
                             double vy_initial, double vy_final,
                             double ax_initial, double ax_final,
                             double ay_initial, double ay_final) const;

    MapData map_data;
    VehicleState vehicle_state;
    uint8_t planner_state;
    Plan current_plan;
    long lane_actual;
    SensorFusionData sensor_fusion_data;

    std::vector<Path> car_paths;

    Path previous_path;
};


#endif //PATH_PLANNING_PATH_PLANNER_HPP
