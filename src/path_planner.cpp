//
// Created by Timothy Smith on 2/4/18.
//

//#define NDEBUG
//#define DEBUG_STATE
#define DEBUG_COST
//#define DEBUG_TRAJ

#include <utility>
#include <vector>
#include <cmath>
#include <iostream>
#include <numeric>
#include <map>
#include <thread>

#include "path_planner.hpp"
#include "utilities.hpp"
#include "spline.h"

using std::vector;
using std::move;
using std::numeric_limits;
using std::tie;
using std::make_tuple;
using std::forward_as_tuple;
using std::map;
using std::min;
using std::max;
using std::tuple;
using std::function;
using std::get;
using std::strcmp;

// Hidden constants
namespace
{
    const size_t n_states = 6;
    const char* states[n_states] = {
        "START",
        "CAUTION",
        "CRUISE",
        "CHANGE_LEFT",
        "CHANGE_RIGHT"
    };


    const int transitions[n_states][n_states]{
/* from \ to    *  START CAUTION  CRUISE CHANGE_LEFT CHANGE_RIGHT */
/* START        */ {0,   1,       0,     0,          0},
/* CAUTION      */ {0,   1,       1,     0,          0},
/* CRUISE       */ {0,   1,       1,     1,          1},
/* CHANGE_LEFT  */ {0,   0,       1,     1,          0},
/* CHANGE_RIGHT */ {0,   0,       1,     0,          1}
    };

    const double LANE_WIDTH = 4.0;
    const double FOLLOW_DISTANCE = 35.0;
    const double CAR_LENGTH = 7.0;
    const double CAR_WIDTH = 3.5;

    const double hard_speed_limit = 22.35;  //  22.35m s^-1 ~= 50 miles / hr
    const double speed_limit = hard_speed_limit * 0.90;
    const double caution_margin = 0.75;
    const double hard_acc_limit = 10.0;
    const double acc_limit = hard_acc_limit * 0.90;
    const double planning_dt = 0.6;
    const size_t planning_steps = 5;
    const double planning_time = planning_steps * planning_dt;
    const double control_dt = 0.02;
    const size_t control_steps = lround(planning_time / control_dt);
    const size_t final_path_overlap = 20;
}

void print_path(const Path& path)
{
    for (size_t i = 0; i < path.x.size(); ++i)
    {
        printf("[%8.1f, %8.1f] ", path.x[i], path.y[i]);
    }
    printf("\n");
}

Path diff(const Path& path)
{
    return {diff(path.x, 1), diff(path.y, 1)};
}

vector<uint8_t> SuccessorStates(size_t state)
{
    vector<uint8_t> states{};
    for (uint8_t i = 0; i < n_states; ++i)
    {
        if (transitions[state][i])
        {
            states.push_back(i);
        }
    }
    return states;
}

PathPlanner::PathPlanner(MapData mapData) :
    map_data{std::move(mapData)},
    current_plan{{}, 1, 1, 0},
    planner_state{0}
{
    this->map_data.PrepareInterpolation();
}

long d_to_lane(double d)
{
    return lround(d / LANE_WIDTH - 0.5);
}

Path PathPlanner::PlanPath()
{
    lane_actual = d_to_lane(vehicle_state.d);

    if (planner_state == 0)
    {
        current_plan.lane_current = lane_actual;
        current_plan.lane_target = lane_actual;
        current_plan.speed_target = 0;
    }

    #ifdef DEBUG_STATE
    printf("Planner state: %s\n"
           " lane_actual  %1ld\n"
           " lane_current %1ld\n"
           " lane_target  %1ld\n"
           " speed_target %4.1f\n",
                 states[planner_state],
                 lane_actual,
                 current_plan.lane_current,
                 current_plan.lane_target,
                 current_plan.speed_target
    );

    printf("Vehicle state:\n"
           " s     %f\n"
           " d     %f\n"
           " x     %f\n"
           " y     %f\n"
           " speed %f\n",
           vehicle_state.s,
           vehicle_state.d,
           vehicle_state.x,
           vehicle_state.y,
           vehicle_state.speed
    );

    {
        double x,y,dx,dy;
        tie(x, y) = map_data.InterpolateRoadCoords(vehicle_state.s, vehicle_state.d);
        tie(dx, dy) = map_data.InterpolateRoadTangent(vehicle_state.s);
        printf("Road state:\n"
               " x  %f\n"
               " y  %f\n"
               " dx %f\n"
               " dy %f\n",
               x,
               y,
               dx,
               dy
        );
    }
    #endif

    auto candidate_states = SuccessorStates(planner_state);
    std::vector<double> cost_list(n_states,
                                  std::numeric_limits<float>::infinity());
    vector<Plan> plan_list(n_states);
    std::vector<CostDebugInfo> cost_debug_info_list(n_states);
    for (uint8_t candidate_state : candidate_states)
    {
        auto plan = GeneratePlanForState(candidate_state);
        if (!plan.path.x.empty())
        {
            cost_list[candidate_state] = CostForTrajectory(
                plan,
                cost_debug_info_list[candidate_state]
            );
            plan_list[candidate_state] = move(plan);
        }
    }

    double minimum = cost_list[0];
    uint8_t minimum_i = 0;
    for (uint8_t i = 1; i < cost_list.size(); ++i)
    {
        if (cost_list[i] < minimum)
        {
            minimum = cost_list[i];
            minimum_i = i;
        }
    }
    auto next_state = minimum_i;

    #ifdef DEBUG_COST
    if (next_state != planner_state)
    {
        printf("== Changing states\n");
        printf("Planner state: %s"
               " lane_actual  %1ld"
               " lane_current %1ld"
               " lane_target  %1ld"
               " speed_target %4.1f\n",
               states[planner_state],
               lane_actual,
               current_plan.lane_current,
               current_plan.lane_target,
               current_plan.speed_target
        );
        printf(" = Leaving  %s cost %6.1g\n", states[planner_state],
               cost_list[planner_state]);
        PrintDebugInfo(cost_debug_info_list[planner_state]);
        printf(" = Entering %s cost %6.1g\n", states[next_state],
               cost_list[next_state]);
        PrintDebugInfo(cost_debug_info_list[next_state]);
    }
    #endif

    planner_state = next_state;
    current_plan = plan_list[minimum_i];

    return FinalTrajectory(previous_path, current_plan);
}

Path PathPlanner::FinalTrajectory(const Path& previous_path, const Plan& plan)
{
    Path waypoints;
    size_t overlap = min(final_path_overlap, previous_path.x.size());
    size_t plan_skip = lround(overlap * control_dt / planning_dt) + 1;

    vector<double> t;
    double t_back = 0;

    if (overlap == 0)
    {
        waypoints.x.push_back(vehicle_state.x);
        waypoints.y.push_back(vehicle_state.y);
        t.push_back(t_back);
        t_back += control_dt;
    }
    else
    {
        for (size_t i = 0; i < overlap; ++i)
        {
            waypoints.x.push_back(previous_path.x[i]);
            waypoints.y.push_back(previous_path.y[i]);
            t.push_back(t_back);
            t_back += control_dt;
        }
    }

    {
        double vx, vy;
        auto n = waypoints.x.size();
        if (n < 2) {
            vx = cos(vehicle_state.yaw);
            vy = sin(vehicle_state.yaw);
        }
        else
        {
            vx = waypoints.x[n - 2] - waypoints.x[n - 1];
            vy = waypoints.y[n - 2] - waypoints.y[n - 1];
        }

//        double x0 = waypoints.x[n - 1];
//        double y0 = waypoints.y[n - 1];
        for (size_t i = plan_skip; i < plan.path.x.size(); ++i)
        {
            double x, y;
            tie(x, y) = map_data.InterpolateRoadCoords(plan.path.x[i],
                                                       plan.path.y[i]);
//            double dot = (x - x0) * vx + (y - y0) * vy;
            double dot = 1;
            if (dot > 0)
            {
                waypoints.x.push_back(x);
                waypoints.y.push_back(y);
                t.push_back(i * planning_dt);
            }
        }
    }

    assert(t.size() == waypoints.x.size());

    tk::spline x;
    x.set_points(t, waypoints.x);

    tk::spline y;
    y.set_points(t, waypoints.y);

    Path final_traj;

    for (size_t i = 0; i < control_steps; ++i)
    {
        final_traj.x.push_back(x(control_dt * i));
        final_traj.y.push_back(y(control_dt * i));
    }
    assert(final_traj.x.size() == final_traj.y.size());
    assert(final_traj.x.size() == control_steps);


    #ifdef DEBUG_TRAJ
    printf("S-D Waypoints\n");
    print_path(plan.path);
    print_path(diff(plan.path));

    printf("X-Y Waypoints\n");
    print_path(waypoints);
    print_path(diff(waypoints));

    printf("X-Y Final\n");
    print_path(final_traj);

    // Sanity checks
    auto vx = diff(final_traj.x, planning_dt);
    auto vy = diff(final_traj.y, planning_dt);
    auto ax = diff(vx, planning_dt);
    auto ay = diff(vy, planning_dt);
    auto jx = diff(ax, planning_dt);
    auto jy = diff(ay, planning_dt);

//    assert(all_of_list(vx, less_than(hard_speed_limit)));
//    assert(all_of_list(vy, less_than(hard_speed_limit)));
//    assert(all_of_list(ax, less_than(acc_limit)));
//    assert(all_of_list(ay, less_than(acc_limit)));
    #endif
    
    return final_traj;
}

Plan PathPlanner::GeneratePlanForState(uint8_t state) const
{
    long lane_target;
    long lane_current;
    double speed_final;
    if (state == 1)
    {
        assert(strcmp(states[state], "CAUTION") == 0);
        lane_current = lane_actual;
        lane_target = lane_actual;
        speed_final = caution_margin * SafeSpeedForLane(current_plan.lane_target);
    }
    else if (state == 2)
    {
        assert(strcmp(states[state], "CRUISE") == 0);
        lane_current = lane_actual;
        lane_target = lane_actual;
        speed_final = SafeSpeedForLane(current_plan.lane_target);
    }
    else if (state == 3)
    {
        assert(strcmp(states[state], "CHANGE_LEFT") == 0);
        lane_current = lane_actual;
        lane_target = lane_actual - 1;
        speed_final = current_plan.speed_target;
    }
    else
    {
        assert(strcmp(states[state], "CHANGE_RIGHT") == 0);
        lane_current = lane_actual;
        lane_target = lane_actual + 1;
        speed_final = current_plan.speed_target;
    }

    auto path = GenerateTrajectoryFromCurrent(lane_target, speed_final);

    return {
        path,
        lane_current,
        lane_target,
        speed_final
    };
}

Path PathPlanner::GenerateTrajectoryFromCurrent(double lane_target,
                                                double speed_target) const
{
    double s_initial;
    double d_initial;
    double speed_initial;

    s_initial = vehicle_state.s;
    d_initial = vehicle_state.d;
    speed_initial = vehicle_state.speed;

    speed_initial = min(speed_initial, speed_limit);
    double d_final = (lane_target + 0.5) * LANE_WIDTH;

    auto path = GenerateTrajectory(
        s_initial, d_initial, d_final, speed_initial, speed_target);
    return path;
}

Path
PathPlanner::GenerateTrajectory(double s_initial, double d_initial,
                                double d_final, double speed_current,
                                double speed_target) const
{
    double acc = (speed_target - speed_current) / planning_time;
    double dd = (d_final - d_initial) / planning_steps;

    Path path;
    double s = s_initial;
    double speed = speed_current;
    for (size_t i = 0; i < planning_steps; ++i)
    {
        path.x.push_back(s);
        path.y.push_back(d_initial + i * dd);

        speed += acc * planning_dt;
        s += speed * planning_dt;
    }
    return path;
}

void PathPlanner::UpdateLocalisation(VehicleState state)
{
    this->vehicle_state = state;
}

void PathPlanner::UpdateSensorFusion(SensorFusionData sensorFusionData)
{
    this->sensor_fusion_data = std::move(sensorFusionData);
    UpdateCarPaths();
}

void PathPlanner::UpdateHistory(Path previousPath)
{
    this->previous_path = std::move(previousPath);
}

double PathPlanner::SafeSpeedForLane(long lane) const
{
    auto car_in_front = FindCarToFollow(lane);
    double speed;
    if ((car_in_front != sensor_fusion_data.id.size())
        && (abs(sensor_fusion_data.s[car_in_front] - vehicle_state.s) <
            FOLLOW_DISTANCE))
    {
        speed = sqrt(
            sensor_fusion_data.vx[car_in_front] *
            sensor_fusion_data.vx[car_in_front]
            + sensor_fusion_data.vy[car_in_front] *
              sensor_fusion_data.vy[car_in_front]);
        speed = min(speed, speed_limit);
    }
    else
    {
        speed = speed_limit;
    }

    return speed;
}

double valid_lane_cost(long target)
{
    if ((target < 0) || (target > 2))
    {
        return 1;
    }
    return 0;
}

double keep_right(long current, long target)
{
    if (target <= current)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

double speed_limit_cost(const Plan& plan, double speed_limit)
{
    return min(1.0, fmax(0.0, exp(plan.speed_target - speed_limit) - 1));
}

class CostComponent
{
public:
    const char* name;
    double weight;
    function<double(const Plan&)> cost_function;
};

double PathPlanner::CostForTrajectory(const Plan& plan,
                                      CostDebugInfo& debug_info) const
{
    // If the speed difference is speed_margin the cost is 1.

    static const CostComponent components[]  {
        {"car collision",  1e9, [=](const Plan& plan) {
            return CarAvoidanceCost(plan.path);
        }}
        , {"valid lane",  1e9, [=](const Plan& plan) {
            return valid_lane_cost(plan.lane_target);
        }}
        , {"speed limit", 1e6, [=](const Plan& plan) {
            return speed_limit_cost(plan, hard_speed_limit);
        }}
        , {"speed cost", 4.0, [=](const Plan& plan) {
            return fmax(0, 1 - plan.speed_target / speed_limit);
        }}
        , {"speed opportunity cost", 4.0, [=](const Plan& plan) {
            return fmax(0, 1 - SafeSpeedForLane(plan.lane_target) / speed_limit);
        }}
        , {"keep right", 1.01, [=](const Plan& plan) {
            return keep_right(plan.lane_current, plan.lane_target);
        }}
        , {"car avoidance", 5.0, [=](const Plan& plan) {
            return SoftCarAvoidanceCost(plan.path);
        }}
        , {"lane change", 1.0, [=](const Plan& plan) {
            return abs(2 * lane_actual - plan.lane_current - plan.lane_target);
        }}
    };


    double cost_total = 0.0;
    for (auto&& item : components)
    {
        double weight = item.weight;
        const auto& cost_fn = item.cost_function;
        double cost = cost_fn(plan);
        cost_total += weight * cost;

        #ifdef DEBUG_COST
        const char* name = item.name;
        debug_info.emplace(
            name, make_tuple(cost, weight, weight * cost)
        );
        #endif
    }

    return cost_total;
}

size_t PathPlanner::FindCarToFollow(long lane) const
{
    auto closest = sensor_fusion_data.id.size();
    double closest_s = numeric_limits<double>::infinity();
    double d = LANE_WIDTH / 2 + lane * LANE_WIDTH;
    for (size_t i = 0; i < sensor_fusion_data.id.size(); ++i)
    {
        if (abs(sensor_fusion_data.d[i] - d) < LANE_WIDTH / 2)
        {
            if (sensor_fusion_data.s[i] > vehicle_state.s)
            {
                if (sensor_fusion_data.s[i] < closest_s)
                {
                    closest = i;
                    closest_s = sensor_fusion_data.s[i];
                }
            }
        }
    }

    return closest;
}

double PathPlanner::CarAvoidanceCost(const Path& path) const
{
    double cost = 0;
    for (size_t i = 0; i < sensor_fusion_data.id.size(); i++)
    {
        cost += CarAvoidanceCostPerCar(path, i);
    }
    return min(1.0, cost / sensor_fusion_data.id.size());
}

double
PathPlanner::CarAvoidanceCostPerCar(const Path& path, size_t car_paths_i) const
{
    double cost = 0.0;

    size_t n_points = min(car_paths[car_paths_i].x.size(), path.x.size());

    for (size_t i = 1; i < n_points; i++)
    {
        cost += n_points / (i + 1.0) *
            CarPotential(path.x[i], path.y[i], car_paths[car_paths_i].x[i],
                         car_paths[car_paths_i].y[i]);
    }
    return cost / n_points;
}

double
PathPlanner::CarPotential(double x, double y, double car_x, double car_y) const
{
    double forward = x - car_x;
    double lateral = y - car_y;

    double cost;
    if ((abs(lateral) < CAR_WIDTH)
        && (abs(forward) < CAR_LENGTH))
    {
        cost = 1;
    }
    else
    {
        cost = 0;
    }

    return cost;
}

double PathPlanner::SoftCarAvoidanceCost(const Path& path) const
{
    double cost = 0;
    for (size_t i = 0; i < sensor_fusion_data.id.size(); i++)
    {
        cost += SoftCarAvoidanceCostPerCar(path, i);
    }
    return cost / sensor_fusion_data.id.size();
}

double PathPlanner::SoftCarAvoidanceCostPerCar(const Path& path,
                                               size_t car_paths_i) const
{
    double cost = 0.0;

    size_t n_points = min(car_paths[car_paths_i].x.size(), path.x.size());

    for (size_t i = 1; i < n_points; i++)
    {
        cost += n_points / (i + 1.0) *
            SoftCarPotential(path.x[i], path.y[i], car_paths[car_paths_i].x[i],
                             car_paths[car_paths_i].y[i]);
    }
    return cost / n_points;
}

double PathPlanner::SoftCarPotential(double x, double y, double car_x, double car_y) const
{
    double forward = x - car_x;

    if (d_to_lane(y) != d_to_lane(car_y))
    {
        return 0;
    }

    return 1 / (pow(forward / FOLLOW_DISTANCE, 2));
}

void PathPlanner::UpdateCarPaths()
{
    vector<Path> car_paths{};
    double t = planning_dt * planning_steps;
    for (int car_id : sensor_fusion_data.id)
    {
        double car_speed = sqrt(pow(sensor_fusion_data.vx[car_id], 2)
                                + pow(sensor_fusion_data.vy[car_id], 2));
        double d_car = sensor_fusion_data.d[car_id];
        double s_car = sensor_fusion_data.s[car_id];
        auto car_path = GenerateTrajectory(s_car, d_car, d_car, car_speed, car_speed);
        car_paths.push_back(move(car_path));
    }
    this->car_paths = move(car_paths);
}

void print_cost_info(const char* name, double cost, double weight,
                     double weighted_cost)
{
    printf("%15s: base %8.3g weight %8.3g weighted cost %8.3g\n",
           name, cost, weight, weighted_cost);
}

void PathPlanner::PrintDebugInfo(const PathPlanner::CostDebugInfo& debug_info)
{
    for (auto&& entry: debug_info)
    {
        std::string name;
        double cost;
        double weight;
        double weighted_cost;
        auto info = forward_as_tuple(cost, weight, weighted_cost);
        tie(name, info) = entry;
        print_cost_info(name.c_str(), cost, weight, weighted_cost);
    }
}

#include "catch2/catch.hpp"

TEST_CASE("PathPlanner")
{
    PathPlanner planner{MapData{}};
    SECTION("CarPotential") {
        REQUIRE(planner.CarPotential(0, 0, 0, 0) == 1.0);
        REQUIRE(planner.CarPotential(0, 0, 0, 0) == 1.0);

        REQUIRE(planner.CarPotential(LANE_WIDTH, 0, 0, 0) == 0.0);
        REQUIRE(planner.CarPotential(-LANE_WIDTH, 0, 0, 0) == 0.0);

        REQUIRE(
            planner.CarPotential(LANE_WIDTH / 2, 0, 0, 0) == 1.0);
        REQUIRE(
            planner.CarPotential(-LANE_WIDTH / 2, 0, 0, 0) == 1.0);

        REQUIRE(planner.CarPotential(0, LANE_WIDTH, 0, 0) == 1.0);
        REQUIRE(planner.CarPotential(0, LANE_WIDTH, 0, 0) == 1.0);

        REQUIRE(
            planner.CarPotential(0, FOLLOW_DISTANCE, 0, 0) == 0.0);
        REQUIRE(
            planner.CarPotential(0, -FOLLOW_DISTANCE, 0, 0) == 0.0);
    }
}
