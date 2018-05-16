//
// Created by Timothy Smith on 2/4/18.
//

//#define NDEBUG
//#define DEBUG_STATE
#define DEBUG_COST
//#define DEBUG_TRAJ
#define POST_PROCESS_TRAJ

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

const size_t n_states = 6;
const char* states[n_states] = {
    "START",
    "SPEED_UP",
    "SLOW_DOWN",
    "CRUISE",
    "CHANGE_LEFT",
    "CHANGE_RIGHT"
};

static const int transitions[n_states][n_states]{
/* from \ to    *  START SPEED_UP SLOW_DOW CRUISE CHANGE_LEFT CHANGE_RIGHT */
/* START        */ {0, 1, 0, 0, 0, 0},
/* SPEED_UP     */
                   {0, 0, 0, 1, 0, 0},
/* SLOW_DOWN    */
                   {0, 0, 0, 1, 0, 0},
/* CRUISE       */
                   {0, 0, 1, 1, 1, 1},
/* CHANGE_LEFT  */
                   {0, 0, 0, 1, 1, 0},
/* CHANGE_RIGHT */
                   {0, 0, 0, 1, 0, 1}
};

static const double LANE_WIDTH = 4.0;
static const double FOLLOW_DISTANCE = 25.0;
static const double CAR_LENGTH = 7.0;
static const double CAR_WIDTH = 3.5;


vector<uint8_t> SuccessorStates(size_t state)
{
    vector<uint8_t> states{};
    for (size_t i = 0; i < n_states; ++i)
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

Path PathPlanner::PlanPath()
{
    lane_actual = lround((vehicle_state.d - LANE_WIDTH / 2) / LANE_WIDTH);

    #ifdef DEBUG_STATE
    printf("\033c");
    printf("\033[%d;%dH", 0, 0);

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
    for (size_t i = 0; i < candidate_states.size(); ++i)
    {
        auto plan = GeneratePlanForState(candidate_states[i]);
        if (!plan.path.x.empty())
        {
            cost_list[candidate_states[i]] = CostForTrajectory(
                plan,
                cost_debug_info_list[candidate_states[i]]
            );
            plan_list[candidate_states[i]] = move(plan);
        }
    }

    double minimum = cost_list[0];
    size_t minimum_i = 0;
    for (size_t i = 1; i < cost_list.size(); ++i)
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

Path smooth(const Path& path, double dt, double max_v, double max_a)
{
    Path smoothed{};

    double x = path.x[0];
    double y = path.y[0];
    double vx = path.vx[0];
    double vy = path.vy[0];

    smoothed.x = {x};
    smoothed.y = {y};

    smoothed.vx = {vx};
    smoothed.vy = {vy};

    for (size_t i = 1; i < path.x.size(); ++i)
    {
        double ax = path.ax[i - 1];
        double ay = path.ay[i - 1];

        double a = min(max_a, length(ax, ay));
        double delta = atan2(ay, ax);
        ax = a * cos(delta);
        ay = a * sin(delta);
        vx += ax * dt;
        vy += ay * dt;

        double v = min(max_v, length(vx, vy));
        double phi = atan2(vy, vx);
        vx = v * cos(phi);
        vy = v * sin(phi);
        x += vx * dt;
        y += vy * dt;

        smoothed.x.push_back(x);
        smoothed.y.push_back(y);
        smoothed.vx.push_back(vx);
        smoothed.vy.push_back(vy);
        smoothed.ax.push_back(vx);
        smoothed.ay.push_back(vy);
    }

    return smoothed;
}

Path PathPlanner::FinalTrajectory(const Path& previous_path, const Plan& plan)
{
    Path path;
    path = plan.path;
//    for (size_t i = 0; i < min(1UL, previous_path.x.size()); ++i)
//    {
//        path.x.push_back(previous_path.x[i]);
//        path.y.push_back(previous_path.y[i]);
//        path.vx.push_back(plan.path.vx[0]);
//        path.vy.push_back(plan.path.vy[0]);
//        path.ax.push_back(plan.path.ax[0]);
//        path.ay.push_back(plan.path.ay[0]);
//    }
//
//    for (size_t i = 0; i < plan.path.x.size(); ++i)
//    {
//        path.x.push_back(plan.path.x[i]);
//        path.y.push_back(plan.path.y[i]);
//        path.vx.push_back(plan.path.vx[i]);
//        path.vy.push_back(plan.path.vy[i]);
//        path.ax.push_back(plan.path.ax[i]);
//        path.ay.push_back(plan.path.ay[i]);
//    }

//    auto smoothed = smooth(path, planning_dt, 0.99 * hard_speed_limit, 0.99 * hard_acc_limit);

    #ifdef DEBUG_TRAJ
    // Sanity checks
    auto vx = diff(smoothed.x, planning_dt);
    auto vy = diff(smoothed.y, planning_dt);
    auto ax = diff(vx, planning_dt);
    auto ay = diff(vy, planning_dt);
    auto jx = diff(ax, planning_dt);
    auto jy = diff(ay, planning_dt);

    assert(all_of_list(vx, less_than(hard_speed_limit)));
    assert(all_of_list(vy, less_than(hard_speed_limit)));
    assert(all_of_list(ax, less_than(acc_limit)));
    assert(all_of_list(ay, less_than(acc_limit)));
    #endif

    vector<double> t;
    t.reserve(path.x.size());
    for (long i = 0; i < path.x.size(); ++i)
    {
        t.push_back(i * planning_dt);
    }

    tk::spline x;
    x.set_points(t, path.x);

    tk::spline y;
    y.set_points(t, path.y);

    Path final_traj;

    auto n_control_points = lround(
        ceil(path.x.size() * planning_dt / control_dt));
    for (size_t i = 0; i <n_control_points; ++i)
    {
        final_traj.x.push_back(x(control_dt * i));
        final_traj.y.push_back(y(control_dt * i));
    }

    return final_traj;
}

Plan PathPlanner::GeneratePlanForState(uint8_t state) const
{
    long lane_target;
    long lane_current;
    double speed_final;
    double t;
    double safe_acc = 5.0;
    if (state == 1)
    {
        assert(strcmp(states[state], "SPEED_UP") == 0);
        lane_current = current_plan.lane_target;
        lane_target = current_plan.lane_target;
        t = t_straight;
        speed_final = current_plan.speed_target + safe_acc * t;
    }
    else if (state == 2)
    {
        assert(strcmp(states[state], "SLOW_DOWN") == 0);
        lane_current = lane_actual;
        lane_target = lane_actual;
        t = t_straight;
        speed_final = current_plan.speed_target - safe_acc * t;
    }
    else if (state == 3)
    {
        assert(strcmp(states[state], "CRUISE") == 0);
        lane_current = lane_actual;
        lane_target = lane_actual;
        speed_final = SafeSpeedForLane(lane_target);
        t = t_straight;
    }
    else if (state == 4)
    {
        assert(strcmp(states[state], "CHANGE_LEFT") == 0);
        lane_current = lane_actual;
        lane_target = lane_actual - 1;
        speed_final = SafeSpeedForLane(lane_target);
        t = t_change;
    }
    else
    {
        assert(strcmp(states[state], "CHANGE_RIGHT") == 0);
        lane_current = lane_actual;
        lane_target = lane_actual + 1;
        speed_final = SafeSpeedForLane(lane_target);
        t = t_change;
    }

    double dr = (current_plan.speed_target + speed_final) * 0.5 * t;
    double d_final = LANE_WIDTH / 2 + lane_target * LANE_WIDTH;
    double dd = d_final - vehicle_state.d;
    double s_final;
    Path path{};

    if (abs(dd) <= abs(dr))
    {
        double ds = sqrt(dr * dr - dd * dd);
        s_final = vehicle_state.s + ds;
    }
    else
    {
        // This path may be infeasible, or we are just standing still at the
        // start. If it is actually infeasible, the smoothing cost will remove
        // it from consideration. Either way, assume we are heading only
        // forward.
        s_final = vehicle_state.s + dr;
    }
    path = GenerateTrajectoryFromCurrent(t, s_final, d_final, speed_final);

    return {
        path,
        lane_current,
        lane_target,
        speed_final
    };
}

Path PathPlanner::GenerateTrajectoryFromCurrent(double t_final, double s_final,
                                                double d_final,
                                                double speed_final) const
{
    double x_initial;
    double y_initial;
    double vx_initial;
    double vy_initial;
    double ax_initial;
    double ay_initial;

    if (previous_path.x.size() > 2)
    {
        // Take the first 3 points, i.e., 0, 1, 2 and make central differences
        // around them. Reasonable compromise between smoothness with previous
        // curve and control responsiveness. Taking more points, or backward
        // differences will make the curve smoother, but responds too slow.

        x_initial = previous_path.x[0];
        y_initial = previous_path.y[0];

        vx_initial = (previous_path.x[1] - previous_path.x[0]) / control_dt;
        double vx2_initial =
            (previous_path.x[2] - previous_path.x[1]) / control_dt;
        ax_initial = (vx2_initial - vx_initial) / control_dt;
        vx_initial = (vx_initial + vx2_initial) / 2.0;

        vy_initial = (previous_path.y[1] - previous_path.y[0]) / control_dt;
        double vy2_initial =
            (previous_path.y[2] - previous_path.y[1]) / control_dt;
        ay_initial = (vy2_initial - vy_initial) / control_dt;
        vy_initial = (vy_initial + vy2_initial) / 2.0;
    }
    else
    {
        x_initial = vehicle_state.x;
        y_initial = vehicle_state.y;

        vx_initial = 0;
        ax_initial = 0;

        vy_initial = 0;
        ay_initial = 0;
    }

    auto path = GenerateTrajectory(t_final, s_final, d_final, speed_final,
                                   x_initial, y_initial,
                                   vx_initial, vy_initial,
                                   ax_initial, ay_initial);

    return path;
}

Path
PathPlanner::GenerateTrajectory(double t_final, double s_final, double d_final,
                                double speed_final,
                                double x_initial, double y_initial,
                                double vx_initial, double vy_initial,
                                double ax_initial, double ay_initial) const
{
    double x_final;
    double y_final;
    tie(x_final, y_final) = map_data.InterpolateRoadCoords(s_final, d_final);

    double tx, ty;
    tie(tx, ty) = map_data.InterpolateRoadTangent(s_final);
    double t = length(tx, ty);
    double vx_final = tx * speed_final / t;
    double vy_final = ty * speed_final / t;

    return InterpolatePath(t_final,
                           x_initial, x_final,
                           y_initial, y_final,
                           vx_initial, vx_final,
                           vy_initial, vy_final,
                           ax_initial, 0,
                           ay_initial, 0);

}

Path
PathPlanner::InterpolatePath(double t_final, double x_initial, double x_final,
                             double y_initial, double y_final,
                             double vx_initial,
                             double vx_final, double vy_initial,
                             double vy_final,
                             double ax_initial, double ax_final,
                             double ay_initial,
                             double ay_final) const
{
    auto x_curve = JerkMinimalTrajectory({x_initial, vx_initial, ax_initial},
                                         {x_final, vx_final, ax_final},
                                         t_final);
    auto y_curve = JerkMinimalTrajectory({y_initial, vy_initial, ay_initial},
                                         {y_final, vy_final, ay_final},
                                         t_final);

    auto vx_curve = x_curve.Differentiate();
    auto vy_curve = y_curve.Differentiate();
    auto ax_curve = vx_curve.Differentiate();
    auto ay_curve = vy_curve.Differentiate();

    Path path{};
    auto n_points = lround(floor(t_final / planning_dt));
    for (int i = 0; i < n_points; i++)
    {
        double t = i * planning_dt;

        path.x.push_back(x_curve.Evaluate(t));
        path.y.push_back(y_curve.Evaluate(t));
        path.vx.push_back(vx_curve.Evaluate(t));
        path.vy.push_back(vy_curve.Evaluate(t));
        path.ax.push_back(ax_curve.Evaluate(t));
        path.ay.push_back(ay_curve.Evaluate(t));
    }

    return move(path);
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

double speed_limit_cost(const Path& path, double speed_limit)
{
    assert(path.vx.size() == path.vy.size());

    double cost_total = 0;
    for (size_t i = 0; i < path.vx.size(); ++i)
    {
        double v = length(path.vx[i], path.vy[i]);
        cost_total += fmax(0.0, exp(v - speed_limit) - 1);
    }

    return min(1.0, cost_total / path.vx.size());
}

double smoothness_cost(const Path& path, double dt, double a_max)
{
    assert(path.ax.size() == path.ay.size());
    assert(path.ax.size() > 0);

    double cost_total = 0;

    for (size_t i = 1; i < path.vx.size(); ++i)
    {
        double a = length(path.ax[i], path.ay[i]);
        cost_total += fmax(0.0, exp(a - a_max) - 1);
    }
    return min(1.0, cost_total / path.vx.size());
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
            return speed_limit_cost(plan.path, hard_speed_limit);
        }}
//        , {"smoothness ", 1e3, [=](const Plan& plan) {
//            return smoothness_cost(plan.path, planning_dt, hard_acc_limit);
//        }}
        , {"speed cost", 3.0, [=](const Plan& plan) {
            return fmax(0, 1 - plan.speed_target / speed_limit);
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
//        , {"speed change", 1.0, [=](const Plan& plan) {
//            return fabs(current_plan.speed_target - plan.speed_target) / (5.0 * t_straight);
//        }}
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
        cost += n_points / (i + 1.0) * CarPotential(
            path.x[i], path.y[i],
            path.vx[i], path.vy[i],
            car_paths[car_paths_i].x[i], car_paths[car_paths_i].y[i],
            car_paths[car_paths_i].vx[i], car_paths[car_paths_i].vy[i]
        );
    }
    return cost / n_points;
}

double PathPlanner::CarPotential(double x, double y,
                                 double vx, double vy,
                                 double car_x, double car_y,
                                 double car_vx, double car_vy) const
{
    double dx = x - car_x;
    double dy = y - car_y;
    double r = sqrt(dx * dx + dy * dy);

    double car_rv = sqrt(car_vx * car_vx + car_vy * car_vy);
    if (car_rv == 0)
    {
        if (r < CAR_WIDTH)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    double forward = (dx * car_vx + dy * car_vy) / car_rv;
    double lateral = (dy * car_vx - dx * car_vy) / car_rv;

    double cost = 0;
    if ((abs(lateral) < CAR_WIDTH)
        && (abs(forward) < CAR_LENGTH))
    {
        cost += 1;
    }
    else
    {
        cost += 0;
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
        cost += n_points / (i + 1.0) * SoftCarPotential(
            path.x[i], path.y[i],
            path.vx[i], path.vy[i],
            car_paths[car_paths_i].x[i], car_paths[car_paths_i].y[i],
            car_paths[car_paths_i].vx[i], car_paths[car_paths_i].vy[i]
        );
    }
    return cost / n_points;
}

double PathPlanner::SoftCarPotential(double x, double y,
                                     double vx, double vy,
                                     double car_x, double car_y,
                                     double car_vx, double car_vy) const
{
    double dx = x - car_x;
    double dy = y - car_y;
    double r = dx * dx + dy * dy;

    double car_rv = sqrt(car_vx * car_vx + car_vy * car_vy);
    if (car_rv == 0)
    {
        if (r < CAR_WIDTH)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    double forward = (dx * car_vx + dy * car_vy) / car_rv;
    double lateral = (dy * car_vx - dx * car_vy) / car_rv;

    return 1 / ((pow(lateral / CAR_WIDTH, 2)
                 + pow(forward / FOLLOW_DISTANCE, 2)));
}

void PathPlanner::UpdateCarPaths()
{
    vector<Path> car_paths{};
    double t = t_straight;
    for (size_t i = 0; i < sensor_fusion_data.id.size(); ++i)
    {
        auto car_id = sensor_fusion_data.id[i];
        double car_speed = sqrt(pow(sensor_fusion_data.vx[car_id], 2)
                                + pow(sensor_fusion_data.vy[car_id], 2));
        double d_car = sensor_fusion_data.d[car_id];

        auto car_path = GenerateTrajectory(
            t, sensor_fusion_data.s[car_id] + car_speed + t, d_car, car_speed,
            sensor_fusion_data.x[car_id], sensor_fusion_data.y[car_id],
            sensor_fusion_data.vx[car_id], sensor_fusion_data.vy[car_id],
            0.0, 0.0
        );
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
        REQUIRE(planner.CarPotential(0, 0, 0, 0, 0, 0, 0, 0) == 1.0);
        REQUIRE(planner.CarPotential(0, 0, 0, 0, 0, 0, 1, 1) == 1.0);

        REQUIRE(planner.CarPotential(LANE_WIDTH, 0, 0, 0, 0, 0, 0, 1) == 0.0);
        REQUIRE(planner.CarPotential(-LANE_WIDTH, 0, 0, 0, 0, 0, 0, 1) == 0.0);

        REQUIRE(
            planner.CarPotential(LANE_WIDTH / 2, 0, 0, 0, 0, 0, 0, 1) == 1.0);
        REQUIRE(
            planner.CarPotential(-LANE_WIDTH / 2, 0, 0, 0, 0, 0, 0, 1) == 1.0);

        REQUIRE(planner.CarPotential(0, LANE_WIDTH, 0, 0, 0, 0, 0, 1) == 1.0);
        REQUIRE(planner.CarPotential(0, LANE_WIDTH, 0, 0, 0, 0, 0, 1) == 1.0);

        REQUIRE(
            planner.CarPotential(0, FOLLOW_DISTANCE, 0, 0, 0, 0, 0, 1) == 0.0);
        REQUIRE(
            planner.CarPotential(0, -FOLLOW_DISTANCE, 0, 0, 0, 0, 0, 1) == 0.0);
    }
}

TEST_CASE("Path costs")
{
    PathPlanner planner{MapData{}};
    const double speed_limit = planner.speed_limit;
    const double hard_speed_limit = planner.hard_speed_limit;
    double t_straight = planner.t_straight;
    double t_change = planner.t_change;
    double dt = planner.planning_dt;
    SECTION("straight") {
        auto path = planner.InterpolatePath(
            t_straight,
            0.0, speed_limit * t_straight,
            0.0, 0.0,
            speed_limit, speed_limit,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0);

        REQUIRE(speed_limit_cost(path, hard_speed_limit) < 1.0);
        REQUIRE(smoothness_cost(path, dt, 10.0) < 1.0);

    }

    SECTION("speed up") {
        double v0 = 0.0;
        double v1 = v0 + 5.0 * t_straight;
        auto path = planner.InterpolatePath(
            t_straight,
            0.0, (v1 + v0) * 0.5 * t_straight,
            0.0, 0.0,
            v0, v1,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0);

        REQUIRE(speed_limit_cost(path, hard_speed_limit) < 1.0);
        REQUIRE(smoothness_cost(path, dt, 10.0) < 1.0);

    }

    SECTION("slow down") {
        double v0 = speed_limit;
        double v1 = v0 - 5.0 * t_straight;
        auto path = planner.InterpolatePath(
            t_straight,
            0.0, (v1 + v0) * 0.5 * t_straight,
            0.0, 0.0,
            v0, v1,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0);

        REQUIRE(speed_limit_cost(path, hard_speed_limit) < 1.0);
        REQUIRE(smoothness_cost(path, dt, 10.0) < 1.0);

    }

    SECTION("left") {
        auto path = planner.InterpolatePath(
            t_change,
            0.0, speed_limit * t_change,
            0.0, -LANE_WIDTH,
            speed_limit, speed_limit,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0
        );
        REQUIRE(speed_limit_cost(path, hard_speed_limit) < 1.0);
        REQUIRE(smoothness_cost(path, dt, 10.0) < 1.0);
    }

    SECTION("right") {
        auto path = planner.InterpolatePath(
            t_change,
            0.0, speed_limit * t_change,
            0.0, LANE_WIDTH,
            speed_limit, speed_limit,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0
        );

        REQUIRE(speed_limit_cost(path, hard_speed_limit) < 1.0);
        REQUIRE(smoothness_cost(path, dt, 10.0) < 1.0);

    }

    SECTION("too fast") {
        auto too_fast = hard_speed_limit * 1.1;
        auto path = planner.InterpolatePath(
            t_straight,
            0.0, too_fast * t_straight,
            0.0, 0.0,
            speed_limit, too_fast,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0
        );

        REQUIRE(speed_limit_cost(path, hard_speed_limit) >= 1.0);
        REQUIRE(smoothness_cost(path, dt, 10.0) >= 1.0);

    }

}
