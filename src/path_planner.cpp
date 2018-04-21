//
// Created by Timothy Smith on 2/4/18.
//

#include <utility>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include <numeric>
#include <map>
#include <functional>
#include <thread>

#include "path_planner.hpp"
#include "utilities.hpp"

using std::vector;
using std::move;
using std::numeric_limits;
using std::tie;
using std::map;
using std::min;
using std::max;
using std::tuple;
using std::function;
using std::get;
using namespace std::this_thread;

const size_t n_states = 6;
const char* states[n_states] = {
    "START",
    "SPEED_UP",
    "SLOW_DOWN",
    "CRUISE",
    "CHANGE_LEFT",
    "CHANGE_RIGHT"
};

static const int transitions[n_states][n_states] {
/* from \ to    *  START SPEED_UP SLOW_DOW CRUISE CHANGE_LEFT CHANGE_RIGHT */
/* START        */ {0,   0,       0,       1,     0,          0},
/* SPEED_UP     */ {0,   0,       0,       1,     0,          0},
/* SLOW_DOWN    */ {0,   0,       0,       1,     0,          0},
/* CRUISE       */ {0,   0,       0,       1,     1,          1},
/* CHANGE_LEFT  */ {0,   0,       0,       1,     1,          0},
/* CHANGE_RIGHT */ {0,   0,       0,       1,     0,          1}
};

static const double LANE_WIDTH = 4.0;
static const double FOLLOW_DISTANCE = 25.0;
static const double CAR_LENGTH = 7.0;
static const double CAR_WIDTH = 3.5;

//#define DEBUG_STATE
//#define DEBUG_COST
#define DEBUG_TRAJ

vector<uint8_t> SuccessorStates(size_t state) {
    vector<uint8_t> states{};
    for(size_t i = 0; i < n_states; ++i) {
        if(transitions[state][i])
        {
            states.push_back(i);
        }
    }
    return states;
}

PathPlanner::PathPlanner(double dt, MapData mapData) :
    dt(dt),
    map_data{std::move(mapData)},
    current_plan{{}, 1, 1, 0},
    planner_state{0}
{
    this->map_data.PrepareInterpolation();
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

    for (size_t i = 1; i < path.x.size(); ++i) {
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

Path PathPlanner::PlanPath()
{
    lane_actual = lround((vehicle_state.d - LANE_WIDTH / 2) / LANE_WIDTH);

    #ifdef DEBUG_STATE
//    printf("\033c");
//    printf("\033[%d;%dH", 0, 0);

    printf("Planner state %s: lane_actual %ld lane_current %ld lane_target %ld speed_target %f\n",
                 states[planner_state],
                 lane_actual,
                 current_plan.lane_current,
                 current_plan.lane_target,
                 current_plan.speed_target
    );

    printf("Vehicle state: s %f d %f x %f y %f speed %f\n",
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
        printf("Road state: x %f y %f dx %f dy %f\n",
               x,
               y,
               dx,
               dy
        );
    }
    #endif

    auto candidate_states = SuccessorStates(planner_state);
    std::vector<double> cost_list(candidate_states.size(), std::numeric_limits<float>::infinity());
    vector<Plan> plan_list(candidate_states.size());
    for (size_t i = 0; i < cost_list.size(); ++i)
    {
        auto plan = GeneratePlanForState(candidate_states[i]);
        if(!plan.path.x.empty()) {
            #ifdef DEBUG_COST
            printf("PLAN %s\n", states[candidate_states[i]]);
            #endif

            cost_list[i] = CostForTrajectory(plan);

            #ifdef DEBUG_COST
            printf("TOTAL: %f\n", cost_list[i]);
            #endif

            plan_list[i] = move(plan);
        }
    }

    double minimum = cost_list[0];
    size_t minimum_i = 0;
    for(size_t i = 1; i < cost_list.size(); ++i) {
        if(cost_list[i] < minimum) {
            minimum = cost_list[i];
            minimum_i = i;
        }
    }
    auto next_state = candidate_states[minimum_i];

    planner_state = next_state;
    current_plan = plan_list[minimum_i];

    vector<double> x;
    vector<double> y;
    for(size_t i = 0; i < min(2UL, previous_path.x.size()); ++i) {
        x.push_back(previous_path.x[i]);
        y.push_back(previous_path.y[i]);
    }

    for(size_t i = 0; i < current_plan.path.x.size(); ++i) {
        x.push_back(current_plan.path.x[i]);
        y.push_back(current_plan.path.y[i]);
    }

    current_plan.path = {x, y};

    #ifdef DEBUG_TRAJ
    // Sanity checks
    auto vx = diff(current_plan.path.x, dt);
    auto vy = diff(current_plan.path.y, dt);
    auto ax = diff(vx, dt);
    auto ay = diff(vy, dt);
    auto jx = diff(ax, dt);
    auto jy = diff(ay, dt);

    assert(all_of_list(vx, less_than(hard_speed_limit)));
    assert(all_of_list(vy, less_than(hard_speed_limit)));
//    assert(all_of_list(ax, less_than(acc_limit)));
//    assert(all_of_list(ay, less_than(acc_limit)));
    #endif

    return current_plan.path;

}

Plan PathPlanner::GeneratePlanForState(uint8_t state) const
{
    long lane;
    double speed_final;
    double t;
    double safe_acc = 5.0;
    if (state == 1)
    {
        assert(strcmp(states[state], "SPEED_UP") == 0);
        lane = current_plan.lane_target;
        t = t_straight;
        speed_final = current_plan.speed_target + safe_acc * t;
    }
    else if(state == 2)
    {
        assert(strcmp(states[state], "SLOW_DOWN") == 0);
        lane = current_plan.lane_target;
        t = t_straight;
        speed_final = current_plan.speed_target - safe_acc * t;
    }
    else if(state == 3)
    {
        assert(strcmp(states[state], "CRUISE") == 0);
        lane = current_plan.lane_target;
        speed_final = SafeSpeedForLane(lane);
        t = t_straight;
    }
    else if(state == 4)
    {
        assert(strcmp(states[state], "CHANGE_LEFT") == 0);
        lane = current_plan.lane_current - 1;
        speed_final = SafeSpeedForLane(lane);
        t = t_change;
    }
    else
    {
        assert(strcmp(states[state], "CHANGE_RIGHT") == 0);
        lane = current_plan.lane_current + 1;
        speed_final = SafeSpeedForLane(lane);
        t = t_change;
    }

    double d_final = LANE_WIDTH / 2 + lane * LANE_WIDTH;
    double s_final = vehicle_state.s + (current_plan.speed_target + speed_final) * 0.5 * t;
    return {
        GenerateTrajectory(t, s_final, d_final, speed_final),
        lane_actual,
        lane,
        speed_final
    };
}

Path PathPlanner::GenerateTrajectory(double t_final, double s_final, double d_final,
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
        x_initial = previous_path.x[2];
        y_initial = previous_path.y[2];

        vx_initial = (previous_path.x[1] - previous_path.x[0]) / dt;
        double vx2_initial =
            (previous_path.x[2] - previous_path.x[1]) / dt;
        ax_initial = (vx2_initial - vx_initial) / dt;
        vx_initial = (vx_initial + vx2_initial) / 2.0;

        vy_initial = (previous_path.y[1] - previous_path.y[0]) / dt;
        double vy2_initial =
            (previous_path.y[2] - previous_path.y[1]) / dt;
        ay_initial = (vy2_initial - vy_initial) / dt;
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

    double v = length(vx_initial, vy_initial);
    if (v > speed_limit) {
        vx_initial = vx_initial * speed_limit / v;
        vy_initial = vy_initial * speed_limit / v;
    }
    double a = length(ax_initial, ay_initial);
    if (a > acc_limit) {
        ax_initial = ax_initial * acc_limit / a;
        ay_initial = ay_initial * acc_limit / a;
    }

    auto path = GenerateTrajectory(t_final, s_final, d_final, speed_final,
                                   x_initial, y_initial,
                                   vx_initial, vy_initial,
                                   ax_initial, ay_initial);
    return smooth(path, dt, hard_speed_limit, hard_acc_limit);
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
                             double y_initial, double y_final, double vx_initial,
                             double vx_final, double vy_initial, double vy_final,
                             double ax_initial, double ax_final, double ay_initial,
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
    auto n_points = lround(floor(t_final / dt));
    for (int i = 0; i < n_points; i++)
    {
        double t = i * dt;

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

double valid_lane_cost(long target) {
    if ((target < 0) || (target > 2)) {
        return 1;
    }
    return 0;
}

double keep_right(long current, long target) {
    if (target <= current) {
        return 1;
    }
    else {
        return 0;
    }
}

double speed_limit_cost(const Path& path, double speed_limit) {
    assert(path.vx.size() == path.vy.size());

    double cost_total = 0;
    for (size_t i = 0; i < path.vx.size(); ++i) {
        double v = length(path.vx[i], path.vy[i]);
        cost_total += fmax(0.0, exp(v - speed_limit) -1);
    }

    return min(1.0, cost_total / path.vx.size());
}

double smoothness_cost(const Path& path, double dt, double a_max) {
    assert(path.ax.size() == path.ay.size());
    assert(path.ax.size() > 0);

    double cost_total = 0;

    for (size_t i = 1; i < path.vx.size(); ++i) {
        double a = length(path.ax[i], path.ay[i]);
        cost_total += fmax(0.0, exp(a - a_max) - 1);
    }
    return min(1.0, cost_total / path.vx.size());
}

double PathPlanner::CostForTrajectory(const Plan& plan) const
{
    // If the speed difference is speed_margin the cost is 1.
    const double speed_margin = 5.0;

    static const map<const char*, tuple<double, function<double(const Plan&)> > > components {
        {"car collision", { 1e9, [=](const Plan& plan) {
            return CarAvoidanceCost(plan.path);
        }}}
        , {"valid lane", { 1e9, [=](const Plan& plan) {
            return valid_lane_cost(plan.lane_target);
        }}}
        , {"speed limit", { 1e6, [=](const Plan& plan) {
            return speed_limit_cost(plan.path, speed_limit);
        }}}
//        , {"smoothness ", { 1e3, [=](const Plan& plan) {
//            return smoothness_cost(plan.path, dt, acc_limit);
//        }}}
        , {"speed cost", { 3.0, [=](const Plan& plan) {
            return fmax(0, speed_limit - plan.speed_target) / speed_margin;
        }}}
        , {"keep right", { 1.01, [=](const Plan& plan) {
            return keep_right(plan.lane_current, plan.lane_target);
        }}}
        , {"car avoidance", { 5.0, [=](const Plan& plan) {
            return SoftCarAvoidanceCost(plan.path);
        }}}
        , {"lane change", { 1.0, [=](const Plan& plan) {
            return abs(2 * lane_actual - plan.lane_current - plan.lane_target);
        }}}
//        , {"speed change", { 1.0, [=](const Plan& plan) {
//            return fabs(current_plan.speed_target - plan.speed_target) / (5.0 * t_straight);
//        }}}
    };


    double cost_total = 0.0;
    for (auto&& item : components)
    {
        const char* name = item.first;
        double weight = get<0>(item.second);
        function<double(const Plan&)> cost_fn = get<1>(item.second);
        double cost = cost_fn(plan);
        cost_total += weight * cost;

        #ifdef DEBUG_COST
        printf("%15s: base %f weighted %f\n", name, cost, cost * weight);
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
        if (abs(sensor_fusion_data.d[i] - d) < LANE_WIDTH / 2) {
            if (sensor_fusion_data.s[i] > vehicle_state.s) {
                if (sensor_fusion_data.s[i] < closest_s) {
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
    return cost / sensor_fusion_data.id.size();
}

double PathPlanner::CarAvoidanceCostPerCar(const Path& path, size_t car_paths_i) const
{
    double cost = 0.0;

    size_t n_points = min(car_paths[car_paths_i].x.size(), path.x.size());

    for (size_t i = 1; i < n_points; i++) {
        cost += CarPotential(
            path.x[i], path.y[i],
            path.vx[i], path.vy[i],
            car_paths[car_paths_i].x[i], car_paths[car_paths_i].y[i],
            car_paths[car_paths_i].vx[i], car_paths[car_paths_i].vy[i]
        );
    }
    return cost / path.x.size();
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

double PathPlanner::SoftCarAvoidanceCostPerCar(const Path& path, size_t car_paths_i) const
{
    double cost = 0.0;

    size_t n_points = min(car_paths[car_paths_i].x.size(), path.x.size());

    for (size_t i = 1; i < n_points; i++) {
        cost += SoftCarPotential(
            path.x[i], path.y[i],
            path.vx[i], path.vy[i],
            car_paths[car_paths_i].x[i], car_paths[car_paths_i].y[i],
            car_paths[car_paths_i].vx[i], car_paths[car_paths_i].vy[i]
        );
    }
    return cost / path.x.size();
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
    if (car_rv == 0) {
        if (r < CAR_WIDTH) {
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
        && (abs(forward) < CAR_LENGTH )) {
        cost += 1;
    }
    else {
        cost += 0;
    }

    return cost;
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
    if (car_rv == 0) {
        if (r < CAR_WIDTH) {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    double forward = (dx * car_vx + dy * car_vy) / car_rv;
    double lateral = (dy * car_vx - dx * car_vy) / car_rv;

    return exp(-(pow(lateral / CAR_WIDTH, 2)
                 + pow(forward / FOLLOW_DISTANCE, 2)));
}

void PathPlanner::UpdateCarPaths()
{
    vector<Path> car_paths{};
    double t = t_straight;
    for (size_t i = 0; i < sensor_fusion_data.id.size(); ++i) {
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

#include "catch2/catch.hpp"

TEST_CASE("PathPlanner") {
    PathPlanner planner{1.0, MapData{}};
    SECTION("CarPotential") {
    REQUIRE(planner.CarPotential(0, 0, 0, 0, 0, 0, 0, 0) == 1.0);
    REQUIRE(planner.CarPotential(0, 0, 0, 0, 0, 0, 1, 1) == 1.0);

    REQUIRE(planner.CarPotential( LANE_WIDTH, 0, 0, 0, 0, 0, 0, 1) == 0.0);
    REQUIRE(planner.CarPotential(-LANE_WIDTH, 0, 0, 0, 0, 0, 0, 1) == 0.0);

    REQUIRE(planner.CarPotential( LANE_WIDTH / 2, 0, 0, 0, 0, 0, 0, 1) == 1.0);
    REQUIRE(planner.CarPotential(-LANE_WIDTH / 2, 0, 0, 0, 0, 0, 0, 1) == 1.0);

    REQUIRE(planner.CarPotential(0, LANE_WIDTH, 0, 0, 0, 0, 0, 1) == 1.0);
    REQUIRE(planner.CarPotential(0, LANE_WIDTH, 0, 0, 0, 0, 0, 1) == 1.0);

    REQUIRE(planner.CarPotential(0,  FOLLOW_DISTANCE, 0, 0, 0, 0, 0, 1) == 0.0);
    REQUIRE(planner.CarPotential(0, -FOLLOW_DISTANCE, 0, 0, 0, 0, 0, 1) == 0.0);
    }
}

TEST_CASE("Path costs") {
    PathPlanner planner{0.02, MapData{}};
    const double speed_limit = planner.speed_limit;
    const double hard_speed_limit = planner.hard_speed_limit;
    double t_straight = planner.t_straight;
    double t_change = planner.t_change;
    double dt = planner.dt;
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

        REQUIRE(speed_limit_cost(path, hard_speed_limit) > 1.0);
        REQUIRE(smoothness_cost(path, dt, 10.0) > 1.0);

    }

}
