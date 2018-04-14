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
using std::tuple;
using std::function;
using std::get;
using namespace std::this_thread;

const size_t n_states = 5;
const char* states[n_states] = {
    "START",
    "FOLLOW",
    "CHANGE_LEFT",
    "CHANGE_RIGHT",
    "EMERGENCY_STOP"
};

static const int transitions[n_states][n_states] {
    {0, 1, 0, 0, 0},
    {0, 1, 1, 1, 1},
    {0, 1, 1, 0, 1},
    {0, 1, 0, 1, 1},
    {0, 1, 0, 0, 1}

};

static const double LANE_WIDTH = 4.0;
static const double FOLLOW_DISTANCE = 25.0;
static const double CAR_LENGTH = 5.0;
static const double CAR_WIDTH = 2.5;

#define DEBUG_STATE
#define DEBUG_COST

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
    current_plan{{}, 2, 2, 0},
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
    return plan_list[minimum_i].path;
}

Path PathPlanner::GenerateTrajectory(double t_final, double s_final, double d_final,
                                     double speed_final) const
{
    double x_initial;
    if (!previous_path.x.empty())
    {
        x_initial = previous_path.x.front();
    }
    else
    {
        x_initial = vehicle_state.x;
    }

    double y_initial;
    if (!previous_path.y.empty())
    {
        y_initial = previous_path.y.front();
    }
    else
    {
        y_initial = vehicle_state.y;
    }

    double vx_initial;
    double vy_initial;
    double ax_initial;
    double ay_initial;
    if (previous_path.x.size() > 2)
    {
        vx_initial = (previous_path.x[1] - previous_path.x[0]) / dt;
        double vx2_initial =
            (previous_path.x[2] - previous_path.x[1]) / dt;
        ax_initial = (vx2_initial - vx_initial) / dt;
        vx_initial = (vx_initial + vx2_initial) / 2.0;
    }
    else
    {
        vx_initial = 0;
        ax_initial = 0;
    }
    if (previous_path.y.size() > 2)
    {
        vy_initial = (previous_path.y[1] - previous_path.y[0]) / dt;
        double vy2_initial =
            (previous_path.y[2] - previous_path.y[1]) / dt;
        ay_initial = (vy2_initial - vy_initial) / dt;
        vy_initial = (vy_initial + vy2_initial) / 2.0;
    }
    else
    {
        vy_initial = 0;
        ay_initial = 0;
    }
    auto path = GenerateTrajectory(t_final, s_final, d_final, speed_final, x_initial,
                       y_initial, vx_initial, vy_initial, ax_initial,
                       ay_initial);

    return std::move(path);
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
    double vx_final = tx * speed_final;
    double vy_final = ty * speed_final;

    auto x_curve = JerkMinimalTrajectory({x_initial, vx_initial, 0},
                                    {x_final, vx_final, 0},
                                    t_final);
    auto y_curve = JerkMinimalTrajectory({y_initial, vy_initial, 0},
                                    {y_final, vy_final, 0},
                                    t_final);

    auto vx_curve = x_curve.Differentiate();
    auto vy_curve = y_curve.Differentiate();

    Path path{};
    auto n_points = lround(floor(t_final / dt));
    for (int i = 0; i < n_points; i++)
    {
        double t = i * dt;

        path.x.push_back(x_curve.Evaluate(t));
        path.y.push_back(y_curve.Evaluate(t));
        path.vx.push_back(vx_curve.Evaluate(t));
        path.vy.push_back(vy_curve.Evaluate(t));
    }

    return std::move(path);
}

void PathPlanner::UpdateLocalisation(VehicleState state)
{
    this->vehicle_state = state;
}

void PathPlanner::UpdateSensorFusion(SensorFusionData sensorFusionData)
{
    this->sensor_fusion_data = std::move(sensorFusionData);
}

void PathPlanner::UpdateHistory(Path previousPath)
{
    this->previous_path = std::move(previousPath);
}

Plan PathPlanner::GeneratePlanForState(uint8_t state) const
{
    long lane;
    double speed;
    double t;
    if (state == 1)  // FOLLOW
    {
        lane = current_plan.lane_target;
        speed = SafeSpeedForLane(lane);
        t = 1.0;
    }
    else if(state == 2) // CHANGE_LEFT
    {
        lane = lane_actual - 1;
        speed = SafeSpeedForLane(lane);
        t = 1.5;
    }
    else if(state == 3)  // CHANGE_RIGHT
    {
        lane = lane_actual + 1;
        speed = SafeSpeedForLane(lane);
        t = 1.5;

    }
    else  // EMERGENCY_STOP
    {
        assert(state == 4);
        lane = lane_actual;
        speed = 0;
        t = 2.5;

    }

    double s_final = vehicle_state.s + speed * t;
    double d_final = LANE_WIDTH / 2 + lane * LANE_WIDTH;
    return {
        GenerateTrajectory(t, s_final, d_final, speed),
        lane_actual,
        lane,
        speed
    };
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

double smoothness_cost(const Path& path, double dt, double a_max) {
    assert(path.vx.size() == path.vy.size());
    assert(path.vx.size() > 0);

    double cost_total = 0;

    double vx0 = path.vx[0];
    double vy0 = path.vy[0];

    for (size_t i = 1; i < path.vx.size(); ++i) {
        double vx1 = path.vx[i];
        double vy1 = path.vy[i];

        double a = distance(vx0, vy0, vx1, vy1) / dt;
        cost_total += a / a_max;

        vx0 = vx1;
        vy0 = vy1;
    }
    return cost_total / path.vx.size();
}

double PathPlanner::CostForTrajectory(const Plan& plan) const
{
    // If the speed difference is speed_margin the cost is 1.
    double speed_margin = 5.0;

    static const map<const char*, tuple<double, function<double(const Plan&)> > > components {
        {"speed cost", { 3.0, [=](const Plan& plan) {
            return fmax(0, speed_limit - plan.speed_target) / speed_margin;
        }}},
        {"valid lane", { 100000.0, [=](const Plan& plan) {
            return valid_lane_cost(plan.lane_target);
        }}},
        {"keep right", { 1.01, [=](const Plan& plan) {
            return keep_right(plan.lane_current, plan.lane_target);
        }}},
        {"car avoidance", { 5.0, [=](const Plan& plan) {
            return CarAvoidanceCost(plan.path);
        }}},
        {"lane change", { 1.0, [=](const Plan& plan) {
            return abs(2 * lane_actual - plan.lane_current - plan.lane_target);
        }}},
        {"smoothness ", { 0.1, [=](const Plan& plan) {
            return smoothness_cost(plan.path, dt, 9.0);
        }}}
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

double PathPlanner::CarAvoidanceCostPerCar(const Path& path, size_t car_id) const
{
    double cost = 0;
    double t = path.x.size() * dt;
    double car_speed = sqrt(pow(sensor_fusion_data.vx[car_id], 2)
                            + pow(sensor_fusion_data.vy[car_id], 2));
    double d_car = sensor_fusion_data.d[car_id];

    auto car_path = GenerateTrajectory(
        t, sensor_fusion_data.s[car_id] + car_speed + t, d_car, car_speed,
        sensor_fusion_data.x[car_id], sensor_fusion_data.y[car_id],
        sensor_fusion_data.vx[car_id], sensor_fusion_data.vy[car_id],
        0.0, 0.0
    );

    for (size_t i = 1; i < path.x.size(); i++) {
        cost += 1000 * CarPotential(
            path.x[i], path.y[i],
            path.vx[i], path.vy[i],
            car_path.x[i], car_path.y[i],
            car_path.vx[i], car_path.vy[i]
        );

        cost += 1.0 * SoftCarPotential(
            path.x[i], path.y[i],
            path.vx[i], path.vy[i],
            car_path.x[i], car_path.y[i],
            car_path.vx[i], car_path.vy[i]
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

#include "catch.hpp"

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
