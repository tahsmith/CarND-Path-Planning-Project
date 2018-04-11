//
// Created by Timothy Smith on 2/4/18.
//

#include <utility>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include <numeric>

#include "path_planner.hpp"
#include "utilities.hpp"

using std::vector;
using std::move;
using std::numeric_limits;

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
    {0, 1, 1, 1, 0},
    {0, 1, 1, 0, 0},
    {0, 1, 0, 1, 0},
    {0, 1, 0, 0, 1}

};

static const double LANE_WIDTH = 4.0;
static const double FOLLOW_DISTANCE = 15.0;
static const double CAR_LENGTH = 5.0;
static const double CAR_WIDTH = 3.2;

static bool debug_cost = false;


vector<uint8_t> SuccessorStates(size_t state) {
    vector<uint8_t> states{};
    for(size_t i = 0; i < n_states; ++i) {
        if(transitions[state][i])
        states.push_back(i);
    }
    return states;
}

PathPlanner::PathPlanner(double dt, MapData mapData) :
    dt(dt),
    mapData{std::move(mapData)},
    current_plan{{}, 2, 2, 0},
    planner_state{0}
{}

Path PathPlanner::PlanPath()
{
    lane_actual = lround((vehicle_state.d - LANE_WIDTH / 2) / LANE_WIDTH);

    printf("PlanPath()\n%s: lane_actual %ld lane_current %ld lane_target %ld speed_target %f\n",
           states[planner_state],
           lane_actual,
           current_plan.lane_current,
           current_plan.lane_target,
           current_plan.speed_target
    );

    auto candidate_states = SuccessorStates(planner_state);
    std::vector<double> cost_list(candidate_states.size(), std::numeric_limits<float>::infinity());
    vector<Plan> plan_list(candidate_states.size());
    for (size_t i = 0; i < cost_list.size(); ++i)
    {
        auto plan = GeneratePlanForState(candidate_states[i]);
        if(!plan.path.x.empty()) {
            debug_cost = true;
            printf("PLAN %s\n", states[candidate_states[i]]);
            cost_list[i] = CostForTrajectory(plan);
            printf("TOTAL: %f\n", cost_list[i]);
            debug_cost = false;
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

//    if (next_state != planner_state)
//    {
//        printf("%s: lane_target %ld speed_target %f\n",
//               states[next_state],
//               plan_list[minimum_i].lane_target,
//               plan_list[minimum_i].speed_target
//        );
//    }

    planner_state = next_state;
    current_plan = plan_list[minimum_i];
    return plan_list[minimum_i].path;
}

Path PathPlanner::GenerateTrajectory(double t_final, double s_final, double d_final,
                                     double speed_final) const
{
    double x_initial;
    if (!previousPath.x.empty())
    {
        x_initial = previousPath.x.front();
    }
    else
    {
        x_initial = vehicle_state.x;
    }

    double y_initial;
    if (!previousPath.y.empty())
    {
        y_initial = previousPath.y.front();
    }
    else
    {
        y_initial = vehicle_state.y;
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
    auto xy_final = getXY(s_final, d_final, mapData.waypoints_s,
                          mapData.waypoints_x, mapData.waypoints_y);

    double x_final = xy_final[0];
    double y_final = xy_final[1];

    auto last_waypoint_i = ClosestWaypoint(x_final, y_final,
                                           mapData.waypoints_x,
                                           mapData.waypoints_y);

    auto vx_final = -speed_final * mapData.waypoints_dy[last_waypoint_i];
    auto vy_final = speed_final * mapData.waypoints_dx[last_waypoint_i];

    auto x_curve = JerkMinimalTrajectory({x_initial, vx_initial, ax_initial},
                                    {x_final, vx_final, ay_initial},
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
    this->sensorFusionData = std::move(sensorFusionData);
}

void PathPlanner::UpdateHistory(Path previousPath)
{
    this->previousPath = std::move(previousPath);
}

Plan PathPlanner::GeneratePlanForState(uint8_t state) const
{
    long lane;
    double speed;
    if (state == 1)  // FOLLOW
    {
        lane = lane_actual;
        speed = SafeSpeedForLane(lane);
    }
    else if(state == 2) // CHANGE_LEFT
    {
        lane = lane_actual - 1;
        speed = SafeSpeedForLane(lane);
    }
    else if(state == 3)  // CHANGE_RIGHT
    {
        lane = lane_actual + 1;
        speed = SafeSpeedForLane(lane);
    }
    else  // EMERGENCY_STOP
    {
        assert(state == 4);
        lane = lane_actual;
        speed = 0;
    }

    double t_final = 1.0;
    double s_final = vehicle_state.s + speed * t_final;
    double d_final = LANE_WIDTH / 2 + lane * LANE_WIDTH;
    return {
        GenerateTrajectory(t_final, s_final, d_final, speed),
        lane_actual,
        lane,
        speed
    };
}

double PathPlanner::SafeSpeedForLane(long lane) const
{
    auto car_in_front = FindCarToFollow(lane);
    double speed;
    if ((car_in_front != sensorFusionData.id.size())
        && (abs(sensorFusionData.s[car_in_front] - vehicle_state.s) <
           FOLLOW_DISTANCE))
    {
        speed = sqrt(
            sensorFusionData.vx[car_in_front] *
            sensorFusionData.vx[car_in_front]
            + sensorFusionData.vy[car_in_front] *
              sensorFusionData.vy[car_in_front]);
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

double PathPlanner::CostForTrajectory(const Plan& plan) const
{
    double speed_cost = 1 - plan.speed_target / speed_limit;

    double valid_lane = valid_lane_cost(plan.lane_target);

    double car_avoidance_cost = CarAvoidanceCost(plan.path) * 1.0;

    vector<double> costs = {
        4.0 * speed_cost,
        100000 * valid_lane,
        0.8 * keep_right(plan.lane_current, plan.lane_target),
        2.0 * car_avoidance_cost,
        1 - exp(-abs(2 * lane_actual - plan.lane_current - plan.lane_target))
    };

    if (debug_cost) {
        std::cout << "costs:\n";
        for (auto&& item : costs)
        {
            std::cout << item << '\n';
        }
    }

    return std::accumulate(costs.begin(), costs.end(), 0.0);
}

size_t PathPlanner::FindCarToFollow(long lane) const
{
    auto closest = sensorFusionData.id.size();
    double closest_s = numeric_limits<double>::infinity();
    double d = LANE_WIDTH / 2 + lane * LANE_WIDTH;
    for (size_t i = 0; i < sensorFusionData.id.size(); ++i)
    {
        if (abs(sensorFusionData.d[i] - d) < LANE_WIDTH / 2) {
            if (sensorFusionData.s[i] > vehicle_state.s) {
                if (sensorFusionData.s[i] < closest_s) {
                    closest = i;
                    closest_s = sensorFusionData.s[i];
                }
            }
        }
    }

    return closest;
}

double PathPlanner::CarAvoidanceCost(const Path& path) const
{
    double cost = 0;
    for (size_t i = 0; i < sensorFusionData.id.size(); i++)
    {
        cost += CarAvoidanceCostPerCar(path, i);
    }
    return cost / sensorFusionData.id.size();
}

double PathPlanner::CarAvoidanceCostPerCar(const Path& path, size_t car_id) const
{
//    assert(CarPotential(0, 0, 0, 0, 0, 0, 0, 0) == 1.0);
//    assert(CarPotential(0, 0, 0, 0, 0, 0, 1, 1) == 1.0);
//
//    assert(CarPotential( LANE_WIDTH, 0, 0, 0, 0, 0, 0, 1) == 0.0);
//    assert(CarPotential(-LANE_WIDTH, 0, 0, 0, 0, 0, 0, 1) == 0.0);
//
//    assert(CarPotential( LANE_WIDTH / 2, 0, 0, 0, 0, 0, 0, 1) == 1.0);
//    assert(CarPotential(-LANE_WIDTH / 2, 0, 0, 0, 0, 0, 0, 1) == 1.0);
//
//    assert(CarPotential(0, LANE_WIDTH, 0, 0, 0, 0, 0, 1) == 1.0);
//    assert(CarPotential(0, LANE_WIDTH, 0, 0, 0, 0, 0, 1) == 1.0);
//
//    assert(CarPotential(0,  FOLLOW_DISTANCE, 0, 0, 0, 0, 0, 1) == 0.0);
//    assert(CarPotential(0, -FOLLOW_DISTANCE, 0, 0, 0, 0, 0, 1) == 0.0);

    double cost = 0;
    double t = path.x.size() * dt;
    double car_speed = sqrt(pow(sensorFusionData.vx[car_id], 2)
                            + pow(sensorFusionData.vy[car_id], 2));
    double d_car = sensorFusionData.d[car_id];

    auto car_path = GenerateTrajectory(
        t, sensorFusionData.s[car_id] + car_speed + t, d_car, car_speed,
        sensorFusionData.x[car_id], sensorFusionData.y[car_id],
        sensorFusionData.vx[car_id], sensorFusionData.vy[car_id],
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
                 + pow(forward / CAR_LENGTH, 2)));
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
