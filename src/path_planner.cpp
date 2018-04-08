//
// Created by Timothy Smith on 2/4/18.
//

#include <utility>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>

#include "path_planner.hpp"
#include "utilities.hpp"

using std::vector;
using std::move;
using std::numeric_limits;

const size_t n_states = 4;
const char* states[n_states] = {
    "START",
    "FOLLOW",
    "CHANGE_LEFT",
    "CHANGE_RIGHT"
};

static const int transitions[n_states][n_states] {
    {0, 1, 0, 0},
    {0, 1, 1, 1},
    {0, 1, 1, 1},
    {0, 1, 1, 1}
};

static const double LANE_WIDTH = 4.0;
static const double FOLLOW_DISTANCE = 30.0;

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
    lane_current{1},
    speed_target{speed_limit},
    planner_state{0}
{}

Path PathPlanner::PlanPath()
{
    lane_current = lround((vehicle_state.d - LANE_WIDTH / 2) / LANE_WIDTH);
    follow_id = FindCarToFollow(0);
    speed_target = SafeSpeedForLane(lane_current);

    printf("%s: lane_current %d follow_id %zd speed_target %f\n",
           states[planner_state],
           lane_current,
           follow_id,
           speed_target
    );

    auto candidate_states = SuccessorStates(planner_state);
    std::vector<double> cost_list(candidate_states.size(), std::numeric_limits<float>::infinity());
    vector<Plan> plan_list(candidate_states.size());
    for (size_t i = 0; i < cost_list.size(); ++i)
    {
        auto plan = GeneratePlanForState(candidate_states[i]);
        if(plan.path.x.size() > 0) {
            cost_list[i] = CostForTrajectory(i, plan);
            plan_list[i] = move(plan);
        }
    }

    double minimum = cost_list[0];
    size_t minimum_i = 0;
    for(int i = 1; i < cost_list.size(); ++i) {
        if(cost_list[i] < minimum) {
            minimum = cost_list[i];
            minimum_i = i;
        }
    }
    auto next_state = candidate_states[minimum_i];

    if (next_state != planner_state)
    {
        printf("%s: lane_target %d speed_target %f\n",
               states[next_state],
               plan_list[minimum_i].lane_target,
               plan_list[minimum_i].speed_target
        );
    }
    planner_state = next_state;
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
    Polynomial x_curve{};
    Polynomial y_curve{};
    GenerateTrajectory(t_final, s_final, d_final, speed_final, x_initial,
                       y_initial, vx_initial, vy_initial, ax_initial,
                       ay_initial, x_curve,
                       y_curve);

    Path path{};
    auto n_points = lround(floor(t_final / dt));
    for (int i = 0; i < n_points; i++)
    {
        double t = i * dt;
        double x = x_curve.Evaluate(t);
        double y = y_curve.Evaluate(t);

        path.x.push_back(x);
        path.y.push_back(y);
    }

    return path;
}

void
PathPlanner::GenerateTrajectory(double t_final, double s_final, double d_final,
                                double speed_final,
                                double x_initial, double y_initial,
                                double vx_initial, double vy_initial,
                                double ax_initial, double ay_initial,
                                Polynomial& x_curve, Polynomial& y_curve) const
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

    x_curve = JerkMinimalTrajectory({x_initial, vx_initial, ax_initial},
                                    {x_final, vx_final, ay_initial},
                                    t_final);
    y_curve = JerkMinimalTrajectory({y_initial, vy_initial, 0},
                                    {y_final, vy_final, 0},
                                    t_final);
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
    if (state == 1)  // FOLLOW
    {
        lane = lane_current;
    }
    else if(state == 2) // CHANGE_LEFT
    {
        lane = lane_current - 1;
    }
    else  // CHANGE_RIGHT
    {
        assert(state == 3);
        lane = lane_current + 1;
    }

    double speed = SafeSpeedForLane(lane);
    double t_final = 1.0;
    double s_final = vehicle_state.s + speed * t_final;
    double d_final = LANE_WIDTH / 2 + lane * LANE_WIDTH;
    return {
        GenerateTrajectory(t_final, s_final, d_final, speed),
        lane,
        speed
    };
}

double PathPlanner::SafeSpeedForLane(size_t lane) const
{
    double follow_distance = 30;
    auto car_in_front = FindCarToFollow(lane);
    double speed;
    if ((car_in_front != sensorFusionData.id.size())
        && (abs(sensorFusionData.s[car_in_front] - vehicle_state.s) <
           follow_distance))
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

double lane_change_cost(long current, long target) {
    return abs(current - target);
}

double over_take_on_inside_lane_only(double target) {
    return abs(2.0 - target) / 2.0;
}

double speed_limit_cost(double speed, double limit) {
    if (speed >= limit) {
        return 1;
    }
    return 0;
}

double PathPlanner::CostForTrajectory(uint8_t state, const Plan& plan) const
{
    double speed_cost = 1 - plan.speed_target / speed_limit;

    double valid_lane = valid_lane_cost(plan.lane_target);

    double safe_lane_cost = over_take_on_inside_lane_only(plan.lane_target) * 0.1;

    double car_avoidance_cost = CarAvoidanceCost(plan.path) * 1.0;

//    printf("speed_cost         = %f\n"
//           "valid_lane   = %f\n"
//           "safe_lane_cost     = %f\n"
//           "car_avoidance_cost = %f\n",
//           speed_cost,
//           valid_lane,
//           safe_lane_cost,
//           car_avoidance_cost
//           );

    double cost = speed_limit_cost(plan.speed_target, speed_limit) * 0
                  + speed_cost
                  + valid_lane * 1000
                  + safe_lane_cost
//                  + car_avoidance_cost
    ;
    return cost;
}

size_t PathPlanner::FindCarToFollow(size_t lane) const
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
    return cost;
}

double PathPlanner::CarAvoidanceCostPerCar(const Path& path, size_t car_id) const
{
    assert(CarPotential(0, 0, 0, 0, 0, 0) == 1.0);
    assert(CarPotential(0, 0, 0, 0, 1, 1) == 1.0);

    assert(CarPotential( LANE_WIDTH, 0, 0, 0, 0, 1) == 0.0);
    assert(CarPotential(-LANE_WIDTH, 0, 0, 0, 0, 1) == 0.0);

    assert(CarPotential( LANE_WIDTH / 2, 0, 0, 0, 0, 1) == 1.0);
    assert(CarPotential(-LANE_WIDTH / 2, 0, 0, 0, 0, 1) == 1.0);

    assert(CarPotential(0, LANE_WIDTH, 0, 0, 0, 1) == 1.0);
    assert(CarPotential(0, LANE_WIDTH, 0, 0, 0, 1) == 1.0);

    assert(CarPotential(0,  FOLLOW_DISTANCE, 0, 0, 0, 1) == 0.0);
    assert(CarPotential(0, -FOLLOW_DISTANCE, 0, 0, 0, 1) == 0.0);

    double cost = 0;
    double t = path.x.size() * dt;
    double car_speed = sqrt(pow(sensorFusionData.vx[car_id], 2)
                            + pow(sensorFusionData.vy[car_id], 2));
    double d_car = round(sensorFusionData.vx[car_id]);

    Polynomial car_x_curve{};
    Polynomial car_y_curve{};
    GenerateTrajectory(
        t, sensorFusionData.s[car_id] + car_speed + t, d_car, car_speed,
        sensorFusionData.x[car_id], sensorFusionData.y[car_id],
        sensorFusionData.vx[car_id], sensorFusionData.vy[car_id],
        0.0, 0.0,
        car_x_curve, car_y_curve
    );

    Polynomial car_vx_curve = car_x_curve.Differentiate();
    Polynomial car_vy_curve = car_y_curve.Differentiate();

    for (size_t i = 1; i < path.x.size(); i++) {
        cost = fmax(CarPotential(
            path.x[i], path.y[i],
            car_x_curve.Evaluate(i * dt),
            car_y_curve.Evaluate(i * dt),
            car_vx_curve.Evaluate(i * dt),
            car_vy_curve.Evaluate(i * dt)
        ), cost);
    }
    return cost;
}

double PathPlanner::CarPotential(double x, double y, double car_x, double car_y,
                                 double car_vx, double car_vy) const
{
    double dx = x - car_x;
    double dy = y - car_y;
    double r = sqrt(dx * dx + dy * dy);
    if (r == 0) {
        return 1;
    }
    double rx = dx / r;
    double ry = dy / r;

    double rv = sqrt(car_vx * car_vx + car_vy * car_vy);
    if (r == 0) {
        if (r < LANE_WIDTH) {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    double rvx = car_vx / rv;
    double rvy = car_vy / rv;
    double angle = acos(rx * rvx + ry * rvy);
    double r_wall = FOLLOW_DISTANCE / 4 * cos(angle)
                    + LANE_WIDTH * 0.8 * sin(angle);

    if (r < r_wall) {
        return 1;
    }
    else {
        return 0;
    }
}

#include "catch.hpp"

TEST_CASE("car_potential") {
    SECTION("on top") {
        REQUIRE(car_potential(0, 0, 0, 0, 0) == 0.0);
    }
}
