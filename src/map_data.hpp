//
// Created by Timothy Smith on 2/4/18.
//

#ifndef PATH_PLANNING_MAP_DATA_HPP
#define PATH_PLANNING_MAP_DATA_HPP

#include <vector>
#include <tuple>
#include "jerk_minimal_trajectory.hpp"

class MapData
{
public:
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;
    double max_s;

    std::vector<Polynomial> x_curves;
    std::vector<Polynomial> y_curves;
    std::vector<Polynomial> s_curves;
    std::vector<Polynomial> dx_curves;
    std::vector<Polynomial> dy_curves;

    void PrepareInterpolation();

    std::tuple<size_t, double> InterpolationPoint(double s) const;
    std::tuple<double, double> InterpolateRoadTangent(double s) const;
    std::tuple<double, double> InterpolateRoadCoords(double s, double d) const;
};

#endif //PATH_PLANNING_MAP_DATA_HPP
