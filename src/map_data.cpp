//
// Created by Timothy Smith on 13/4/18.
//

#include <tuple>
#include <cmath>

#include "map_data.hpp"

using std::make_tuple;
using std::move;
using std::tie;
using std::vector;
using std::tuple;

tuple<size_t, double> MapData::InterpolationPoint(double s) const
{
    s = fmod(s, max_s);
    size_t i = 1;
    for (; i < waypoints_s.size(); ++i)
    {
        if (waypoints_s[i] > s) {
            break;
        }
    }

    double s0 = waypoints_s[i - 1];
    double s1;
    if (i == waypoints_s.size())
    {
        s1 = max_s;
    }
    else {
        s1 = waypoints_s[i];;
    }

    double l = (s - s0) / (s1 - s0);

    return {
        i - 1,
        l
    };
}

tuple<double, double> MapData::InterpolateRoadTangent(double s) const
{
    size_t i;
    double l;
    tie(i, l) = InterpolationPoint(s);

    return {
        -dy_curves[i].Evaluate(l),
        dx_curves[i].Evaluate(l)
    };
}

tuple<double, double> MapData::InterpolateRoadCoords(double s, double d) const
{
    size_t i;
    double l;
    tie(i, l) = InterpolationPoint(s);

    return {
        x_curves[i].Evaluate(l) + dx_curves[i].Evaluate(l) * d,
        y_curves[i].Evaluate(l) + dy_curves[i].Evaluate(l) * d,
    };
}

vector<Polynomial> GeneratePolys(vector<double> point_list) {
    vector<Polynomial> poly_list;
    auto n_points = point_list.size();
    double x0 = point_list[n_points - 1];
    double x1;
    double x2;
    double x3;

    for(long i = 0; i < n_points; ++i) {
        x1 = point_list[i];
        x2 = point_list[(i + 1) % n_points];
        x3 = point_list[(i + 2) % n_points];

        // Not necessarily a "velocity", just a tangent.
        double v0 = (x2 - x0) / 2.0;
        double v1 = (x3 - x1) / 2.0;
        double a = v1 - v0;
        Polynomial poly = JerkMinimalTrajectory({x1, v0, a}, {x2, v1, a}, 1.0);
        poly_list.push_back(move(poly));
        x0 = x1;
    }

    return poly_list;
}

void MapData::PrepareInterpolation()
{
    x_curves = GeneratePolys(waypoints_x);
    y_curves = GeneratePolys(waypoints_y);
    s_curves = GeneratePolys(waypoints_s);
    dx_curves = GeneratePolys(waypoints_dx);
    dy_curves = GeneratePolys(waypoints_dy);
}
