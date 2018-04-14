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
    double s1 = waypoints_s[i];

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

    double v0 = point_list[0] - point_list[point_list.size()];
    for(long i = 0; i < point_list.size() - 1; ++i) {
        double x0 = point_list[i];
        double x1 = point_list[i + 1];

        // Not necessarily a "velocity", just a tangent.
        double v1 = x1 - x0;
        double a = v1 - v0;
        v0 = v1;
        Polynomial poly = JerkMinimalTrajectory({x0, v0, a}, {x1, v1, a}, 1.0);
        poly_list.push_back(move(poly));
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
