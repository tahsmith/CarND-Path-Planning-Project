//
// Created by Timothy Smith on 13/4/18.
//

#include <tuple>
#include <cmath>
#include <cassert>

#include "map_data.hpp"
#include "utilities.hpp"

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

    return tuple<double, double> {
        i - 1,
        s - s0
    };
}

tuple<double, double> MapData::InterpolateRoadTangent(double s) const
{
    size_t i;
    double l;
    tie(i, l) = InterpolationPoint(s);

    return tuple<double, double> {
        -dy_curves[i].Evaluate(l),
        dx_curves[i].Evaluate(l)
    };
}

tuple<double, double> MapData::InterpolateRoadCoords(double s, double d) const
{
    size_t i;
    double l;
    tie(i, l) = InterpolationPoint(s);

    return tuple<double, double> {
        x_curves[i].Evaluate(l) + dx_curves[i].Evaluate(l) * d,
        y_curves[i].Evaluate(l) + dy_curves[i].Evaluate(l) * d,
    };
}

vector<Polynomial> GeneratePolys(vector<double> x, vector<double> t, double t_last) {
    vector<Polynomial> poly_list;
    auto n_points = x.size();
    if (n_points < 4) {
        return {};
    }
    double x0 = x[n_points - 1];
    double x1;
    double x2;
    double x3;

    // poly_list[i] is the curve from x[i] to x[i + 1]
    for(long i = 0; i < n_points; ++i) {
        x1 = x[i];
        x2 = x[(i + 1) % n_points];
        x3 = x[(i + 2) % n_points];

        double t0 = t[i];
        double t1;
        if (i < n_points - 1) {
            t1 = t[i + 1];
        }
        else {
            t1 = t_last;
        }
        double dt = t1 - t0;
        assert(dt > 0.0);

        // Not necessarily a "velocity", just a tangent.
        double v0 = (x2 - x0) / 2.0 / dt;
        double v1 = (x3 - x1) / 2.0 / dt;
        double a = (v1 - v0) / dt;
        Polynomial poly = JerkMinimalTrajectory({x1, v0, a}, {x2, v1, a}, dt);
        poly_list.push_back(move(poly));
        x0 = x1;
    }

    return poly_list;
}

void MapData::PrepareInterpolation()
{
    x_curves = GeneratePolys(waypoints_x, waypoints_s, max_s);
    y_curves = GeneratePolys(waypoints_y, waypoints_s, max_s);
    dx_curves = GeneratePolys(waypoints_dx, waypoints_s, max_s);
    dy_curves = GeneratePolys(waypoints_dy, waypoints_s, max_s);

    // Do some sanity checks on the the results.
    for (size_t i = 1; i < waypoints_s.size(); ++i) {
        double x0 = waypoints_x[i - 1];
        double x;
        double x1 = waypoints_x[i];
        double y0 = waypoints_y[i - 1];
        double y;
        double y1 = waypoints_y[i];
        double dx0 = waypoints_dx[i - 1];
        double dx;
        double dx1 = waypoints_dx[i];
        double dy0 = waypoints_dy[i - 1];
        double dy;
        double dy1 = waypoints_dy[i];
        double dt = waypoints_s[i] -  waypoints_s[i - 1];

        x = x_curves[i - 1].Evaluate(dt);
        y = y_curves[i - 1].Evaluate(dt);
        dx = dx_curves[i - 1].Evaluate(dt);
        dy = dy_curves[i - 1].Evaluate(dt);

        assert(abs(x - x1) < 1e-6);
        assert(abs(y - y1) < 1e-6);
        assert(abs(dx - dx1) < 1e-6);
        assert(abs(dy - dy1) < 1e-6);

        x = x_curves[i].Evaluate(0);
        y = y_curves[i].Evaluate(0);
        dx = dx_curves[i].Evaluate(0);
        dy = dy_curves[i].Evaluate(0);

        assert(abs(x - x1) < 1e-6);
        assert(abs(y - y1) < 1e-6);
        assert(abs(dx - dx1) < 1e-6);
        assert(abs(dy - dy1) < 1e-6);
    }
}
