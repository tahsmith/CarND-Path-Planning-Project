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
using std::min;

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);

    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

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
    auto xy = getXY(s, d, waypoints_s, waypoints_x, waypoints_y);
    return make_tuple(xy[0], xy[1]);
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
