//
// Created by Timothy Smith on 13/4/18.
//

#include <tuple>
#include <cmath>

#include "map_data.hpp"

using std::make_tuple;

std::tuple<double, double> MapData::InterpolateRoadTangent(double s) const
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

    auto tx0 = -waypoints_dy[i - 1];
    auto ty0 = waypoints_dx[i - 1];

    auto tx1 = -waypoints_dy[i];
    auto ty1 = waypoints_dx[i];

    return make_tuple(tx0 + l * (tx1 - tx0), ty0 + l * (ty1 - ty0));
}
