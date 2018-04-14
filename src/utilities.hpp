//
// Created by Timothy Smith on 2/4/18.
//

#ifndef PATH_PLANNING_UTILITIES_HPP
#define PATH_PLANNING_UTILITIES_HPP

#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

// For converting back and forth between radians and degrees.
constexpr double pi()
{ return M_PI; }

constexpr double deg2rad(double x)
{ return x * pi() / 180; }

constexpr double rad2deg(double x)
{ return x * 180 / pi(); }

inline double length(double x, double y) {
    return sqrt(x * x + y * y);
}

inline double distance(double x1, double y1, double x2, double y2)
{
    return length(x2 - x1, y2 - y1);
}

#endif //PATH_PLANNING_UTILITIES_HPP
