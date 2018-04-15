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

inline std::vector<double> diff(std::vector<double> x_list, double dt) {
    std::vector<double> v_list{};
    v_list.reserve(x_list.size() - 1);
    for (size_t i = 0; i < x_list.size() - 1; ++i) {
        double x0 = x_list[i];
        double x1 = x_list[i + 1];
        double v = (x1 - x0) / dt;
        v_list.push_back(v);
    }
    return v_list;
}

template<typename L, typename Op>
bool all_of_list(const L& l, const Op& op) {
    for (auto&& x : l) {
        if(!op(x)) {
            return false;
        }
    }
    return true;
};

template<typename T>
std::function<bool(T)> less_than(T value) {
    return [=] (const T& other) {
        return other < value;
    };
}



#endif //PATH_PLANNING_UTILITIES_HPP
