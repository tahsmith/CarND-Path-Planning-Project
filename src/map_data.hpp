//
// Created by Timothy Smith on 2/4/18.
//

#ifndef PATH_PLANNING_MAP_DATA_HPP
#define PATH_PLANNING_MAP_DATA_HPP

#include <vector>

class MapData
{
public:
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;
    double max_s;
};

#endif //PATH_PLANNING_MAP_DATA_HPP
