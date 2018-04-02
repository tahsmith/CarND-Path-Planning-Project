//
// Created by Timothy Smith on 2/4/18.
//

#ifndef PATH_PLANNING_SENSOR_FUSION_DATA_HPP
#define PATH_PLANNING_SENSOR_FUSION_DATA_HPP

#include <vector>

class SensorFusionData
{
public:
    std::vector<int> id;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> s;
    std::vector<double> d;

};

#endif //PATH_PLANNING_SENSOR_FUSION_DATA_HPP
