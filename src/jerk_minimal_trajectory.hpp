//
// Created by Timothy Smith on 3/4/18.
//

#ifndef PATH_PLANNING_JERK_MINIMAL_TRAJECTORY_HPP
#define PATH_PLANNING_JERK_MINIMAL_TRAJECTORY_HPP

#include <vector>

class Polynomial
{
public:
    Polynomial(std::initializer_list<double> coefficients);
    Polynomial(std::vector<double> coefficients);
    double Evaluate(double x) const;
    Polynomial Differentiate() const;

private:
    std::vector<double> coefficients;
};


Polynomial
JerkMinimalTrajectory(std::vector<double> start, std::vector<double> end,
                      double t);

#endif //PATH_PLANNING_JERK_MINIMAL_TRAJECTORY_HPP
