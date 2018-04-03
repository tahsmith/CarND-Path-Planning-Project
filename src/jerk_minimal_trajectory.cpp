//
// Created by Timothy Smith on 3/4/18.
//

#include "jerk_minimal_trajectory.hpp"

#include <utility>

#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Polynomial::Polynomial(std::initializer_list<double> coefficients) :
    coefficients(begin(coefficients), end(coefficients))
{}

double Polynomial::Evaluate(double x)
{
    double sum = 0;
    double x_power = 1;
    for (auto&& coefficient : coefficients)
    {
        sum += x_power * coefficient;
        x_power *= x;
    }
    return sum;
}

Polynomial
JerkMinimalTrajectory(std::vector<double> initial, std::vector<double> final,
                      double T)
{
    VectorXd C(3);
    C <<
      final[0] - (initial[0] + initial[1] * T + 0.5 * initial[2] * T * T),
        final[1] - (initial[1] + initial[2] * T),
        final[2] - initial[2];
    MatrixXd A(3, 3);
    A <<
      pow(T, 3), pow(T, 4), pow(T, 5),
        3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
        6 * pow(T, 1), 12 * pow(T, 2), 20 * pow(T, 3);

    VectorXd alpha = A.inverse() * C;
    return {initial[0], initial[1], 0.5 * initial[2], alpha[0], alpha[1],
            alpha[2]};

}
