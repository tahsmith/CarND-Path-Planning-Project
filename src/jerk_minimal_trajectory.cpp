//
// Created by Timothy Smith on 3/4/18.
//

#include "jerk_minimal_trajectory.hpp"

#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Polynomial::Polynomial(std::initializer_list<double> coefficients) :
    coefficients(begin(coefficients), end(coefficients))
{}

double Polynomial::Evaluate(double x) const
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

Polynomial Polynomial::Differentiate() const
{
    vector<double> new_coefficients(coefficients.size() - 1, 0.0);
    for (int i = 1; i < coefficients.size(); ++i)
    {
        new_coefficients[i - 1] = coefficients[i] * i;
    }
    return Polynomial{new_coefficients};
}

Polynomial::Polynomial(std::vector<double> coefficients) : coefficients(std::move(coefficients))
{}

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

#include "catch.hpp"

TEST_CASE("Polynomial")
{
    SECTION("x^2 - 2x + 1")
    {
        Polynomial polynomial{1, -2, 1};

        REQUIRE(polynomial.Evaluate(0) == 1.0);
        REQUIRE(polynomial.Evaluate(0.5) == 0.25);
        REQUIRE(polynomial.Evaluate(1) == 0.0);
        REQUIRE(polynomial.Evaluate(0.5) == 0.25);
        REQUIRE(polynomial.Evaluate(2) == 1.0);
    }

    SECTION("-x^2 + 2x - 1")
    {
        Polynomial polynomial{-1, 2, -1};

        REQUIRE(polynomial.Evaluate(0) == -1.0);
        REQUIRE(polynomial.Evaluate(0.5) == -0.25);
        REQUIRE(polynomial.Evaluate(1) == 0.0);
        REQUIRE(polynomial.Evaluate(0.5) == -0.25);
        REQUIRE(polynomial.Evaluate(2) == -1.0);
    }
}

TEST_CASE("JerkMinimalTrajectory")
{
    SECTION("Trivial")
    {
        auto poly = JerkMinimalTrajectory({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                                          1.0);
        REQUIRE(poly.Evaluate(0.0) == 0.0);
        REQUIRE(poly.Evaluate(0.5) == 0.0);
        REQUIRE(poly.Evaluate(1.0) == 0.0);
    }

    SECTION("1 -> 1")
    {
        auto poly = JerkMinimalTrajectory({1.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
                                          1.0);
        REQUIRE(poly.Evaluate(0.0) == Approx(1.0));
        REQUIRE(poly.Evaluate(0.5) == Approx(1.0));
        REQUIRE(poly.Evaluate(1.0) == Approx(1.0));
    }

    SECTION("0 -> 1")
    {
        auto poly = JerkMinimalTrajectory({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
                                          1.0);
        REQUIRE(poly.Evaluate(0.0) == Approx(0.0));
        REQUIRE(poly.Evaluate(0.5) == Approx(0.5));
        REQUIRE(poly.Evaluate(1.0) == Approx(1.0));
    }

    SECTION("1 -> 0")
    {
        auto poly = JerkMinimalTrajectory({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0},
                                          1.0);
        REQUIRE(poly.Evaluate(0.0) == 1.0);
        REQUIRE(poly.Evaluate(0.5) == Approx(0.5));
        REQUIRE(poly.Evaluate(1.0) <= 1e-6);
    }
}