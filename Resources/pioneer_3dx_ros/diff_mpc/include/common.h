#ifndef COMMON_DIFF_MPC_H
#define COMMON_DIFF_MPC_H

#include <map>
#include <string>
#include <Eigen/Core>
#include <Eigen/QR>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace common
{

// If exists, load parameter from map
// else, don't change the value of the parameter
template <typename ParamType>
void LoadParameter(
    const char *const param_name,
    const std::map<std::string, double>& params,
    ParamType& param)
{
    if (params.find(param_name) != params.end())
    {
        param = static_cast<ParamType>(params.at(param_name));
    }
}

// Fit a polynomial on (x, y) points
static Eigen::VectorXd PolyFit(
    const Eigen::VectorXd& xvals,
    const Eigen::VectorXd& yvals,
    size_t order)
{
    if (xvals.size() != yvals.size())
    {
        ROS_ERROR("The vectors of x and y values must have the same size.");
        return {};
    }

    if (order == 0U)
    {
        ROS_ERROR("The order of the polynomial should be positive.");
        return {};
    }

    Eigen::MatrixXd A(xvals.size(), order);
    for (size_t i = 0U; i < xvals.size(); ++i)
    {
        for (size_t j = 1U; j <= order; ++j)
        {
            A(i, j - 1U) = pow(xvals[i], j);
        }
    }

    return A.householderQr().solve(yvals);
}

// Evaluate the polynomial at x
static double PolyEval(
    Eigen::VectorXd coeffs,
    double x)
{
    double result = 0.;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i + 1U);
    }

    return result;
}

// Evaluate first derivative of the polynomial at x
static double PolyDerivativeEval(
    Eigen::VectorXd coeffs,
    double x)
{
    double result = 0.;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += (i + 1U) * coeffs[i] * pow(x, i);
    }

    return result;
}


} // common

#endif
