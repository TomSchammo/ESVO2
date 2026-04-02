// #pragma once
#ifndef ESVO2_CORE_FACTOR_POSELOCAL_H
#define ESVO2_CORE_FACTOR_POSELOCAL_H

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <esvo2_core/factor/utility.h>
namespace esvo2_core
{
namespace factor
{
class PoseLocalParameterization : public ceres::Manifold
{
public:
    ~PoseLocalParameterization() {};
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const
    {
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
    } ;
    bool PlusJacobian(const double *x, double *jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        j.topRows<6>().setIdentity();
        j.bottomRows<1>().setZero();

    return true;
    };
    bool Minus(const double *y, const double *x, double *y_minus_x) const
    {
        Eigen::Map<const Eigen::Vector3d> p_y(y);
        Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);
        Eigen::Map<const Eigen::Vector3d> p_x(x);
        Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);
        Eigen::Map<Eigen::Matrix<double, 6, 1>> delta(y_minus_x);
        delta.head<3>() = p_y - p_x;
        Eigen::Quaterniond q_diff = q_x.inverse() * q_y;
        delta.tail<3>() = 2.0 * q_diff.vec() / q_diff.w();
        if (q_diff.w() < 0) { delta.tail<3>() = -delta.tail<3>(); }
        return true;
    }
    bool MinusJacobian(const double *x, double *jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
        j.setZero();
        j.leftCols<6>().setIdentity();
        return true;
    }
    int AmbientSize() const { return 7; };
    int TangentSize() const  { return 6; };
};
}
}

#endif
