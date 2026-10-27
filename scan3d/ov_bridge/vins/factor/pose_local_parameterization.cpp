#include "pose_local_parameterization.h"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
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
}
bool PoseLocalParameterization::PlusJacobian(const double *x, double *jacobian) const  // ovswba: was ComputeJacobian
{
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

// ovswba 2026-10-26: Manifold interface (not in VINS-Mono). Inverse of Plus: dp = p_y - p_x, dq = q_x^-1 q_y, and
// Utility::deltaQ(theta) = [1, theta / 2] (normalised after the product) -> theta = 2 * vec(dq) / w(dq).
bool PoseLocalParameterization::Minus(const double *y, const double *x, double *y_minus_x) const
{
    Eigen::Map<const Eigen::Vector3d> py(y), px(x);
    Eigen::Map<const Eigen::Quaterniond> qy(y + 3), qx(x + 3);
    Eigen::Quaterniond dq = qx.inverse() * qy;
    if (dq.w() < 0)
        dq.coeffs() = -dq.coeffs();
    Eigen::Map<Eigen::Vector3d> d(y_minus_x), dth(y_minus_x + 3);
    d = py - px;
    dth = 2.0 * dq.vec() / dq.w();
    return true;
}
bool PoseLocalParameterization::MinusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
    j.setZero();
    j.block<3, 3>(0, 0).setIdentity();
    j.block<3, 3>(3, 3) = 2.0 * Eigen::Matrix3d::Identity();  // at y = x (dq = identity), xyz part of q
    return true;
}
