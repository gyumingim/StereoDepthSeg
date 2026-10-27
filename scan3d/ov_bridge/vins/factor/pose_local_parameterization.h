#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

// ovswba 2026-10-26: Ceres 2.2 removed ceres::LocalParameterization -> the same parameterization as a ceres::Manifold.
// Plus and the 7x6 lifting Jacobian (PlusJacobian = old ComputeJacobian) are unchanged from VINS-Mono. Minus /
// MinusJacobian are new (required by the Manifold interface; the inverse of Plus to first order) and are not used by
// the trust-region solve VINS runs.
class PoseLocalParameterization : public ceres::Manifold
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool PlusJacobian(const double *x, double *jacobian) const;  // ovswba: = VINS ComputeJacobian
    virtual bool Minus(const double *y, const double *x, double *y_minus_x) const;  // ovswba: new
    virtual bool MinusJacobian(const double *x, double *jacobian) const;  // ovswba: new
    virtual int AmbientSize() const { return 7; };  // ovswba: = GlobalSize
    virtual int TangentSize() const { return 6; };  // ovswba: = LocalSize
};
