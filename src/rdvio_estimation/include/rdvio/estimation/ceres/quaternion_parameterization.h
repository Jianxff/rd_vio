#pragma once

#include <ceres/ceres.h>
#include <rdvio/types.h>
#include <rdvio/geometry/lie_algebra.h>

namespace rdvio {

struct QuaternionParameterization : public ceres::LocalParameterization {
    bool Plus(const double *q, const double *dq,
              double *q_plus_dq) const override {
        map<quaternion> result(q_plus_dq);
        result = (const_map<quaternion>(q) * expmap(const_map<vector<3>>(dq)))
                     .normalized();
        return true;
    }
    bool ComputeJacobian(const double *, double *jacobian) const override {
        map<matrix<4, 3, true>> J(jacobian);
        J.setIdentity(); // the composited jacobian is computed in
                         // PreIntegrationError::Evaluate(), we simply forward
                         // it.
        return true;
    }
    int GlobalSize() const override { return 4; }
    int LocalSize() const override { return 3; }
};

} // namespace rdvio

