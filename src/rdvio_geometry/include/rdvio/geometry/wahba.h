#pragma once

#include <rdvio/types.h>

namespace rdvio {

// h(p2) = R * h(p1)
inline matrix<3> solve_rotation_2pt(const std::array<vector<3>, 2> &points1,
                                    const std::array<vector<3>, 2> &points2) {
    matrix<3> cov = matrix<3>::Zero();
    for (size_t i = 0; i < points1.size(); ++i) {
        cov += points1[i] * points2[i].transpose();
    }
    cov = cov * 0.5;
    Eigen::JacobiSVD<matrix<3>> svd(cov,
                                    Eigen::ComputeFullU | Eigen::ComputeFullV);
    const matrix<3> &U = svd.matrixU();
    const matrix<3> &V = svd.matrixV();
    matrix<3> E = matrix<3>::Identity();
    if ((V * U.transpose()).determinant() >= 0.0) {
        E(2, 2) = 1.0;
    } else {
        E(2, 2) = -1.0;
    }
    return V * E * U.transpose();
}

} // namespace rdvio

