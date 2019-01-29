/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <sDiff/sDiff.h>
#include <iostream>


int main() {
    using namespace sDiff;

    Identity identity(3,3);

    Expression antiIdentity = Expression::ComposeByRows({identity.row(2), identity.row(1), identity.row(0)}, "Anti-identity");

    std::cerr << antiIdentity.evaluate() << std::endl;

    Variable x(2, "x");
    auto derivative = antiIdentity.getColumnDerivative(1, x);

    Expression antiIdentity2 = Expression::ComposeByCols({identity.col(2), identity.col(1), identity.col(0)}, "Anti-identity2");

    std::cerr << antiIdentity2.evaluate() << std::endl;

    auto derivative2 = antiIdentity2.getColumnDerivative(1, x);

    Variable k(3, "k");

    Expression kSquared = k.transpose() * k;

    Eigen::Vector3d k_values;
    k_values = Eigen::Vector3d::Random();
    k = k_values;

    Eigen::MatrixXd squaredNorm(1,1);
    squaredNorm(0,0) = k_values.squaredNorm();

    assert((kSquared.evaluate() - squaredNorm).cwiseAbs().maxCoeff() < 1e-10);

    Expression kSquaredDerivative = kSquared.getColumnDerivative(0, k);

    assert((kSquaredDerivative.evaluate() - 2*k_values.transpose()).cwiseAbs().maxCoeff() < 1e-10);

    return 0;

}
