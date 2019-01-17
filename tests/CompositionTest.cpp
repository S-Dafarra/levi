/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <sDiff/sDiff.h>
#include <iostream>

int main() {
    using namespace Eigen;
    using namespace sDiff;
    ExpressionElement<ConstantEvaluable<>> a(MatrixXd::Identity(3,3), "a");
    Matrix<double,3,3> B;
    B.setIdentity();
    ExpressionElement<ConstantEvaluable<>> b(B, "b");

    Expression c;
    c = a + b;
    auto d = c + a;
    std::cerr << (d).evaluate() <<std::endl;

    Variable<VectorXd> x(2,"x");

    return 0;
}
