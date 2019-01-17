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
    Constant a(MatrixXd::Identity(3,3), "a");
    Matrix<double,3,3> B;
    B.setIdentity();
    Constant b(3,3,"b");

    b = B;

    Expression c;
    c = a + b;
    auto d = c + a;
    std::cerr << (d).evaluate() <<std::endl;

    VectorXd x_value(2);
    x_value.setRandom();

    Variable x(2,"x"), y(2, "y");

    auto f = x + y;

    Matrix<double,3,1> V;
    V.setIdentity();
    ExpressionElement<ConstantEvaluable<>> v(V, "v");
    //x = v;

    x = x_value;

    assert(x.evaluate() == x_value);

    return 0;
}
