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
    MatrixXd A(MatrixXd::Identity(3,3));
    Constant a(MatrixXd::Identity(3,3), "a");
    Matrix<double,3,3> B;
    B.setIdentity();
    Constant b(3,3,"b");

    auto sum  = A + a;

    auto subtraction  = A - a;


    b = B;

    Expression c = a + b;
    c = c * a;
    std::cerr << c.name() << ": " <<std::endl << c.evaluate() <<std::endl;

    Variable x(3,"x"), y(3, "y");

    auto f = x + y;

    Matrix<double,3,3> V;
    V.setIdentity();
    V *= 3;
    Constant v(V, "V");
    Expression g = v * c;
    Expression k = V * g;
    std::cerr << g.name() << " " <<std::endl << g.evaluate() <<std::endl;

    Scalar test(0.5);
    c = test* c;
    std::cerr << c.name() << ": " <<std::endl << c.evaluate() <<std::endl;

    c = 0.5* c;
    std::cerr << c.name() << ": " <<std::endl << c.evaluate() <<std::endl;

    VectorXd x_value(3);
    x_value.setRandom();
    x = x_value;
    ColumnExpression testX = g*x;
    std::cerr << testX.name() << " " <<std::endl << testX.evaluate() << std::endl <<"with x= " << x.evaluate()<<std::endl;
    x_value.setRandom();
    x = x_value;
    std::cerr << testX.name() << " " <<std::endl << testX.evaluate() << std::endl <<"with x= " << x.evaluate()<<std::endl;

    assert(x.evaluate() == x_value);



    return 0;
}
