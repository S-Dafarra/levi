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
    std::cerr << k.name() << " " <<std::endl << k.evaluate() <<std::endl;


    Scalar test(0.5);
    c = test* c;
    std::cerr << c.name() << ": " <<std::endl << c.evaluate() <<std::endl;

    c = 0.5* c + c * 0.5;
    std::cerr << c.name() << ": " <<std::endl << c.evaluate() <<std::endl;

    Expression d = 1.0 * c;

    assert(d.evaluate() == c.evaluate());

    d = 1.0 * d + c;

    VectorXd x_value(3);
    x_value.setRandom();
    x = x_value;
    ColumnExpression testX = g*x;
    std::cerr << testX.name() << " " <<std::endl << testX.evaluate() << std::endl <<"with x= " << x.evaluate()<<std::endl;
    x_value.setRandom();
    x = x_value;
    std::cerr << testX.name() << " " <<std::endl << testX.evaluate() << std::endl <<"with x= " << x.evaluate()<<std::endl;

    assert(x.evaluate() == x_value);

    auto row1 = c.row(0);

    row1.evaluate();

    assert(row1.evaluate() == c.evaluate().row(0));

    auto col1 = c.col(0);

    col1.evaluate();

    assert(col1.evaluate() == c.evaluate().col(0));

    auto testRow = test.row(0);

    auto testCol = test.col(0);

    auto cElement = c(0, 1);
    std::cerr << cElement.name() << " " <<std::endl << cElement.evaluate() <<std::endl;
    assert(cElement.evaluate() == c.evaluate()(0, 1));

    auto testElement = test(0,0);

    auto testBlock = c.block(1,1, 2, 2);
    std::cerr << testBlock.name() << " " <<std::endl << testBlock.evaluate() <<std::endl;
    assert(testBlock.evaluate() == c.evaluate().block(1,1,2,2));

    auto testSkew = x.skew();


    return 0;
}
