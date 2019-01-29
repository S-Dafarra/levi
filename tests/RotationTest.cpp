/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <sDiff/sDiff.h>
#include <Eigen/Geometry>
#include <iostream>

int main() {
    using namespace Eigen;
    using namespace sDiff;

    Variable quaternion(4, "q");

    Variable x(3, "x");

    Expression rotation;

    rotation = Identity(3,3) + 2.0 * quaternion(0,0) * quaternion.block(1,0,3,1).skew() + 2.0 * quaternion.block(1,0,3,1).skew() * quaternion.block(1,0,3,1).skew();

    std::cerr << rotation.name() << std::endl;

    Vector4d quaternionValue;

    quaternionValue.setZero();
    quaternionValue[0] = 1.0;

    quaternion = quaternionValue;

    MatrixXd testRotation(3,3);
    testRotation.setIdentity();

    assert(rotation.evaluate() == testRotation);

    quaternionValue = Vector4d::Random();
    quaternionValue.normalize();

    quaternion = quaternionValue;

    Matrix<double, 3, 3, Eigen::RowMajor> skew;
    skew << 0.0, -quaternionValue[3], quaternionValue[2],
          quaternionValue[3], 0.0, -quaternionValue[1],
         -quaternionValue[2], quaternionValue[1], 0.0;

    testRotation = MatrixXd::Identity(3,3) + 2.0 * quaternionValue(0) * skew + 2.0 * skew * skew;

    assert(rotation.evaluate() == testRotation);

    Quaterniond quaternionEigen;
    quaternionEigen.w() = quaternionValue[0];
    quaternionEigen.x() = quaternionValue[1];
    quaternionEigen.y() = quaternionValue[2];
    quaternionEigen.z() = quaternionValue[3];

    assert(rotation.evaluate() == quaternionEigen.toRotationMatrix());

    return 0;
}
