/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <sDiff/sDiff.h>
#include <Eigen/Geometry>
#include <iostream>
#include <chrono>


int main() {
    using namespace sDiff;

    Variable quaternion(4, "q");

    Variable x(3, "x");

    Expression rotation;

    //Rodriguez formula
    rotation = Identity(3,3) + 2.0 * quaternion(0,0) * quaternion.block(1,0,3,1).skew() + 2.0 * quaternion.block(1,0,3,1).skew() * quaternion.block(1,0,3,1).skew();

    std::cerr << rotation.name() << std::endl;

    Eigen::Vector4d quaternionValue;

    quaternionValue.setZero();
    quaternionValue[0] = 1.0;

    quaternion = quaternionValue;

    Eigen::MatrixXd testRotation(3,3);
    testRotation.setIdentity();

    assert(rotation.evaluate() == testRotation);

    quaternionValue = Eigen::Vector4d::Random();
    quaternionValue.normalize();

    quaternion = quaternionValue;

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> skew;
    skew << 0.0, -quaternionValue[3], quaternionValue[2],
          quaternionValue[3], 0.0, -quaternionValue[1],
         -quaternionValue[2], quaternionValue[1], 0.0;

    testRotation = Eigen::MatrixXd::Identity(3,3) + 2.0 * quaternionValue(0) * skew + 2.0 * skew * skew;

    assert(rotation.evaluate() == testRotation);

    Eigen::Quaterniond quaternionEigen;
    quaternionEigen.w() = quaternionValue[0];
    quaternionEigen.x() = quaternionValue[1];
    quaternionEigen.y() = quaternionValue[2];
    quaternionEigen.z() = quaternionValue[3];

    assert(rotation.evaluate() == quaternionEigen.toRotationMatrix());

    Eigen::Matrix<double, 3, 3> testSpeed;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    testSpeed = quaternionEigen.toRotationMatrix();
    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
    std::cout << "Elapsed time us (eigen): " << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) <<std::endl;

    begin = std::chrono::steady_clock::now();
    testSpeed = rotation.evaluate();
    end= std::chrono::steady_clock::now();
    std::cout << "Elapsed time us (evaluate): " << (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) <<std::endl;

    //-------------------------Validation of first derivative

    double perturbationValue = 1e-3;
    Eigen::Vector4d quaternionPerturbed, perturbation;
    Eigen::Vector3d vector, output, perturbedOutput, firstOrderTaylor;
    vector = Eigen::Vector3d::Random() * 10.0;
    Eigen::Matrix<double, 3, 3> perturbedRotation;

    Eigen::Vector4d maxQuaternion;
    maxQuaternion.setConstant(1.0);
    Eigen::Vector4d minQuaternion;
    minQuaternion.setConstant(-1);
    minQuaternion(0) = 0;

    Expression rotatedVector = rotation * x;

    x = vector;

    output = rotatedVector.evaluate();

    Expression rotatedVectorDerivative = rotatedVector.getColumnDerivative(0, quaternion);

    Eigen::MatrixXd derivativeValue = rotatedVectorDerivative.evaluate();

    std::cerr << rotatedVectorDerivative.name() << std::endl;


    // Test separetly the derivative of quaternion
    for (unsigned int i = 0; i < 4; i++)
    {
        quaternionPerturbed = quaternionValue;
        quaternionPerturbed(i) = quaternionPerturbed(i) + perturbationValue;

        //ensure validity of obtained quaternion even if the quaternion is no more a step different than
        //the original
        quaternionPerturbed = quaternionPerturbed.array().min(maxQuaternion.array());
        quaternionPerturbed = quaternionPerturbed.array().max(minQuaternion.array());
        quaternionPerturbed.normalize();

        perturbation = quaternionPerturbed - quaternionValue;

        quaternion = quaternionPerturbed;

        perturbedOutput = rotatedVector.evaluate();

        firstOrderTaylor = derivativeValue * perturbation;

        firstOrderTaylor += output;

        assert ((firstOrderTaylor - perturbedOutput).cwiseAbs().maxCoeff() < perturbationValue/50.0);

    }

    return 0;
}
