/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_VARIABLE_H
#define SDIFF_VARIABLE_H

#include <Eigen/Core>
#include <string>
namespace sDiff {

    template <typename Vector = Eigen::VectorXd>
    class Variable;
}

template <typename Vector>
class sDiff::Variable{

    Vector m_values;
    std::string m_name;

    public:

    Variable(Eigen::Index dimension, const std::string name) {
        static_assert (Vector::ColsAtCompileTime == 1, "The chosen VectorType for the Variable should have exactly one column at compile time.");
    }

};





#endif // SDIFF_VARIABLE_H
