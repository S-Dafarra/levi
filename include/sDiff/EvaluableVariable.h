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
#include <cassert>

namespace sDiff {

    template <typename Vector = Eigen::VectorXd>
    class EvaluableVariable;
}

template <typename Vector>
class sDiff::EvaluableVariable {

    template <typename OtherVector>
    friend class EvaluableVariable;

    Vector m_values;
    std::string m_name;

public:

    EvaluableVariable(Eigen::Index dimension, const std::string name) {
        static_assert (Vector::ColsAtCompileTime == 1, "The chosen VectorType for the Variable should have exactly one column at compile time.");

        m_values.resize(dimension);
        m_values.setZero();

        m_name = name;
    }

    template <typename otherVector>
    EvaluableVariable(const EvaluableVariable<otherVector>& other) = delete;

    template <typename otherVector>
    EvaluableVariable(EvaluableVariable<otherVector>&& other) = delete;

    Eigen::Index dimension() const{
        return m_values.size();
    }

    std::string name() const {
        return m_name;
    }

    Eigen::Index rows() const {
        return dimension();
    }

    Eigen::Index cols() const {
        return 1;
    }

    virtual const Vector& evaluate() {
        return m_values;
    }

    template<typename otherVector>
    void operator=(const otherVector& rhs) {
        static_assert (otherVector::ColsAtCompileTime == 1, "The chosen VectorType for the rhs should have exactly one column at compile time.");
        assert(rhs.size() == this->m_values.size());
        this->m_values = rhs;
    }

    template<typename OtherVector>
    void operator=(const EvaluableVariable<OtherVector>& rhs) {
        static_assert (OtherVector::ColsAtCompileTime == 1, "The chosen VectorType for the rhs should have exactly one column at compile time.");
        assert(rhs.dimension() == this->dimension());
        this->m_values = rhs;
    }

    typedef Vector matrix_type;

    static const bool is_variable = true;

};





#endif // SDIFF_VARIABLE_H
