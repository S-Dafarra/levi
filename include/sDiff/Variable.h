/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_VARIABLE_H
#define SDIFF_VARIABLE_H

#include <sDiff/ForwardDeclarations.h>

class sDiff::VariableBase {

protected:
    std::string m_name;

    Eigen::Index m_dimension;

    VariableBase(Eigen::Index dimension, const std::string& name)
        : m_name(name)
        , m_dimension(dimension)
    { }

public:

    Eigen::Index dimension() const{
        return m_dimension;
    }

    std::string name() const {
        return m_name;
    }

};

template <typename Vector>
class sDiff::EvaluableVariable<Vector, typename std::enable_if<!std::is_arithmetic<Vector>::value>::type> : public sDiff::VariableBase {

    Vector m_values;

public:

    typedef Vector matrix_type;

    typedef typename Vector::value_type value_type;

    static const Eigen::Index rows_at_compile_time = Vector::RowsAtCompileTime;

    static const Eigen::Index cols_at_compile_time = 1;

    typedef Evaluable<Eigen::Matrix<value_type, Vector::RowsAtCompileTime, Eigen::Dynamic>> derivative_evaluable;


    EvaluableVariable(Eigen::Index dimension, const std::string& name)
        : sDiff::VariableBase(dimension, name)
    {
        static_assert (Vector::ColsAtCompileTime == 1, "The chosen VectorType for the Variable should have exactly one column at compile time.");

        m_values.resize(dimension);
        m_values.setZero();

    }

    template <typename otherVector>
    EvaluableVariable(const EvaluableVariable<otherVector>& other) = delete;

    template <typename otherVector>
    EvaluableVariable(EvaluableVariable<otherVector>&& other) = delete;


    Eigen::Index rows() const {
        return dimension();
    }

    Eigen::Index cols() const {
        return 1;
    }

    virtual const Vector& evaluate() const {
        return m_values;
    }

    template<typename otherVector>
    void operator=(const otherVector& rhs) {
        static_assert (otherVector::ColsAtCompileTime == 1, "The chosen VectorType for the rhs should have exactly one column at compile time.");
        assert(rhs.size() == this->dimension());
        this->m_values = rhs;
    }

    template<typename OtherVector>
    void operator=(const EvaluableVariable<OtherVector>& rhs) {
        static_assert (OtherVector::ColsAtCompileTime == 1, "The chosen VectorType for the rhs should have exactly one column at compile time.");
        assert(rhs.dimension() == this->dimension());
        this->m_values = rhs.evaluate();
    }

};

template <typename Scalar>
class sDiff::EvaluableVariable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> : public sDiff::VariableBase {

    Scalar m_values;

public:

    typedef Scalar matrix_type;

    typedef Scalar value_type;

    typedef Evaluable<Eigen::Matrix<value_type, 1, Eigen::Dynamic>> derivative_evaluable;


    EvaluableVariable(const std::string& name)
    : sDiff::VariableBase(1, name)
    {

        m_values = 0;
        m_name = name;
    }

    template <typename otherVector>
    EvaluableVariable(const EvaluableVariable<otherVector>& other) = delete;

    template <typename otherVector>
    EvaluableVariable(EvaluableVariable<otherVector>&& other) = delete;


    Eigen::Index rows() const {
        return this->dimension();
    }

    Eigen::Index cols() const {
        return 1;
    }

    virtual const Scalar& evaluate() const {
        return m_values;
    }

    void operator=(const Scalar& rhs) {
        assert(rhs.size() == this->dimension());
        this->m_values = rhs;
    }

    void operator=(const EvaluableVariable<Scalar>& rhs) {
        assert(rhs.dimension() == this->dimension());
        this->m_values = rhs.evaluate();
    }

};


#endif // SDIFF_VARIABLE_H
