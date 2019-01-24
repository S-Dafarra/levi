/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_EVALUABLE_H
#define SDIFF_EVALUABLE_H

#include <sDiff/ForwardDeclarations.h>

template<typename Matrix>
class sDiff::Evaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> {

    std::string m_name;

public:

    typedef Matrix matrix_type;

    typedef typename Matrix::value_type value_type;

    static const Eigen::Index rows_at_compile_time = Matrix::RowsAtCompileTime;

    static const Eigen::Index cols_at_compile_time = Matrix::ColsAtCompileTime;

    typedef Eigen::Matrix<value_type, 1, cols_at_compile_time> row_type;

    typedef Evaluable<Eigen::Matrix<value_type, rows_at_compile_time, Eigen::Dynamic>> derivative_evaluable;

protected:

    Matrix m_evaluationBuffer;
    std::shared_ptr<derivative_evaluable> m_derivative;

public:

    Evaluable() = delete;

    Evaluable(const std::string& name)
        : m_name(name)
        , m_derivative(nullptr)
    { }

    Evaluable(Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : m_name(name)
        , m_evaluationBuffer(rows, cols)
        , m_derivative(nullptr)
    {
        m_evaluationBuffer.setZero();
    }

    Evaluable(const Matrix& initialValue, const std::string& name)
        : m_name(name)
        , m_evaluationBuffer(initialValue)
        , m_derivative(nullptr)
    { }

    template <typename OtherMatrix>
    Evaluable(const Evaluable<OtherMatrix>& other) = delete;

    template <typename OtherMatrix>
    Evaluable(Evaluable<OtherMatrix>&& other) = delete;

    virtual ~Evaluable() { }

    Eigen::Index rows() const {
        return m_evaluationBuffer.rows();
    }

    Eigen::Index cols() const {
        return m_evaluationBuffer.cols();
    }

    void resize(Eigen::Index newRows, Eigen::Index newCols) {
        m_evaluationBuffer.resize(newRows, newCols);
    }

    std::string name() const {
        return m_name;
    }

    virtual const Matrix& evaluate() = 0;

    virtual std::shared_ptr<derivative_evaluable> getColumnDerivative(Eigen::Index column, std::shared_ptr<sDiff::VariableBase> variable) {
        return nullptr;
    }

    Evaluable<Matrix>& operator=(const Evaluable& other) = delete;

    void operator=(Evaluable&& other) = delete;

    Evaluable<Matrix>& operator+(const Evaluable& other) const  = delete;

    Evaluable<Matrix>& operator-(const Evaluable& other) const = delete;

    Evaluable<Matrix>& operator*(const Evaluable& other) const = delete;

};

template <typename Scalar>
class sDiff::Evaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {

    std::string m_name;

public:

    typedef Scalar matrix_type;

    typedef Scalar value_type;

    typedef Scalar row_type;

    typedef Evaluable<Eigen::Matrix<value_type, 1, Eigen::Dynamic>> derivative_evaluable;


protected:

    Scalar m_evaluationBuffer;

    std::shared_ptr<derivative_evaluable> m_derivative;

public:

    Evaluable() = delete;

    Evaluable(const std::string& name)
        : m_name(name)
        , m_derivative(nullptr)
    { }

    Evaluable(const Scalar& initialValue, const std::string& name)
        : m_name(name)
        , m_evaluationBuffer(initialValue)
        , m_derivative(nullptr)
    { }

    Evaluable(const Scalar& initialValue)
        : m_name(std::to_string(initialValue))
        , m_evaluationBuffer(initialValue)
        , m_derivative(nullptr)
    { }

    template <typename OtherMatrix, typename OtherDerivativeEvaluable>
    Evaluable(const Evaluable<OtherMatrix, OtherDerivativeEvaluable>& other) = delete;

    template <typename OtherMatrix, typename OtherDerivativeEvaluable>
    Evaluable(Evaluable<OtherMatrix, OtherDerivativeEvaluable>&& other) = delete;

    virtual ~Evaluable() { }

    Eigen::Index rows() const {
        return 1;
    }

    Eigen::Index cols() const {
        return 1;
    }

    std::string name() const {
        return m_name;
    }

    virtual const Scalar& evaluate() = 0;

    virtual std::shared_ptr<derivative_evaluable> getColumnDerivative(Eigen::Index column, std::shared_ptr<sDiff::VariableBase> variable) {
        return nullptr;
    }

    Evaluable<Scalar>& operator=(const Evaluable& other) = delete;

    void operator=(Evaluable&& other) = delete;

    Evaluable<Scalar>& operator+(const Evaluable& other) const  = delete;

    Evaluable<Scalar>& operator-(const Evaluable& other) const = delete;

    Evaluable<Scalar>& operator*(const Evaluable& other) const = delete;

};

#endif // SDIFF_EVALUABLE_H
