/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_BASICEVALUABLES_H
#define LEVI_BASICEVALUABLES_H

#include <levi/ForwardDeclarations.h>
#include <levi/Evaluable.h>
#include <levi/VariableBase.h>

/**
 * @brief The ConstantEvaluable
 *
 * Evaluable containing a simple matrix, which can be assigned through the operator =
 *
 */
template <typename Matrix>
class levi::ConstantEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> : public levi::Evaluable<Matrix>{
public:

    ConstantEvaluable(const Matrix& constant, std::string name)
        : levi::Evaluable<Matrix>(constant, name)
    { }

    ConstantEvaluable(Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : levi::Evaluable<Matrix>(rows, cols, name)
    { }

    virtual const Matrix& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<Matrix>::derivative_evaluable> getColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) final{
        return levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<Matrix>::derivative_evaluable::matrix_type>>(this->rows(), variable->dimension());
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return false;
    }

    void operator=(const Matrix& rhs) {
        this->m_evaluationBuffer = rhs;
    }
};

/**
 * @brief The ConstantEvaluable
 *
 * Evaluable containing a simple scalar, which can be assigned through the operator =
 *
 */
template <typename Scalar>
class levi::ConstantEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> : public levi::Evaluable<Scalar>{
public:

    ConstantEvaluable(const Scalar& constant)
        : levi::Evaluable<Scalar>(constant)
    { }

    virtual const Scalar& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<Scalar>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                  std::shared_ptr<levi::VariableBase> variable) final {
        return levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(1, variable->dimension());
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return false;
    }

    void operator=(const Scalar& rhs) {
        this->m_evaluationBuffer = rhs;
    }
};

/**
 * @brief The NullEvaluable
 *
 * Evaluable containing a simple matrix made of zeros.
 *
 */
template <typename Matrix>
class levi::NullEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> : public levi::Evaluable<Matrix>{
public:

    NullEvaluable(Eigen::Index rows, Eigen::Index cols)
        : levi::Evaluable<Matrix>(rows, cols, "0")
    {
        this->m_evaluationBuffer.setZero();
    }

    NullEvaluable(Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : levi::Evaluable<Matrix>(rows, cols, name)
    {
        this->m_evaluationBuffer.setZero();
    }

    virtual const Matrix& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<Matrix>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                  std::shared_ptr<levi::VariableBase> variable) final {
        return levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<Matrix>::derivative_evaluable::matrix_type>>(this->rows(), variable->dimension());
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return false;
    }
};

/**
 * @brief The NullEvaluable
 *
 * Evaluable containing a zero.
 *
 */
template <typename Scalar>
class levi::NullEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> : public levi::Evaluable<Scalar>{
public:

    NullEvaluable(const Scalar& constant)
        : levi::Evaluable<Scalar>(constant)
    {
        this->m_evaluationBuffer = 0;
    }

    virtual const Scalar& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<Scalar>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                  std::shared_ptr<levi::VariableBase> variable) final {
        return levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(1, variable->dimension());
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return false;
    }
};

/**
 * @brief The IdentityEvaluable
 *
 * Evaluable containing a simple identity matrix.
 *
 */
template <typename Matrix>
class levi::IdentityEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> : public levi::Evaluable<Matrix>{
public:


    IdentityEvaluable(Eigen::Index rows, Eigen::Index cols)
        : levi::Evaluable<Matrix>(rows, cols, "I")
    {
        this->m_evaluationBuffer.setIdentity();
    }

    IdentityEvaluable(Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : levi::Evaluable<Matrix>(rows, cols, name)
    {
        this->m_evaluationBuffer.setIdentity();
    }

    virtual const Matrix& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<Matrix>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                  std::shared_ptr<levi::VariableBase> variable) final {
        return levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<Matrix>::derivative_evaluable::matrix_type>>(this->rows(), variable->dimension());
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return false;
    }
};

/**
 * @brief The IdentityEvaluable
 *
 * Evaluable containing a 1 of a specified type.
 *
 */
template <typename Scalar>
class levi::IdentityEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> : public levi::Evaluable<Scalar>{
public:

    IdentityEvaluable()
        : levi::Evaluable<Scalar>(1)
    { }

    virtual const Scalar& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<Scalar>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                  std::shared_ptr<levi::VariableBase> variable) final {
        return levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(1, variable->dimension());
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return false;
    }
};

#endif // LEVI_BASICEVALUABLES_H
