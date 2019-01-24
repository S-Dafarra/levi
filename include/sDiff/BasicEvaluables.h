/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_BASICEVALUABLES_H
#define SDIFF_BASICEVALUABLES_H

#include <sDiff/ForwardDeclarations.h>
#include <sDiff/Evaluable.h>
#include <sDiff/VariableBase.h>

template <typename Matrix>
class sDiff::ConstantEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> : public sDiff::Evaluable<Matrix>{
public:

    ConstantEvaluable(const Matrix& constant, std::string name)
        : Evaluable<Matrix>(constant, name)
    { }

    ConstantEvaluable(Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : Evaluable<Matrix>(rows, cols, name)
    { }

    virtual const Matrix& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual std::shared_ptr<typename sDiff::Evaluable<Matrix>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                  std::shared_ptr<sDiff::VariableBase> variable) final {
        return std::make_shared<NullEvaluable<typename sDiff::Evaluable<Matrix>::derivative_evaluable::matrix_type>>(this->rows(), variable->dimension());
    }


    void operator=(const Matrix& rhs) {
        this->m_evaluationBuffer = rhs;
    }
};

template <typename Scalar>
class sDiff::ConstantEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> : public sDiff::Evaluable<Scalar>{
public:

    ConstantEvaluable(const Scalar& constant)
        : Evaluable<Scalar>(constant)
    { }

    virtual const Scalar& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual std::shared_ptr<typename sDiff::Evaluable<Scalar>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                         std::shared_ptr<sDiff::VariableBase> variable) final {
        return std::make_shared<NullEvaluable<typename sDiff::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(1, variable->dimension());
    }

    void operator=(const Scalar& rhs) {
        this->m_evaluationBuffer = rhs;
    }
};

template <typename Matrix>
class sDiff::NullEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> : public sDiff::Evaluable<Matrix>{
public:


    NullEvaluable(Eigen::Index rows, Eigen::Index cols)
        : Evaluable<Matrix>(rows, cols, "0")
    {
        this->m_evaluationBuffer.setZero();
    }

    virtual const Matrix& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual std::shared_ptr<typename sDiff::Evaluable<Matrix>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                         std::shared_ptr<sDiff::VariableBase> variable) final {
        return std::make_shared<NullEvaluable<typename sDiff::Evaluable<Matrix>::derivative_evaluable::matrix_type>>(this->rows(), variable->dimension());
    }
};

template <typename Scalar>
class sDiff::NullEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> : public sDiff::Evaluable<Scalar>{
public:

    NullEvaluable(const Scalar& constant)
        : Evaluable<Scalar>(constant)
    {
        this->m_evaluationBuffer = 0;
    }

    virtual const Scalar& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual std::shared_ptr<typename sDiff::Evaluable<Scalar>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                         std::shared_ptr<sDiff::VariableBase> variable) final {
        return std::make_shared<NullEvaluable<typename sDiff::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(1, variable->dimension());
    }
};

template <typename Matrix>
class sDiff::IdentityEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> : public sDiff::Evaluable<Matrix>{
public:


    IdentityEvaluable(Eigen::Index rows, Eigen::Index cols)
        : Evaluable<Matrix>(rows, cols, "1")
    {
        this->m_evaluationBuffer.setIdentity();
    }

    virtual const Matrix& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual std::shared_ptr<typename sDiff::Evaluable<Matrix>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                         std::shared_ptr<sDiff::VariableBase> variable) final {
        return std::make_shared<NullEvaluable<typename sDiff::Evaluable<Matrix>::derivative_evaluable::matrix_type>>(this->rows(), variable->dimension());
    }
};

template <typename Scalar>
class sDiff::IdentityEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> : public sDiff::Evaluable<Scalar>{
public:

    IdentityEvaluable()
        : Evaluable<Scalar>(1)
    { }

    virtual const Scalar& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual std::shared_ptr<typename sDiff::Evaluable<Scalar>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                         std::shared_ptr<sDiff::VariableBase> variable) final {
        return std::make_shared<NullEvaluable<typename sDiff::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(1, variable->dimension());
    }
};

#endif // SDIFF_BASICEVALUABLES_H
