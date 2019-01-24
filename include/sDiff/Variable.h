/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_VARIABLE_H
#define SDIFF_VARIABLE_H

#include <sDiff/ForwardDeclarations.h>
#include <sDiff/VariableBase.h>
#include <sDiff/BasicEvaluables.h>

template <typename Vector>
class sDiff::EvaluableVariable<Vector, typename std::enable_if<!std::is_arithmetic<Vector>::value>::type> : public sDiff::VariableBase, public sDiff::Evaluable<Vector> {
public:

    EvaluableVariable(Eigen::Index dimension, const std::string& name)
        : sDiff::VariableBase(dimension, name)
        , sDiff::Evaluable<Vector>(dimension, 1, name)
    {
        static_assert (Vector::ColsAtCompileTime == 1, "The chosen VectorType for the Variable should have exactly one column at compile time.");

        this->m_evaluationBuffer.setZero();

    }

    template <typename otherVector>
    EvaluableVariable(const EvaluableVariable<otherVector>& other) = delete;

    template <typename otherVector>
    EvaluableVariable(EvaluableVariable<otherVector>&& other) = delete;


    template<typename otherVector>
    void operator=(const otherVector& rhs) {
        static_assert (otherVector::ColsAtCompileTime == 1, "The chosen VectorType for the rhs should have exactly one column at compile time.");
        assert(rhs.size() == this->dimension());
        this->m_evaluationBuffer = rhs;
    }

    template<typename OtherVector>
    void operator=(const EvaluableVariable<OtherVector>& rhs) {
        static_assert (OtherVector::ColsAtCompileTime == 1, "The chosen VectorType for the rhs should have exactly one column at compile time.");
        assert(rhs.dimension() == this->dimension());
        this->m_evaluationBuffer = rhs.evaluate();
    }

    virtual const Vector& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual std::shared_ptr<typename sDiff::Evaluable<Vector>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                  std::shared_ptr<sDiff::VariableBase> variable) final {
        assert(column == 0);
        if ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension())) {
            return std::make_shared<sDiff::IdentityEvaluable<typename sDiff::Evaluable<Vector>::derivative_evaluable::matrix_type>>(this->dimension(), this->dimension());
        } else {
            return std::make_shared<sDiff::NullEvaluable<typename sDiff::Evaluable<Vector>::derivative_evaluable::matrix_type>>(this->dimension(), variable->dimension());
        }
    }

};

template <typename Scalar>
class sDiff::EvaluableVariable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> : public sDiff::VariableBase, public sDiff::Evaluable<Scalar> {

public:

    EvaluableVariable(const std::string& name)
    : sDiff::VariableBase(1, name)
    , sDiff::Evaluable<Scalar>(0, name)
    { }

    template <typename otherVector>
    EvaluableVariable(const EvaluableVariable<otherVector>& other) = delete;

    template <typename otherVector>
    EvaluableVariable(EvaluableVariable<otherVector>&& other) = delete;

    void operator=(const Scalar& rhs) {
        assert(rhs.size() == this->dimension());
        this->m_evaluationBuffer = rhs;
    }

    void operator=(const EvaluableVariable<Scalar>& rhs) {
        assert(rhs.dimension() == this->dimension());
        this->m_evaluationBuffer = rhs.evaluate();
    }

    virtual const Scalar& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual std::shared_ptr<typename sDiff::Evaluable<Scalar>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                  std::shared_ptr<sDiff::VariableBase> variable) final {
        assert(column == 0);
        if ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension())) {
            return std::make_shared<sDiff::IdentityEvaluable<typename sDiff::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(this->dimension(), this->dimension());
        } else {
            return std::make_shared<sDiff::NullEvaluable<typename sDiff::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(this->dimension(), variable->dimension());
        }
    }

};


#endif // SDIFF_VARIABLE_H
