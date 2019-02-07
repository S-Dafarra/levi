/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_VARIABLE_H
#define LEVI_VARIABLE_H

#include <levi/HelpersForwardDeclarations.h>
#include <levi/ForwardDeclarations.h>
#include <levi/VariableBase.h>
#include <levi/BasicEvaluables.h>

/**
 * @brief The EvaluableVariable class, specialized for vector values.
 *
 * This class inherits from both VariableBase and Evaluable. This allows to use the variables inside expressions as they were evaluables,
 * thus allowing for seamless application of arithmetic operators.
 */
template <typename Vector>
class levi::EvaluableVariable<Vector, typename std::enable_if<!std::is_arithmetic<Vector>::value>::type> : public levi::VariableBase, public levi::Evaluable<Vector> {

    levi::ExpressionComponent<levi::IdentityEvaluable<typename levi::Evaluable<Vector>::derivative_evaluable::matrix_type>> m_identityDerivative;

    template<typename OtherVector, bool isScalar>
    void copy_constant(bool_value<isScalar>, const OtherVector& rhs);

    template<typename OtherVector>
    void copy_constant(bool_value<true>, const OtherVector& rhs) {
        static_assert (this->rows_at_compile_time == Eigen::Dynamic || this->rows_at_compile_time == 1, "Cannot copy a scalar to this variable.");

        this->resize(1,1);

        this->m_evaluationBuffer(0,0) = rhs;
    }

    template<typename OtherVector>
    void copy_constant(bool_value<false>, const OtherVector& rhs) {
        static_assert (OtherVector::ColsAtCompileTime == 1, "The chosen VectorType for the rhs should have exactly one column at compile time.");
        assert(rhs.size() == this->dimension());
        this->m_evaluationBuffer = rhs;
    }

public:

    EvaluableVariable(Eigen::Index dimension, const std::string& name)
        : levi::VariableBase(dimension, name)
        , levi::Evaluable<Vector>(dimension, 1, name)
        , m_identityDerivative(dimension, dimension, "d " + name + "/(d " + name + ")")
    {
        static_assert (Vector::ColsAtCompileTime == 1, "The chosen VectorType for the Variable should have exactly one column at compile time.");

        this->m_evaluationBuffer.setZero();

    }

    template <typename otherVector>
    EvaluableVariable(const EvaluableVariable<otherVector>& other) = delete;

    template <typename otherVector>
    EvaluableVariable(EvaluableVariable<otherVector>&& other) = delete;

    /**
     * @brief Assignement operator to set the values of the variable.
     *
     * The right hand side has to be a vector.
     */
    template<typename otherVector>
    void operator=(const otherVector& rhs) {
        copy_constant(bool_value<std::is_arithmetic<otherVector>::value>(), rhs);
        this->resetEvaluationRegister();
    }

    /**
     * @brief Assignement operator to set the values of the variable equal to another variable.
     */
    template<typename OtherVector>
    void operator=(const EvaluableVariable<OtherVector>& rhs) {
        static_assert (OtherVector::ColsAtCompileTime == 1, "The chosen VectorType for the rhs should have exactly one column at compile time.");
        assert(rhs.dimension() == this->dimension());
        this->m_evaluationBuffer = rhs.evaluate();
        this->resetEvaluationRegister();
    }

    virtual const Vector& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<Vector>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                  std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);
        if ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension())) {
            return m_identityDerivative;
        } else {
            return levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<Vector>::derivative_evaluable::matrix_type>>(this->dimension(), variable->dimension(),
                                                                                                                                       "d " + variableName() + "/(d " + variable->variableName() + ")");
        }
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension()));
    }

};

/**
 * @brief The EvaluableVariable class, specialized for scalar values.
 *
 * This class inherits from both VariableBase and Evaluable. This allows to use the variables inside expressions as they were evaluables,
 * thus allowing for seamless application of arithmetic operators.
 */
template <typename Scalar>
class levi::EvaluableVariable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> : public levi::VariableBase, public levi::Evaluable<Scalar> {

    levi::ExpressionComponent<levi::IdentityEvaluable<typename levi::Evaluable<Scalar>::derivative_evaluable::matrix_type>> m_identityDerivative;

public:

    EvaluableVariable(const std::string& name)
        : levi::VariableBase(1, name)
        , levi::Evaluable<Scalar>(0, name)
        , m_identityDerivative(1,1)
    { }

    template <typename otherVector>
    EvaluableVariable(const EvaluableVariable<otherVector>& other) = delete;

    template <typename otherVector>
    EvaluableVariable(EvaluableVariable<otherVector>&& other) = delete;

    /**
     * @brief Assignement operator to set the values of the variable.
     *
     * The right hand side has to be a scalar.
     */
    void operator=(const Scalar& rhs) {
        this->m_evaluationBuffer = rhs;
        this->resetEvaluationRegister();
    }

    /**
     * @brief Assignement operator to set the values of the variable equal to another variable.
     */
    void operator=(const EvaluableVariable<Scalar>& rhs) {
        this->m_evaluationBuffer = rhs.evaluate();
        this->resetEvaluationRegister();
    }

    virtual const Scalar& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<Scalar>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                  std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);
        if ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension())) {
            return m_identityDerivative;
        } else {
            return levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(this->dimension(), variable->dimension());
        }
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension()));
    }

};


#endif // LEVI_VARIABLE_H
