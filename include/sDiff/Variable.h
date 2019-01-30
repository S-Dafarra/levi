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

/**
 * @brief The EvaluableVariable class, specialized for vector values.
 *
 * This class inherits from both VariableBase and Evaluable. This allows to use the variables inside expressions as they were evaluables,
 * thus allowing for seamless application of arithmetic operators.
 */
template <typename Vector>
class sDiff::EvaluableVariable<Vector, typename std::enable_if<!std::is_arithmetic<Vector>::value>::type> : public sDiff::VariableBase, public sDiff::Evaluable<Vector> {

    template<typename OtherVector, bool isScalar>
    void copy_constant(bool_value<isScalar>, const OtherVector& rhs);

    template<typename OtherVector>
    void copy_constant(bool_value<true>, const OtherVector& rhs) {
        static_assert (this->rows_at_compile_time == Eigen::Dynamic || this->rows_at_compile_time == 1, "Cannot copy a scalar to this variable.");

        if (this->rows_at_compile_time == Eigen::Dynamic) {
            this->resize(1,1);
        }

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

    /**
     * @brief Assignement operator to set the values of the variable.
     *
     * The right hand side has to be a vector.
     */
    template<typename otherVector>
    void operator=(const otherVector& rhs) {
        copy_constant(bool_value<std::is_arithmetic<otherVector>::value>(), rhs);
    }

    /**
     * @brief Assignement operator to set the values of the variable equal to another variable.
     */
    template<typename OtherVector>
    void operator=(const EvaluableVariable<OtherVector>& rhs) {
        static_assert (OtherVector::ColsAtCompileTime == 1, "The chosen VectorType for the rhs should have exactly one column at compile time.");
        assert(rhs.dimension() == this->dimension());
        this->m_evaluationBuffer = rhs.evaluate();
    }

    virtual const Vector& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<Vector>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                    std::shared_ptr<sDiff::VariableBase> variable) final {
        assert(column == 0);
        if ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension())) {
            return sDiff::ExpressionComponent<sDiff::IdentityEvaluable<typename sDiff::Evaluable<Vector>::derivative_evaluable::matrix_type>>(this->dimension(), this->dimension(),
                                                                                                                                              "d " + variableName() + "/(d " + variable->variableName() + ")");
        } else {
            return sDiff::ExpressionComponent<sDiff::NullEvaluable<typename sDiff::Evaluable<Vector>::derivative_evaluable::matrix_type>>(this->dimension(), variable->dimension(),
                                                                                                                                          "d " + variableName() + "/(d " + variable->variableName() + ")");
        }
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
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

    /**
     * @brief Assignement operator to set the values of the variable.
     *
     * The right hand side has to be a scalar.
     */
    void operator=(const Scalar& rhs) {
        this->m_evaluationBuffer = rhs;
    }

    /**
     * @brief Assignement operator to set the values of the variable equal to another variable.
     */
    void operator=(const EvaluableVariable<Scalar>& rhs) {
        this->m_evaluationBuffer = rhs.evaluate();
    }

    virtual const Scalar& evaluate() final {
        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<Scalar>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                    std::shared_ptr<sDiff::VariableBase> variable) final {
        assert(column == 0);
        if ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension())) {
            return sDiff::ExpressionComponent<sDiff::IdentityEvaluable<typename sDiff::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(this->dimension(), this->dimension());
        } else {
            return sDiff::ExpressionComponent<sDiff::NullEvaluable<typename sDiff::Evaluable<Scalar>::derivative_evaluable::matrix_type>>(this->dimension(), variable->dimension());
        }
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension()));
    }

};


#endif // SDIFF_VARIABLE_H
