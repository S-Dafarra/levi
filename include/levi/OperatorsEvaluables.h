/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_OPERATORS_EVALUABLES_H
#define LEVI_OPERATORS_EVALUABLES_H

#include <levi/HelpersForwardDeclarations.h>
#include <levi/ForwardDeclarations.h>
#include <levi/BasicEvaluables.h>
#include <levi/Expression.h>
#include <levi/OperatorsBase.h>

/**
 * Helper struct for determining the type resulting from an addition
 */
template <typename Scalar_lhs, typename Scalar_rhs>
struct levi::scalar_sum_return {
    //decltype allow to get the return type of the addition of a variable of type Scalar_lhs to a variable of type Scalar_rhs.
    typedef decltype (std::declval<Scalar_lhs>() + std::declval<Scalar_rhs>()) type;
};

/**
 * Helper struct for determining the type resulting from an addition of two matrices. Specialization for two matrices
 *
 */
template<typename Scalar_lhs, int lhsRows, int lhsCols, int lhsOptions, int lhsMaxRows, int lhsMaxCols, typename Scalar_rhs, int rhsRows, int rhsCols, int rhsOptions, int rhsMaxRows, int rhsMaxCols>
struct levi::matrix_sum_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols, lhsOptions, lhsMaxRows, lhsMaxCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols, rhsOptions, rhsMaxRows, rhsMaxCols>,
        typename std::enable_if<levi::is_valid_sum<lhsRows, lhsCols, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename levi::scalar_sum_return<Scalar_lhs, Scalar_rhs>::type, (lhsRows < rhsRows) ? lhsRows : rhsRows, (lhsCols < rhsCols) ? lhsCols : rhsCols> type; //here we assume that Eigen::Dynamic == -1, thus, given that it is a valid sum, the minimum will be -1 if present, thus using Eigen::Dynamic as output
};

/**
 * Helper struct for determining the type resulting from an addition of two matrices. Specialization for two scalars.
 *
 */
template<typename Scalar_lhs, typename Scalar_rhs>
struct levi::matrix_sum_return<Scalar_lhs, Scalar_rhs,
        typename std::enable_if<std::is_arithmetic<Scalar_lhs>::value && std::is_arithmetic<Scalar_rhs>::value>::type> {
    typedef typename levi::scalar_sum_return<Scalar_lhs, Scalar_rhs>::type type;
};

/**
 * Helper struct for determining the type resulting from an addition of two matrices. Specialization for a scalar and a matrix.
 *
 */
template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols, int rhsOptions, int rhsMaxRows, int rhsMaxCols>
struct levi::matrix_sum_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols, rhsOptions, rhsMaxRows, rhsMaxCols>,
        typename std::enable_if<std::is_arithmetic<Scalar>::value && levi::is_valid_sum<1,1, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename levi::scalar_sum_return<Scalar, Scalar_rhs>::type, rhsRows, rhsCols> type;
};

/**
 * Helper struct for determining the type resulting from an addition of two matrices. Specialization for a matrix and a scalar
 *
 */
template<typename Scalar_lhs, int lhsRows, int lhsCols, int lhsOptions, int lhsMaxRows, int lhsMaxCols, typename Scalar>
struct levi::matrix_sum_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols, lhsOptions, lhsMaxRows, lhsMaxCols>, Scalar,
        typename std::enable_if<std::is_arithmetic<Scalar>::value && levi::is_valid_sum<1,1, lhsRows, lhsCols>::value>::type> {
    typedef Eigen::Matrix<typename levi::scalar_sum_return<Scalar, Scalar_lhs>::type, lhsRows, lhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication
 */
template <typename Scalar_lhs, typename Scalar_rhs>
struct levi::scalar_product_return {
    //decltype allow to get the return type of the multiplication of a variable of type Scalar_lhs to a variable of type Scalar_rhs.
    typedef decltype (std::declval<Scalar_lhs>() * std::declval<Scalar_rhs>()) type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization for two matrices
 *
 */
template<typename Scalar_lhs, int lhsRows, int lhsCols, int lhsOptions, int lhsMaxRows, int lhsMaxCols, typename Scalar_rhs, int rhsRows, int rhsCols, int rhsOptions, int rhsMaxRows, int rhsMaxCols>
struct levi::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols, lhsOptions, lhsMaxRows, lhsMaxCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols, rhsOptions, rhsMaxRows, rhsMaxCols>,
        typename std::enable_if<levi::is_valid_product<lhsRows, lhsCols, rhsRows, rhsCols>::value && !(lhsRows == 1 && lhsCols == 1) && !(rhsRows == 1 && rhsCols == 1)>::type> {
    typedef Eigen::Matrix<typename levi::scalar_product_return<Scalar_lhs, Scalar_rhs>::type, lhsRows, rhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization in the case the lhs has exactly one row and one column.
 *
 */
template<typename Scalar_lhs, int lhsRows, int lhsCols, int lhsOptions, int lhsMaxRows, int lhsMaxCols, typename Scalar_rhs, int rhsRows, int rhsCols, int rhsOptions, int rhsMaxRows, int rhsMaxCols>
struct levi::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols, lhsOptions, lhsMaxRows, lhsMaxCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols, rhsOptions, rhsMaxRows, rhsMaxCols>,
        typename std::enable_if<lhsRows == 1 && lhsCols == 1>::type> {
    typedef Eigen::Matrix<typename levi::scalar_product_return<Scalar_lhs, Scalar_rhs>::type, rhsRows, rhsCols> type;
};


/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization in the case the rhs has exactly one row and one column.
 *
 */
template<typename Scalar_lhs, int lhsRows, int lhsCols, int lhsOptions, int lhsMaxRows, int lhsMaxCols, typename Scalar_rhs, int rhsRows, int rhsCols, int rhsOptions, int rhsMaxRows, int rhsMaxCols>
struct levi::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols, lhsOptions, lhsMaxRows, lhsMaxCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols, rhsOptions, rhsMaxRows, rhsMaxCols>,
        typename std::enable_if<rhsRows == 1 && rhsCols == 1>::type> {
    typedef Eigen::Matrix<typename levi::scalar_product_return<Scalar_lhs, Scalar_rhs>::type, lhsRows, lhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization for a scalar and a matrix
 */
template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols, int rhsOptions, int rhsMaxRows, int rhsMaxCols>
struct levi::matrix_product_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols, rhsOptions, rhsMaxRows, rhsMaxCols>,
        typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Eigen::Matrix<typename levi::scalar_product_return<Scalar, Scalar_rhs>::type, rhsRows, rhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization for a matrix and a scalar.
 */
template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols, int lhsOptions, int lhsMaxRows, int lhsMaxCols>
struct levi::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols, lhsOptions, lhsMaxRows, lhsMaxCols>, Scalar,
        typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Eigen::Matrix<typename levi::scalar_product_return<Scalar, Scalar_lhs>::type, lhsRows, lhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization for two scalars.
 */
template<typename Scalar_lhs, typename Scalar_rhs>
struct levi::matrix_product_return<Scalar_lhs, Scalar_rhs,
        typename std::enable_if<std::is_arithmetic<Scalar_lhs>::value && std::is_arithmetic<Scalar_rhs>::value>::type> {
    typedef typename levi::scalar_product_return<Scalar_lhs, Scalar_rhs>::type type;
};

/**
 * @brief The SumEvaluable. Implements the sum of two evaluables.
 */
template <class LeftEvaluable, class RightEvaluable>
class levi::SumEvaluable : public levi::BinaryOperator<typename levi::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type, LeftEvaluable, RightEvaluable>{

public:

    typedef typename levi::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type sum_type;

    SumEvaluable(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs)
        : levi::BinaryOperator<sum_type, LeftEvaluable, RightEvaluable>(lhs, rhs, lhs.rows(), lhs.cols(), "(" + lhs.name() + " + " + rhs.name() + ")")
    { }

    virtual typename levi::eval_return_type<sum_type>::type evaluateID(size_t callerID) final {
        if (callerID < this->m_evaluationRegister.size()) {
            this->m_evaluationRegister[callerID] = true;
            this->m_alreadyComputed = true;
        }
        return evaluate();
    }

    virtual typename levi::eval_return_type<sum_type>::type evaluate() final {
        return this->m_lhs.evaluate() + this->m_rhs.evaluate();

        this->m_evaluationBuffer = this->m_lhs.evaluate() + this->m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<sum_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                       std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename levi::Evaluable<sum_type>::derivative_evaluable> sumDerivative;

        bool isLeftDependent = this->m_lhs.isDependentFrom(variable);
        bool isRightDependent = this->m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            sumDerivative = this->m_lhs.getColumnDerivative(column, variable) + this->m_rhs.getColumnDerivative(column, variable);

            return sumDerivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<sum_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d" + this->name() + "/d" + variable->variableName());
            sumDerivative = nullDerivative;
            return sumDerivative;
        }

        if (isLeftDependent) {
            sumDerivative = this->m_lhs.getColumnDerivative(column, variable);
            return sumDerivative;
        } else {
            sumDerivative = this->m_rhs.getColumnDerivative(column, variable);
            return sumDerivative;
        }
    }

};

/**
 * @brief The SubtractionEvaluable. Implements the subtraction of two evaluables.
 */
template <class LeftEvaluable, class RightEvaluable>
class levi::SubtractionEvaluable : public levi::BinaryOperator<typename levi::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type, LeftEvaluable, RightEvaluable>{

public:

    typedef typename levi::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type sum_type;

    SubtractionEvaluable(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs)
        : levi::BinaryOperator<sum_type, LeftEvaluable, RightEvaluable>(lhs, rhs, lhs.rows(), lhs.cols(), "(" + lhs.name() + " - " + rhs.name() + ")")
    { }

    virtual typename levi::eval_return_type<sum_type>::type evaluateID(size_t callerID) {
        if (callerID < this->m_evaluationRegister.size()) {
            this->m_evaluationRegister[callerID] = true;
            this->m_alreadyComputed = true;
        }
        return evaluate();
    }

    virtual typename levi::eval_return_type<sum_type>::type evaluate() final {
        return this->m_lhs.evaluate() - this->m_rhs.evaluate();
        this->m_evaluationBuffer = this->m_lhs.evaluate() - this->m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<sum_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                       std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename levi::Evaluable<sum_type>::derivative_evaluable> sumDerivative;

        bool isLeftDependent = this->m_lhs.isDependentFrom(variable);
        bool isRightDependent = this->m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            sumDerivative = this->m_lhs.getColumnDerivative(column, variable) - this->m_rhs.getColumnDerivative(column, variable);

            return sumDerivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<sum_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d" + this->name() + "/d" + variable->variableName());
            sumDerivative = nullDerivative;
            return sumDerivative;
        }

        if (isLeftDependent) {
            sumDerivative = this->m_lhs.getColumnDerivative(column, variable);
            return sumDerivative;
        } else {
            sumDerivative = - this->m_rhs.getColumnDerivative(column, variable);
            return sumDerivative;
        }
    }

};

template <class EvaluableT>
class levi::SignInvertedEvaluable : public levi::UnaryOperator<typename EvaluableT::matrix_type, EvaluableT>{

public:

    SignInvertedEvaluable(const levi::ExpressionComponent<EvaluableT>& expression)
        : levi::UnaryOperator<typename EvaluableT::matrix_type, EvaluableT>(expression, expression.rows(), expression.cols(), "-" + expression.name())
    { }

    virtual typename levi::eval_return_type<typename EvaluableT::matrix_type>::type evaluateID(size_t callerID) {
        if (callerID < this->m_evaluationRegister.size()) {
            this->m_evaluationRegister[callerID] = true;
            this->m_alreadyComputed = true;
        }
        return evaluate();
    }

    virtual typename levi::eval_return_type<typename EvaluableT::matrix_type>::type evaluate() final {
        return -(this->m_expression.evaluate());

        this->m_evaluationBuffer = -(this->m_expression.evaluate());

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::matrix_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                                               std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::matrix_type>::derivative_evaluable> derivative;

        derivative = -(this->m_expression.getColumnDerivative(column, variable));

        return derivative;
    }

};

/**
 * @brief The ProductEvaluable. Implements the product of two evaluables.
 */
template <class LeftEvaluable, class RightEvaluable>
class levi::ProductEvaluable : public levi::BinaryOperator<typename levi::matrix_product_return<
        typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type, LeftEvaluable, RightEvaluable>{

public:

    typedef typename levi::matrix_product_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type product_type;

private:

    typedef levi::ExpressionComponent<typename levi::Evaluable<product_type>::derivative_evaluable> derivative_expression;

    template<bool lhsIsScalar, bool rhsIsScalar>
    derivative_expression get_derivative(levi::bool_value<lhsIsScalar>, levi::bool_value<rhsIsScalar>, Eigen::Index column, std::shared_ptr<levi::VariableBase> variable);

    /**
     * @brief Helper function for the derivative of the multiplication between two scalars
     */
    derivative_expression get_derivative(levi::bool_value<true>, levi::bool_value<true>, Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {
        derivative_expression derivative;

        bool isLeftDependent = this->m_lhs.isDependentFrom(variable);
        bool isRightDependent = this->m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            derivative = this->m_rhs * this->m_lhs.getColumnDerivative(column, variable) + this->m_lhs * this->m_rhs.getColumnDerivative(column, variable);
            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d(" + this->name() + ")/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (isLeftDependent) {
            derivative = this->m_rhs * this->m_lhs.getColumnDerivative(column, variable);
            return derivative;
        } else {
            derivative = this->m_lhs * this->m_rhs.getColumnDerivative(column, variable);
            return derivative;
        }
    }

    /**
     * @brief Helper function for the derivative of the multiplication between a scalar and a matrix.
     */
    derivative_expression get_derivative(levi::bool_value<true>, levi::bool_value<false>, Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {

        //k*A
        derivative_expression derivative;

        bool isLeftDependent = this->m_lhs.isDependentFrom(variable);
        bool isRightDependent = this->m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            derivative = this->m_lhs * this->m_rhs.getColumnDerivative(column, variable) + this->m_rhs.col(column) * this->m_lhs.getColumnDerivative(0, variable);
            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d(" + this->name() + ")/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (isLeftDependent) {
            derivative = this->m_rhs.col(column) * this->m_lhs.getColumnDerivative(0, variable);
            return derivative;
        } else {
            derivative = this->m_lhs * this->m_rhs.getColumnDerivative(column, variable);
            return derivative;
        }
    }

    /**
     * @brief Helper function for the derivative of the multiplication between a matrix and a scalar.
     */
    derivative_expression get_derivative(levi::bool_value<false>, levi::bool_value<true>, Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {

        //A*k
        derivative_expression derivative;

        bool isLeftDependent = this->m_lhs.isDependentFrom(variable);
        bool isRightDependent = this->m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            derivative = this->m_rhs * this->m_lhs.getColumnDerivative(column, variable) + this->m_lhs.col(column) * this->m_rhs.getColumnDerivative(0, variable);
            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d(" + this->name() + ")/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (isLeftDependent) {
            derivative = this->m_rhs * this->m_lhs.getColumnDerivative(column, variable);
            return derivative;
        } else {
            derivative = this->m_lhs.col(column) * this->m_rhs.getColumnDerivative(0, variable);
            return derivative;
        }
    }

    /**
     * @brief Helper function for the derivative of the multiplication between two matrices.
     */
    derivative_expression get_derivative(levi::bool_value<false>, levi::bool_value<false>, Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {
        derivative_expression derivative;

        bool isLeftDependent = this->m_lhs.isDependentFrom(variable);
        bool isRightDependent = this->m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {

            derivative = this->m_lhs * this->m_rhs.getColumnDerivative(column, variable);

            for (Eigen::Index i = 0; i < this->m_rhs.rows(); ++i) {
                derivative = derivative + this->m_lhs.getColumnDerivative(i, variable) * this->m_rhs(i, column);
            }

            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d(" + this->name() + ")/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (isLeftDependent) {

            derivative = this->m_lhs.getColumnDerivative(0, variable) * this->m_rhs(0, column);

            for (Eigen::Index i = 1; i < this->m_rhs.rows(); ++i) {
                derivative = derivative + this->m_lhs.getColumnDerivative(i, variable) * this->m_rhs(i, column);
            }

            return derivative;

        } else {

            derivative = this->m_lhs * this->m_rhs.getColumnDerivative(column, variable);

            return derivative;
        }

    }

    template<bool value>
    void eval(levi::bool_value<value>);

    void eval(levi::bool_value<true>) {
        this->m_evaluationBuffer = this->m_lhs.evaluate() * this->m_rhs.evaluate();
    }

    void eval(levi::bool_value<false>) {
        this->m_evaluationBuffer.noalias() = this->m_lhs.evaluate() * this->m_rhs.evaluate();
    }

public:


    ProductEvaluable(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs)
        : levi::BinaryOperator<product_type, LeftEvaluable, RightEvaluable>(lhs, rhs, (lhs.rows() == 1 && lhs.cols() == 1 && rhs.rows() != 1)? rhs.rows() : lhs.rows(),
                                        (rhs.rows() == 1 && rhs.cols() == 1 && lhs.cols() != 1)? lhs.cols() : rhs.cols(),
                                        lhs.name() + " * " + rhs.name())
    { }

    virtual typename levi::eval_return_type<product_type>::type evaluate() final {

        eval(levi::bool_value<std::is_arithmetic<product_type>::value>());

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<product_type>::derivative_evaluable>
    getNewColumnDerivative(Eigen::Index column,
                           std::shared_ptr<levi::VariableBase> variable) final {

        return get_derivative(levi::bool_value<std::is_arithmetic<typename LeftEvaluable::matrix_type>::value>(),
                              levi::bool_value<std::is_arithmetic<typename RightEvaluable::matrix_type>::value>(),
                              column, variable);
    }

};

template <class EvaluableT>
class levi::PowEvaluable : public levi::UnaryOperator<typename EvaluableT::value_type, EvaluableT> {

    typename EvaluableT::value_type m_exponent;

    template<bool rhsIsScalar>
    double get_value(levi::bool_value<rhsIsScalar>);

    double get_value(levi::bool_value<true>) {
        return this->m_expression.evaluate();
    }

    double get_value(levi::bool_value<false>) {
        return this->m_expression.evaluate()(0,0);
    }

public:

    PowEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, typename EvaluableT::value_type exponent)
        : levi::UnaryOperator<typename EvaluableT::value_type, EvaluableT>(expression, "(" +expression.name() + ")^(" + std::to_string(exponent) + ")")
        , m_exponent(exponent)
    {
        assert(this->m_expression.rows() == 1 && this->m_expression.cols() == 1);
    }

    virtual typename levi::eval_return_type<typename EvaluableT::value_type>::type evaluate() final {

        this->m_evaluationBuffer = std::pow(get_value(levi::bool_value<std::is_arithmetic<typename EvaluableT::matrix_type>::value>()), m_exponent);

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                                              std::shared_ptr<levi::VariableBase> variable) final {
        assert(column == 0);

        levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> derivative;

        derivative = this->m_expression.pow(m_exponent - 1) * m_exponent * this->m_expression.getColumnDerivative(column, variable);

        return derivative;
    }

};

template <class LeftEvaluable, class RightEvaluable>
class levi::DivisionEvaluable : public levi::BinaryOperator<typename levi::matrix_product_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::value_type>::type, LeftEvaluable, RightEvaluable>
{
public:

    typedef typename levi::matrix_product_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::value_type>::type product_type;

private:

    template<bool rhsIsScalar>
    double get_rhs_value(levi::bool_value<rhsIsScalar>);

    double get_rhs_value(levi::bool_value<true>) {
        return this->m_rhs.evaluate();
    }

    double get_rhs_value(levi::bool_value<false>) {
        return this->m_rhs.evaluate()(0,0);
    }

public:

    DivisionEvaluable(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs)
        : levi::BinaryOperator<product_type, LeftEvaluable, RightEvaluable>(lhs, rhs, lhs.rows(), rhs.cols(), lhs.name() + "/(" + rhs.name() + ")")
    {
        assert(rhs.rows() == 1 && rhs.cols() == 1);
    }

    virtual typename levi::eval_return_type<product_type>::type evaluate() final {

        this->m_evaluationBuffer = this->m_lhs.evaluate() / get_rhs_value(levi::bool_value<std::is_arithmetic<typename RightEvaluable::matrix_type>::value>());

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<product_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) final {

        levi::ExpressionComponent<typename levi::Evaluable<product_type>::derivative_evaluable> derivative;

        derivative = (this->m_lhs * this->m_rhs.pow(-1)).getColumnDerivative(column, variable);

        return derivative;
    }

};

/**
 * @brief The SkewEvaluable.
 *
 * It allows to compute the skew symmetric matrix out of a three dimensional vector.
 */
template <typename EvaluableT>
class levi::SkewEvaluable : public levi::UnaryOperator<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>, EvaluableT> {

    Eigen::Matrix<typename EvaluableT::value_type, 3, 1> m_vector;
    Eigen::Matrix<typename EvaluableT::value_type, 3, 3> m_derivativeBuffer;

public:

    SkewEvaluable(const levi::ExpressionComponent<EvaluableT>& expression)
        : levi::UnaryOperator<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>, EvaluableT>(expression, "skew(" + expression.name() + ")")
    {
        assert((expression.rows() == 3) && (expression.cols() == 1));
    }

    virtual typename levi::eval_return_type<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>::type evaluate() final {
        m_vector = this->m_expression.evaluate();

        this->m_evaluationBuffer(0,0) = 0.0;
        this->m_evaluationBuffer(0,1) = -m_vector[2];
        this->m_evaluationBuffer(0,2) = m_vector[1];
        this->m_evaluationBuffer(1,0) = m_vector[2];
        this->m_evaluationBuffer(1,1) = 0.0;
        this->m_evaluationBuffer(1,2) = -m_vector[0];
        this->m_evaluationBuffer(2,0) = -m_vector[1];
        this->m_evaluationBuffer(2,1) = m_vector[0];
        this->m_evaluationBuffer(2,2) = 0.0;

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                                                                   std::shared_ptr<levi::VariableBase> variable) final {

        assert( column < 3);
        levi::ExpressionComponent<typename levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>::derivative_evaluable> derivative;

        if (!this->m_expression.isDependentFrom(variable)) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), this->cols(), "d" + this->name() + "/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (column == 0) {

            m_derivativeBuffer.setZero();
            m_derivativeBuffer(1,2) = 1;
            m_derivativeBuffer(2,1) = -1;

            levi::ExpressionComponent<levi::ConstantEvaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> col1(m_derivativeBuffer, "LeviCivita_ij0");

            derivative = col1 * this->m_expression.getColumnDerivative(0, variable);
            return derivative;

        }

        if (column == 1) {

            m_derivativeBuffer.setZero();
            m_derivativeBuffer(0,2) = -1;
            m_derivativeBuffer(2,0) = 1;

            levi::ExpressionComponent<levi::ConstantEvaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> col2(m_derivativeBuffer, "LeviCivita_ij1");

            derivative = col2 * this->m_expression.getColumnDerivative(0, variable);
            return derivative;

        }

        if (column == 2) {

            m_derivativeBuffer.setZero();
            m_derivativeBuffer(0,1) = 1;
            m_derivativeBuffer(1,0) = -1;

            levi::ExpressionComponent<levi::ConstantEvaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> col3(m_derivativeBuffer, "LeviCivita_ij2");

            derivative = col3 * this->m_expression.getColumnDerivative(0, variable);
            return derivative;

        }

        return derivative;
    }

};

template <typename EvaluableT>
class levi::TransposeEvaluable : public levi::UnaryOperator<typename levi::transpose_type<EvaluableT>::type, EvaluableT> {

    std::vector<levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::value_type>::derivative_evaluable>> m_derivatives;

public:

    TransposeEvaluable(const levi::ExpressionComponent<EvaluableT>& expression)
        : levi::UnaryOperator<typename levi::transpose_type<EvaluableT>::type, EvaluableT>(expression, expression.cols(), expression.rows(), expression.name() + "^T")
    { }

    virtual typename levi::eval_return_type<typename levi::Evaluable<typename levi::transpose_type<EvaluableT>::type>::matrix_type>::type evaluateID(size_t callerID) final {
        if (callerID < this->m_evaluationRegister.size()) {
            this->m_evaluationRegister[callerID] = true;
            this->m_alreadyComputed = true;
        }
        return evaluate();
    }

    virtual typename levi::eval_return_type<typename levi::Evaluable<typename levi::transpose_type<EvaluableT>::type>::matrix_type>::type evaluate() final {
        return this->m_expression.evaluate().transpose();

        this->m_evaluationBuffer = this->m_expression.evaluate().transpose();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename levi::transpose_type<EvaluableT>::type>::derivative_evaluable>
    getNewColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) final {
        m_derivatives.resize(this->rows());

        for (Eigen::Index j = 0; j < this->rows(); ++j) {
            m_derivatives[j] = this->m_expression(column, j).getColumnDerivative(0, variable);
        }

        return levi::ExpressionComponent<typename levi::Evaluable<typename levi::transpose_type<EvaluableT>::type>::derivative_evaluable>::ComposeByRows(m_derivatives, "d(" + this->name() + ")/d" + variable->variableName());
    }

};

#endif // LEVI_OPERATORS_H
