/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_OPERATORS_EVALUABLES_H
#define LEVI_OPERATORS_EVALUABLES_H

#include <levi/BasicEvaluables.h>
#include <levi/ForwardDeclarations.h>
#include <levi/Expression.h>

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
template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
struct levi::matrix_sum_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<levi::is_valid_sum<lhsRows, lhsCols, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename levi::scalar_sum_return<Scalar_lhs, Scalar_rhs>::type, std::min(lhsRows, rhsRows), std::min(lhsCols, rhsCols)> type; //here we assume that Eigen::Dynamic == -1, thus, given that it is a valid sum, the minimum will be -1 if present, thus using Eigen::Dynamic as output
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
template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols>
struct levi::matrix_sum_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<std::is_arithmetic<Scalar>::value && levi::is_valid_sum<1,1, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename levi::scalar_sum_return<Scalar, Scalar_rhs>::type, rhsRows, rhsCols> type;
};

/**
 * Helper struct for determining the type resulting from an addition of two matrices. Specialization for a matrix and a scalar
 *
 */
template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols>
struct levi::matrix_sum_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Scalar,
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
template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
struct levi::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<levi::is_valid_product<lhsRows, lhsCols, rhsRows, rhsCols>::value && !(lhsRows == 1 && lhsCols == 1) && !(rhsRows == 1 && rhsCols == 1)>::type> {
    typedef Eigen::Matrix<typename levi::scalar_product_return<Scalar_lhs, Scalar_rhs>::type, lhsRows, rhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization in the case the lhs has exactly one row and one column.
 *
 */
template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
struct levi::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<lhsRows == 1 && lhsCols == 1>::type> {
    typedef Eigen::Matrix<typename levi::scalar_product_return<Scalar_lhs, Scalar_rhs>::type, rhsRows, rhsCols> type;
};


/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization in the case the rhs has exactly one row and one column.
 *
 */
template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
struct levi::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<rhsRows == 1 && rhsCols == 1>::type> {
    typedef Eigen::Matrix<typename levi::scalar_product_return<Scalar_lhs, Scalar_rhs>::type, lhsRows, lhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization for a scalar and a matrix
 */
template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols>
struct levi::matrix_product_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Eigen::Matrix<typename levi::scalar_product_return<Scalar, Scalar_rhs>::type, rhsRows, rhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization for a matrix and a scalar.
 */
template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols>
struct levi::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Scalar,
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
class levi::SumEvaluable : public levi::Evaluable<typename levi::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type>{

    levi::ExpressionComponent<LeftEvaluable> m_lhs;
    levi::ExpressionComponent<RightEvaluable> m_rhs;

public:

    typedef typename levi::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type sum_type;

    SumEvaluable(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs)
        : levi::Evaluable<sum_type>(lhs.rows(), lhs.cols(), "(" + lhs.name() + " + " + rhs.name() + ")")
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    virtual const sum_type& evaluate() final {
        this->m_evaluationBuffer = m_lhs.evaluate() + m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<sum_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                    std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename levi::Evaluable<sum_type>::derivative_evaluable> sumDerivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            sumDerivative = m_lhs.getColumnDerivative(column, variable) + m_rhs.getColumnDerivative(column, variable);

            return sumDerivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<sum_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d" + this->name() + "/d" + variable->variableName());
            sumDerivative = nullDerivative;
            return sumDerivative;
        }

        if (isLeftDependent) {
            sumDerivative = m_lhs.getColumnDerivative(column, variable);
            return sumDerivative;
        } else {
            sumDerivative = m_rhs.getColumnDerivative(column, variable);
            return sumDerivative;
        }
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_lhs.isDependentFrom(variable) || m_rhs.isDependentFrom(variable);
    }

};

/**
 * @brief The SubtractionEvaluable. Implements the subtraction of two evaluables.
 */
template <class LeftEvaluable, class RightEvaluable>
class levi::SubtractionEvaluable : public levi::Evaluable<typename levi::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type>{

    levi::ExpressionComponent<LeftEvaluable> m_lhs;
    levi::ExpressionComponent<RightEvaluable> m_rhs;

public:

    typedef typename levi::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type sum_type;

    SubtractionEvaluable(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs)
        : levi::Evaluable<sum_type>(lhs.rows(), lhs.cols(), "(" + lhs.name() + " - " + rhs.name() + ")")
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    virtual const sum_type& evaluate() final {
        this->m_evaluationBuffer = m_lhs.evaluate() - m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<sum_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                    std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename levi::Evaluable<sum_type>::derivative_evaluable> sumDerivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            sumDerivative = m_lhs.getColumnDerivative(column, variable) - m_rhs.getColumnDerivative(column, variable);

            return sumDerivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<sum_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d" + this->name() + "/d" + variable->variableName());
            sumDerivative = nullDerivative;
            return sumDerivative;
        }

        if (isLeftDependent) {
            sumDerivative = m_lhs.getColumnDerivative(column, variable);
            return sumDerivative;
        } else {
            sumDerivative = - m_rhs.getColumnDerivative(column, variable);
            return sumDerivative;
        }
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_lhs.isDependentFrom(variable) || m_rhs.isDependentFrom(variable);
    }

};

template <class EvaluableT>
class levi::SignInvertedEvaluable : public levi::Evaluable<typename EvaluableT::matrix_type>{

    levi::ExpressionComponent<EvaluableT> m_expression;

public:

    SignInvertedEvaluable(const levi::ExpressionComponent<EvaluableT>& expression)
        : levi::Evaluable<typename EvaluableT::matrix_type>(expression.rows(), expression.cols(), "-" + expression.name())
        , m_expression(expression)
    { }

    virtual const typename EvaluableT::matrix_type& evaluate() final {
        this->m_evaluationBuffer = -m_expression.evaluate();

        return this->m_evaluationBuffer;
    }

    levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::matrix_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                    std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::matrix_type>::derivative_evaluable> derivative;

        derivative = -m_expression.getColumnDerivative(column, variable);

        return derivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The ProductEvaluable. Implements the product of two evaluables.
 */
template <class LeftEvaluable, class RightEvaluable>
class levi::ProductEvaluable : public levi::Evaluable<typename levi::matrix_product_return<
        typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type>{

public:

    typedef typename levi::matrix_product_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type product_type;

private:

    levi::ExpressionComponent<LeftEvaluable> m_lhs;
    levi::ExpressionComponent<RightEvaluable> m_rhs;

    typedef levi::ExpressionComponent<typename levi::Evaluable<product_type>::derivative_evaluable> derivative_expression;

    template<bool lhsIsScalar, bool rhsIsScalar>
    derivative_expression get_derivative(levi::bool_value<lhsIsScalar>, levi::bool_value<rhsIsScalar>, Eigen::Index column, std::shared_ptr<levi::VariableBase> variable);

    /**
     * @brief Helper function for the derivative of the multiplication between two scalars
     */
    derivative_expression get_derivative(levi::bool_value<true>, levi::bool_value<true>, Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {
        derivative_expression derivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            derivative = m_rhs * m_lhs.getColumnDerivative(column, variable) + m_lhs * m_rhs.getColumnDerivative(column, variable);
            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d(" + this->name() + ")/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (isLeftDependent) {
            derivative = m_rhs * m_lhs.getColumnDerivative(column, variable);
            return derivative;
        } else {
            derivative = m_lhs * m_rhs.getColumnDerivative(column, variable);
            return derivative;
        }
    }

    /**
     * @brief Helper function for the derivative of the multiplication between a scalar and a matrix.
     */
    derivative_expression get_derivative(levi::bool_value<true>, levi::bool_value<false>, Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {

        //k*A
        derivative_expression derivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            derivative = m_lhs * m_rhs.getColumnDerivative(column, variable) + m_rhs.col(column) * m_lhs.getColumnDerivative(0, variable);
            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d(" + this->name() + ")/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (isLeftDependent) {
            derivative = m_rhs.col(column) * m_lhs.getColumnDerivative(0, variable);
            return derivative;
        } else {
            derivative = m_lhs * m_rhs.getColumnDerivative(column, variable);
            return derivative;
        }
    }

    /**
     * @brief Helper function for the derivative of the multiplication between a matrix and a scalar.
     */
    derivative_expression get_derivative(levi::bool_value<false>, levi::bool_value<true>, Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {

        //A*k
        derivative_expression derivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            derivative = m_rhs * m_lhs.getColumnDerivative(column, variable) + m_lhs.col(column) * m_rhs.getColumnDerivative(0, variable);
            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d(" + this->name() + ")/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (isLeftDependent) {
            derivative = m_rhs * m_lhs.getColumnDerivative(column, variable);
            return derivative;
        } else {
            derivative = m_lhs.col(column) * m_rhs.getColumnDerivative(0, variable);
            return derivative;
        }
    }

    /**
     * @brief Helper function for the derivative of the multiplication between two matrices.
     */
    derivative_expression get_derivative(levi::bool_value<false>, levi::bool_value<false>, Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {
        derivative_expression derivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {

            derivative = m_lhs * m_rhs.getColumnDerivative(column, variable);

            for (size_t i = 0; i < m_rhs.rows(); ++i) {
                derivative = derivative + m_lhs.getColumnDerivative(i, variable) * m_rhs(i, column);
            }

            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), variable->dimension(), "d(" + this->name() + ")/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (isLeftDependent) {

            derivative = m_lhs.getColumnDerivative(0, variable) * m_rhs(0, column);

            for (size_t i = 1; i < m_rhs.rows(); ++i) {
                derivative = derivative + m_lhs.getColumnDerivative(i, variable) * m_rhs(i, column);
            }

            return derivative;

        } else {

            derivative = m_lhs * m_rhs.getColumnDerivative(column, variable);

            return derivative;
        }

    }

public:


    ProductEvaluable(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs)
        : levi::Evaluable<product_type>((lhs.rows() == 1 && lhs.cols() == 1 && rhs.rows() != 1)? rhs.rows() : lhs.rows(),
                                        (rhs.rows() == 1 && rhs.cols() == 1 && lhs.cols() != 1)? lhs.cols() : rhs.cols(),
                                        lhs.name() + " * " + rhs.name())
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    virtual const product_type& evaluate() final {

        this->m_evaluationBuffer = m_lhs.evaluate() * m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<product_type>::derivative_evaluable>
    getColumnDerivative(Eigen::Index column,
                        std::shared_ptr<levi::VariableBase> variable) final {

        return get_derivative(levi::bool_value<std::is_arithmetic<typename LeftEvaluable::matrix_type>::value>(),
                              levi::bool_value<std::is_arithmetic<typename RightEvaluable::matrix_type>::value>(),
                              column, variable);
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_lhs.isDependentFrom(variable) || m_rhs.isDependentFrom(variable);
    }

};

template <class EvaluableT>
class levi::PowEvaluable : public levi::Evaluable<typename EvaluableT::value_type> {

    levi::ExpressionComponent<EvaluableT> m_expression;
    typename EvaluableT::value_type m_exponent;

    template<bool rhsIsScalar>
    double get_value(levi::bool_value<rhsIsScalar>);

    double get_value(levi::bool_value<true>) {
        return m_expression.evaluate();
    }

    double get_value(levi::bool_value<false>) {
        return m_expression.evaluate()(0,0);
    }

public:

    PowEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, typename EvaluableT::value_type exponent)
        : levi::Evaluable<typename EvaluableT::value_type>("(" +expression.name() + ")^(" + std::to_string(exponent) + ")")
        , m_expression(expression)
        , m_exponent(exponent)
    {
        assert(m_expression.rows() == 1 && m_expression.cols() == 1);
    }

    virtual const typename EvaluableT::value_type& evaluate() final {

        this->m_evaluationBuffer = std::pow(get_value(levi::bool_value<std::is_arithmetic<typename EvaluableT::matrix_type>::value>()), m_exponent);

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                           std::shared_ptr<levi::VariableBase> variable) final {
        assert(column == 0);

        levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> derivative;

        derivative = m_expression.pow(m_exponent - 1) * m_exponent * m_expression.getColumnDerivative(column, variable);

        return derivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }

};

template <class LeftEvaluable, class RightEvaluable>
class levi::DivisionEvaluable : public levi::Evaluable<typename levi::matrix_product_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::value_type>::type>
{
public:

    typedef typename levi::matrix_product_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::value_type>::type product_type;

private:

    levi::ExpressionComponent<LeftEvaluable> m_lhs;
    levi::ExpressionComponent<RightEvaluable> m_rhs;

    template<bool rhsIsScalar>
    double get_rhs_value(levi::bool_value<rhsIsScalar>);

    double get_rhs_value(levi::bool_value<true>) {
        return m_rhs.evaluate();
    }

    double get_rhs_value(levi::bool_value<false>) {
        return m_rhs.evaluate()(0,0);
    }

public:

    DivisionEvaluable(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs)
        : levi::Evaluable<product_type>(lhs.rows(), rhs.cols(), lhs.name() + "/(" + rhs.name() + ")")
        , m_lhs(lhs)
        , m_rhs(rhs)
    {
        assert(rhs.rows() == 1 && rhs.cols() == 1);
    }

    virtual const product_type& evaluate() final {

        this->m_evaluationBuffer = m_lhs.evaluate() / get_rhs_value(levi::bool_value<std::is_arithmetic<typename RightEvaluable::matrix_type>::value>());

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<product_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) final {

        levi::ExpressionComponent<typename levi::Evaluable<product_type>::derivative_evaluable> derivative;

        derivative = (m_lhs * m_rhs.pow(-1)).getColumnDerivative(column, variable);

        return derivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_lhs.isDependentFrom(variable) || m_rhs.isDependentFrom(variable);
    }
};

/**
 * @brief The SkewEvaluable.
 *
 * It allows to compute the skew symmetric matrix out of a three dimensional vector.
 */
template <typename EvaluableT>
class levi::SkewEvaluable : public levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>> {

    levi::ExpressionComponent<EvaluableT> m_expression;
    Eigen::Matrix<typename EvaluableT::value_type, 3, 1> m_vector;
    Eigen::Matrix<typename EvaluableT::value_type, 3, 3> m_derivativeBuffer;

public:

    SkewEvaluable(const levi::ExpressionComponent<EvaluableT>& expression)
        : levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>("skew(" + expression.name() + ")")
        , m_expression(expression)
    {
        assert((expression.rows() == 3) && (expression.cols() == 1));
    }

    virtual const Eigen::Matrix<typename EvaluableT::value_type, 3, 3>& evaluate() final {
        m_vector = m_expression.evaluate();

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

    virtual levi::ExpressionComponent<typename levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                                                std::shared_ptr<levi::VariableBase> variable) final {

        assert( column < 3);
        levi::ExpressionComponent<typename levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>::derivative_evaluable> derivative;

        if (!m_expression.isDependentFrom(variable)) {
            levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), this->cols(), "d" + this->name() + "/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (column == 0) {

            m_derivativeBuffer.setZero();
            m_derivativeBuffer(1,2) = 1;
            m_derivativeBuffer(2,1) = -1;

            levi::ExpressionComponent<levi::ConstantEvaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> col1(m_derivativeBuffer, "LeviCivita_ij0");

            derivative = col1 * m_expression.getColumnDerivative(0, variable);
            return derivative;

        }

        if (column == 1) {

            m_derivativeBuffer.setZero();
            m_derivativeBuffer(0,2) = -1;
            m_derivativeBuffer(2,0) = 1;

            levi::ExpressionComponent<levi::ConstantEvaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> col2(m_derivativeBuffer, "LeviCivita_ij1");

            derivative = col2 * m_expression.getColumnDerivative(0, variable);
            return derivative;

        }

        if (column == 2) {

            m_derivativeBuffer.setZero();
            m_derivativeBuffer(0,1) = 1;
            m_derivativeBuffer(1,0) = -1;

            levi::ExpressionComponent<levi::ConstantEvaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> col3(m_derivativeBuffer, "LeviCivita_ij2");

            derivative = col3 * m_expression.getColumnDerivative(0, variable);
            return derivative;

        }

        return derivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }

};

template <typename EvaluableT>
class levi::TransposeEvaluable : public levi::Evaluable<typename levi::transpose_type<EvaluableT>::type> {

    levi::ExpressionComponent<EvaluableT> m_expression;
    std::vector<levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::value_type>::derivative_evaluable>> m_derivatives;

public:

    TransposeEvaluable(const levi::ExpressionComponent<EvaluableT>& expression)
        : levi::Evaluable<typename levi::transpose_type<EvaluableT>::type>(expression.cols(), expression.rows(), expression.name() + "^T")
        , m_expression(expression)
    { }

    virtual const typename levi::Evaluable<typename levi::transpose_type<EvaluableT>::type>::matrix_type & evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate().transpose();

        return this->m_evaluationBuffer;
    }

    levi::ExpressionComponent<typename levi::Evaluable<typename levi::transpose_type<EvaluableT>::type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                                   std::shared_ptr<levi::VariableBase> variable) final {
        m_derivatives.resize(this->rows());

        for (size_t j = 0; j < this->rows(); ++j) {
            m_derivatives[j] = m_expression(column, j).getColumnDerivative(0, variable);
        }

        return levi::ExpressionComponent<typename levi::Evaluable<typename levi::transpose_type<EvaluableT>::type>::derivative_evaluable>::ComposeByRows(m_derivatives, "d(" + this->name() + ")/d" + variable->variableName());
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }

};

#endif // LEVI_OPERATORS_H