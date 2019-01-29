/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_OPERATORS_EVALUABLES_H
#define SDIFF_OPERATORS_EVALUABLES_H

#include <sDiff/BasicEvaluables.h>
#include <sDiff/ForwardDeclarations.h>
#include <sDiff/Expression.h>

/**
 * Helper struct for determining the type resulting from an addition
 */
template <typename Scalar_lhs, typename Scalar_rhs>
struct sDiff::scalar_sum_return {
    //decltype allow to get the return type of the addition of a variable of type Scalar_lhs to a variable of type Scalar_rhs.
    typedef decltype (std::declval<Scalar_lhs>() + std::declval<Scalar_rhs>()) type;
};

/**
 * Helper struct for determining the type resulting from an addition of two matrices. Specialization for two matrices
 *
 */
template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
struct sDiff::matrix_sum_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<sDiff::is_valid_sum<lhsRows, lhsCols, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_sum_return<Scalar_lhs, Scalar_rhs>::type, std::min(lhsRows, rhsRows), std::min(lhsCols, rhsCols)> type; //here we assume that Eigen::Dynamic == -1, thus, given that it is a valid sum, the minimum will be -1 if present, thus using Eigen::Dynamic as output
};

/**
 * Helper struct for determining the type resulting from an addition of two matrices. Specialization for two scalars.
 *
 */
template<typename Scalar_lhs, typename Scalar_rhs>
struct sDiff::matrix_sum_return<Scalar_lhs, Scalar_rhs,
        typename std::enable_if<std::is_arithmetic<Scalar_lhs>::value && std::is_arithmetic<Scalar_rhs>::value>::type> {
    typedef typename sDiff::scalar_sum_return<Scalar_lhs, Scalar_rhs>::type type;
};

/**
 * Helper struct for determining the type resulting from an addition of two matrices. Specialization for a scalar and a matrix.
 *
 */
template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols>
struct sDiff::matrix_sum_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<std::is_arithmetic<Scalar>::value && sDiff::is_valid_sum<1,1, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_sum_return<Scalar, Scalar_rhs>::type, rhsRows, rhsCols> type;
};

/**
 * Helper struct for determining the type resulting from an addition of two matrices. Specialization for a matrix and a scalar
 *
 */
template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols>
struct sDiff::matrix_sum_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Scalar,
        typename std::enable_if<std::is_arithmetic<Scalar>::value && sDiff::is_valid_sum<1,1, lhsRows, lhsCols>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_sum_return<Scalar, Scalar_lhs>::type, lhsRows, lhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication
 */
template <typename Scalar_lhs, typename Scalar_rhs>
struct sDiff::scalar_product_return {
    //decltype allow to get the return type of the multiplication of a variable of type Scalar_lhs to a variable of type Scalar_rhs.
    typedef decltype (std::declval<Scalar_lhs>() * std::declval<Scalar_rhs>()) type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization for two matrices
 *
 */
template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
struct sDiff::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<sDiff::is_valid_product<lhsRows, lhsCols, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_product_return<Scalar_lhs, Scalar_rhs>::type, lhsRows, rhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization for a scalar and a matrix
 */
template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols>
struct sDiff::matrix_product_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_product_return<Scalar, Scalar_rhs>::type, rhsRows, rhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization for a matrix and a scalar.
 */
template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols>
struct sDiff::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Scalar,
        typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_product_return<Scalar, Scalar_lhs>::type, lhsRows, lhsCols> type;
};

/**
 * Helper struct for determining the type resulting from a multiplication of two matrices. Specialization for two scalars.
 */
template<typename Scalar_lhs, typename Scalar_rhs>
struct sDiff::matrix_product_return<Scalar_lhs, Scalar_rhs,
        typename std::enable_if<std::is_arithmetic<Scalar_lhs>::value && std::is_arithmetic<Scalar_rhs>::value>::type> {
    typedef typename sDiff::scalar_product_return<Scalar_lhs, Scalar_rhs>::type type;
};

template<typename Matrix>
struct sDiff::dynamic_block_return<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> {
    typedef Eigen::Matrix<typename Matrix::value_type, Eigen::Dynamic, Eigen::Dynamic> type;
};

template<typename Scalar>
struct sDiff::dynamic_block_return<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Scalar type;
};


/**
 * @brief The SumEvaluable. Implements the sum of two evaluables.
 */
template <class LeftEvaluable, class RightEvaluable>
class sDiff::SumEvaluable : public sDiff::Evaluable<typename sDiff::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type>{

    sDiff::ExpressionComponent<LeftEvaluable> m_lhs;
    sDiff::ExpressionComponent<RightEvaluable> m_rhs;

public:

    typedef typename sDiff::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type sum_type;

    SumEvaluable(const sDiff::ExpressionComponent<LeftEvaluable>& lhs, const sDiff::ExpressionComponent<RightEvaluable>& rhs)
        : sDiff::Evaluable<sum_type>(lhs.rows(), lhs.cols(), "(" + lhs.name() + " + " + rhs.name() + ")")
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    virtual const sum_type& evaluate() final {
        this->m_evaluationBuffer = m_lhs.evaluate() + m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<sum_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                      std::shared_ptr<sDiff::VariableBase> variable) final {
        sDiff::ExpressionComponent<typename sDiff::Evaluable<sum_type>::derivative_evaluable> sumDerivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            sumDerivative = m_lhs.getColumnDerivative(column, variable) + m_rhs.getColumnDerivative(column, variable);

            return sumDerivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            sDiff::ExpressionComponent<sDiff::NullEvaluable<typename sDiff::Evaluable<sum_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), this->cols(), "d" + this->name() + "/d" + variable->variableName());
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

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_lhs.isDependentFrom(variable) || m_rhs.isDependentFrom(variable);
    }

};

/**
 * @brief The SubtractionEvaluable. Implements the subtraction of two evaluables.
 */
template <class LeftEvaluable, class RightEvaluable>
class sDiff::SubtractionEvaluable : public sDiff::Evaluable<typename sDiff::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type>{

    sDiff::ExpressionComponent<LeftEvaluable> m_lhs;
    sDiff::ExpressionComponent<RightEvaluable> m_rhs;

public:

    typedef typename sDiff::matrix_sum_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type sum_type;

    SubtractionEvaluable(const sDiff::ExpressionComponent<LeftEvaluable>& lhs, const sDiff::ExpressionComponent<RightEvaluable>& rhs)
        : sDiff::Evaluable<sum_type>(lhs.rows(), lhs.cols(), "(" + lhs.name() + " - " + rhs.name() + ")")
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    virtual const sum_type& evaluate() final {
        this->m_evaluationBuffer = m_lhs.evaluate() - m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<sum_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                      std::shared_ptr<sDiff::VariableBase> variable) final {
        sDiff::ExpressionComponent<typename sDiff::Evaluable<sum_type>::derivative_evaluable> sumDerivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            sumDerivative = m_lhs.getColumnDerivative(column, variable) - m_rhs.getColumnDerivative(column, variable);

            return sumDerivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            sDiff::ExpressionComponent<sDiff::NullEvaluable<typename sDiff::Evaluable<sum_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), this->cols(), "d" + this->name() + "/d" + variable->variableName());
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

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_lhs.isDependentFrom(variable) || m_rhs.isDependentFrom(variable);
    }

};

template <class EvaluableT>
class sDiff::SignInvertedEvaluable : public sDiff::Evaluable<typename EvaluableT::matrix_type>{

    sDiff::ExpressionComponent<EvaluableT> m_expression;

public:

    SignInvertedEvaluable(const sDiff::ExpressionComponent<EvaluableT>& expression)
        : sDiff::Evaluable<typename EvaluableT::matrix_type>(expression.rows(), expression.cols(), "-" + expression.name())
        , m_expression(expression)
    { }

    virtual const typename EvaluableT::matrix_type& evaluate() final {
        this->m_evaluationBuffer = -m_expression.evaluate();

        return this->m_evaluationBuffer;
    }

    sDiff::ExpressionComponent<typename sDiff::Evaluable<typename EvaluableT::matrix_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                      std::shared_ptr<sDiff::VariableBase> variable) final {
        sDiff::ExpressionComponent<typename sDiff::Evaluable<typename EvaluableT::matrix_type>::derivative_evaluable> derivative;

        derivative = -m_expression.getColumnDerivative(column, variable);

        return derivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The ProductEvaluable. Implements the product of two evaluables.
 */
template <class LeftEvaluable, class RightEvaluable>
class sDiff::ProductEvaluable : public sDiff::Evaluable<typename sDiff::matrix_product_return<
        typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type>{

public:

    typedef typename sDiff::matrix_product_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type product_type;

private:

    sDiff::ExpressionComponent<LeftEvaluable> m_lhs;
    sDiff::ExpressionComponent<RightEvaluable> m_rhs;

    typedef sDiff::ExpressionComponent<typename sDiff::Evaluable<product_type>::derivative_evaluable> derivative_expression;

    template<bool lhsIsScalar, bool rhsIsScalar>
    derivative_expression get_derivative(sDiff::bool_value<lhsIsScalar>, sDiff::bool_value<rhsIsScalar>, Eigen::Index column, std::shared_ptr<sDiff::VariableBase> variable);

    /**
     * @brief Helper function for the derivative of the multiplication between two scalars
     */
    derivative_expression get_derivative(sDiff::bool_value<true>, sDiff::bool_value<true>, Eigen::Index column, std::shared_ptr<sDiff::VariableBase> variable) {
        derivative_expression derivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            derivative = m_rhs * m_lhs.getColumnDerivative(column, variable) + m_lhs * m_rhs.getColumnDerivative(column, variable);
            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            sDiff::ExpressionComponent<sDiff::NullEvaluable<typename sDiff::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), this->cols(), "d" + this->name() + "/d" + variable->variableName());
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
    derivative_expression get_derivative(sDiff::bool_value<true>, sDiff::bool_value<false>, Eigen::Index column, std::shared_ptr<sDiff::VariableBase> variable) {

        //k*A
        derivative_expression derivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            derivative = m_lhs * m_rhs.getColumnDerivative(column, variable) + m_rhs.col(column) * m_lhs.getColumnDerivative(0, variable);
            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            sDiff::ExpressionComponent<sDiff::NullEvaluable<typename sDiff::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), this->cols(), "d" + this->name() + "/d" + variable->variableName());
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
    derivative_expression get_derivative(sDiff::bool_value<false>, sDiff::bool_value<true>, Eigen::Index column, std::shared_ptr<sDiff::VariableBase> variable) {

        //A*k
        derivative_expression derivative;

        bool isLeftDependent = m_lhs.isDependentFrom(variable);
        bool isRightDependent = m_rhs.isDependentFrom(variable);

        if (isLeftDependent && isRightDependent) {
            derivative = m_rhs * m_lhs.getColumnDerivative(column, variable) + m_lhs.col(column) * m_rhs.getColumnDerivative(0, variable);
            return derivative;
        }

        if (!isLeftDependent && !isRightDependent) {
            sDiff::ExpressionComponent<sDiff::NullEvaluable<typename sDiff::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), this->cols(), "d" + this->name() + "/d" + variable->variableName());
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
    derivative_expression get_derivative(sDiff::bool_value<false>, sDiff::bool_value<false>, Eigen::Index column, std::shared_ptr<sDiff::VariableBase> variable) {
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
            sDiff::ExpressionComponent<sDiff::NullEvaluable<typename sDiff::Evaluable<product_type>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), this->cols(), "d" + this->name() + "/d" + variable->variableName());
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


    ProductEvaluable(const sDiff::ExpressionComponent<LeftEvaluable>& lhs, const sDiff::ExpressionComponent<RightEvaluable>& rhs)
        : sDiff::Evaluable<product_type>((lhs.rows() == 1 && lhs.cols() == 1 && rhs.rows() != 1)? rhs.rows() : lhs.rows(),
                                         (rhs.rows() == 1 && rhs.cols() == 1 && lhs.cols() != 1)? lhs.cols() : rhs.cols(),
                                         lhs.name() + " * " + rhs.name())
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    virtual const product_type& evaluate() final {

        this->m_evaluationBuffer = m_lhs.evaluate() * m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<product_type>::derivative_evaluable>
    getColumnDerivative(Eigen::Index column,
                        std::shared_ptr<sDiff::VariableBase> variable) final {

        return get_derivative(sDiff::bool_value<std::is_arithmetic<typename LeftEvaluable::matrix_type>::value>(),
                              sDiff::bool_value<std::is_arithmetic<typename RightEvaluable::matrix_type>::value>(),
                              column, variable);
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_lhs.isDependentFrom(variable) || m_rhs.isDependentFrom(variable);
    }

};

/**
 * @brief The RowEvaluable. To retrieve a specified row from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class sDiff::RowEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public sDiff::Evaluable<typename EvaluableT::row_type>
{
    sDiff::ExpressionComponent<EvaluableT> m_expression;
    Eigen::Index m_row;

public:

    RowEvaluable(const sDiff::ExpressionComponent<EvaluableT>& evaluable, Eigen::Index row)
        : sDiff::Evaluable<typename EvaluableT::row_type>(1, evaluable.cols(), "(" + evaluable.name() + ")(" + std::to_string(row) + ",:)")
        , m_expression(evaluable)
        , m_row(row)
    { }

    virtual const typename EvaluableT::row_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate().row(m_row);

        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<typename EvaluableT::row_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                           std::shared_ptr<sDiff::VariableBase> variable) final {
        sDiff::ExpressionComponent<typename sDiff::Evaluable<typename EvaluableT::row_type>::derivative_evaluable> newDerivative;

        newDerivative = sDiff::ExpressionComponent<sDiff::RowEvaluable<typename EvaluableT::derivative_evaluable>>(m_expression.getColumnDerivative(column, variable), m_row);

        return newDerivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }

};

/**
 * @brief The RowEvaluable. To retrieve a specified row from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class sDiff::RowEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public sDiff::Evaluable<typename EvaluableT::row_type>
{
    sDiff::ExpressionComponent<EvaluableT> m_expression;

public:

    RowEvaluable(const sDiff::ExpressionComponent<EvaluableT>& expression, Eigen::Index row)
        : sDiff::Evaluable<typename EvaluableT::row_type>(expression.name())
        , m_expression(expression)
    {
        assert(row == 0);
    }

    virtual const typename EvaluableT::row_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                      std::shared_ptr<sDiff::VariableBase> variable) final {
        assert(column == 0);

        return m_expression.getColumnDerivative(0, variable);
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The ColEvaluable. To retrieve a specified column from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class sDiff::ColEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public sDiff::Evaluable<typename EvaluableT::col_type>
{
    sDiff::ExpressionComponent<EvaluableT> m_expression;
    Eigen::Index m_col;

public:

    ColEvaluable(const sDiff::ExpressionComponent<EvaluableT>& evaluable, Eigen::Index col)
        : sDiff::Evaluable<typename EvaluableT::col_type>(evaluable.rows(), 1, "(" + evaluable.name() + ")(:," + std::to_string(col) + ")")
        , m_expression(evaluable)
        , m_col(col)
    { }

    virtual const typename EvaluableT::col_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate().col(m_col);

        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<typename EvaluableT::col_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                           std::shared_ptr<sDiff::VariableBase> variable) final {
        assert(column == 0);
        sDiff::ExpressionComponent<typename sDiff::Evaluable<typename EvaluableT::col_type>::derivative_evaluable> newDerivative;

        newDerivative = m_expression.getColumnDerivative(m_col, variable);

        return newDerivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }

};

/**
 * @brief The ColEvaluable. To retrieve a specified column from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class sDiff::ColEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public sDiff::Evaluable<typename EvaluableT::col_type>
{
    sDiff::ExpressionComponent<EvaluableT> m_expression;

public:

    ColEvaluable(const sDiff::ExpressionComponent<EvaluableT>& expression, Eigen::Index col)
        : sDiff::Evaluable<typename EvaluableT::col_type>(expression.name())
        , m_expression(expression)
    {
        assert(col == 0);
    }

    virtual const typename EvaluableT::col_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                      std::shared_ptr<sDiff::VariableBase> variable) final {
        assert(column == 0);

        return m_expression.getColumnDerivative(0, variable);
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The ElementEvaluable. To retrieve a specified element from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class sDiff::ElementEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public sDiff::Evaluable<typename EvaluableT::value_type>
{
    sDiff::ExpressionComponent<EvaluableT> m_expression;
    Eigen::Index m_row, m_col;

public:

    ElementEvaluable(const sDiff::ExpressionComponent<EvaluableT>& expression, Eigen::Index row, Eigen::Index col)
        : sDiff::Evaluable<typename EvaluableT::value_type>("[" + expression.name() + "](" + std::to_string(row) + ", " + std::to_string(col) + ")")
        , m_expression(expression)
        , m_row(row)
        , m_col(col)
    { }

    virtual const typename EvaluableT::value_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate()(m_row, m_col);

        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                             std::shared_ptr<sDiff::VariableBase> variable) final {
        assert(column == 0);
        sDiff::ExpressionComponent<typename sDiff::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> newDerivative;

        newDerivative = sDiff::ExpressionComponent<sDiff::RowEvaluable<typename EvaluableT::derivative_evaluable>>(m_expression.getColumnDerivative(m_col, variable), m_row);

        return newDerivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The ElementEvaluable. To retrieve a specified element from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class sDiff::ElementEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public sDiff::Evaluable<typename EvaluableT::value_type> {

    sDiff::ExpressionComponent<EvaluableT> m_expression;

public:

    ElementEvaluable(const sDiff::ExpressionComponent<EvaluableT>& expression, Eigen::Index row, Eigen::Index col)
        : sDiff::Evaluable<typename EvaluableT::value_type>(expression.name())
        , m_expression(expression)
    {
        assert(row == 0 && col == 0);
    }

    virtual const typename EvaluableT::value_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate();
        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                      std::shared_ptr<sDiff::VariableBase> variable) final {
        assert(column == 0);

        return m_expression.getColumnDerivative(0, variable);
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The BlockEvaluable. To retrieve a specified block from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class sDiff::BlockEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public sDiff::Evaluable<typename sDiff::dynamic_block_return<typename EvaluableT::matrix_type>::type>
{
    sDiff::ExpressionComponent<EvaluableT> m_expression;
    Eigen::Index m_startRow, m_startCol;

public:

    typedef typename sDiff::dynamic_block_return<typename EvaluableT::matrix_type>::type block_type;

    BlockEvaluable(const sDiff::ExpressionComponent<EvaluableT>& expression, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index numberOfRows, Eigen::Index numberOfCols)
        : sDiff::Evaluable<block_type>(numberOfRows, numberOfCols, "[" + expression.name() + "](" + std::to_string(startRow) + ":" + std::to_string(startRow + numberOfRows) + ", " + std::to_string(startCol) + ":" + std::to_string(startCol + numberOfCols) + ")")
        , m_expression(expression)
        , m_startRow(startRow)
        , m_startCol(startCol)
    {
        assert(((startRow + numberOfRows) <= expression.rows()) && ((startCol + numberOfCols) <= expression.cols()));
    }

    virtual const block_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate().block(m_startRow, m_startCol, this->rows(), this->cols());

        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<block_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column, std::shared_ptr<sDiff::VariableBase> variable) final {
        sDiff::ExpressionComponent<typename sDiff::Evaluable<block_type>::derivative_evaluable> newDerivative;

        newDerivative = m_expression.getColumnDerivative(m_startCol + column, variable).block(m_startRow, 0, this->rows(), variable->dimension());

        return newDerivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The BlockEvaluable. To retrieve a specified block from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class sDiff::BlockEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public sDiff::Evaluable<typename sDiff::dynamic_block_return<typename EvaluableT::matrix_type>::type> {

    sDiff::ExpressionComponent<EvaluableT> m_expression;

public:

    typedef typename sDiff::dynamic_block_return<typename EvaluableT::matrix_type>::type block_type;

    BlockEvaluable(const sDiff::ExpressionComponent<EvaluableT>& expression, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index numberOfRows, Eigen::Index numberOfCols)
        : sDiff::Evaluable<block_type>(expression.name())
        , m_expression(expression)
    {
        assert(startRow == 0 && startCol == 0 && numberOfRows == 1 && numberOfCols == 1);
    }

    virtual const block_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate();
        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<block_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                        std::shared_ptr<sDiff::VariableBase> variable) final {
        assert(column == 0);

        return m_expression.getColumnDerivative(0, variable);
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The CastEvaluable.
 *
 * It allows to assign an evaluable to an expression whose pointer cannot be directly casted. It assumes that the two evaluation buffers can be casted.
 */
template <class LeftEvaluable, class RightEvaluable>
class sDiff::CastEvaluable : public sDiff::Evaluable<typename LeftEvaluable::matrix_type> {

    sDiff::ExpressionComponent<RightEvaluable> m_rhs;

public:

    CastEvaluable(const sDiff::ExpressionComponent<RightEvaluable>& rhs)
        : sDiff::Evaluable<typename LeftEvaluable::matrix_type>(rhs.rows(), rhs.cols(), rhs.name())
        , m_rhs(rhs)
    { }

    virtual const typename LeftEvaluable::matrix_type& evaluate() final {
        this->m_evaluationBuffer = m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<typename LeftEvaluable::matrix_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                      std::shared_ptr<sDiff::VariableBase> variable) final {
        sDiff::ExpressionComponent<typename RightEvaluable::derivative_evaluable> rightDerivative = m_rhs.getColumnDerivative(column, variable);

        sDiff::ExpressionComponent<CastEvaluable<typename sDiff::Evaluable<typename LeftEvaluable::matrix_type>::derivative_evaluable, typename RightEvaluable::derivative_evaluable>> newCast(rightDerivative);

        return newCast;
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_rhs.isDependentFrom(variable);
    }
};

/**
 * @brief The SkewEvaluable.
 *
 * It allows to compute the skew symmetric matrix out of a three dimensional vector.
 */
template <typename EvaluableT>
class sDiff::SkewEvaluable : public sDiff::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>> {

    sDiff::ExpressionComponent<EvaluableT> m_expression;
    Eigen::Matrix<typename EvaluableT::value_type, 3, 1> m_vector;
    Eigen::Matrix<typename EvaluableT::value_type, 3, 3> m_derivativeBuffer;

public:

    SkewEvaluable(const sDiff::ExpressionComponent<EvaluableT>& expression)
        : sDiff::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>("skew(" + expression.name() + ")")
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

    virtual sDiff::ExpressionComponent<typename sDiff::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                                                  std::shared_ptr<sDiff::VariableBase> variable) final {

        assert( column < 3);
        sDiff::ExpressionComponent<typename sDiff::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>::derivative_evaluable> derivative;

        if (!m_expression.isDependentFrom(variable)) {
            sDiff::ExpressionComponent<sDiff::NullEvaluable<typename sDiff::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>::derivative_evaluable::matrix_type>> nullDerivative(this->rows(), this->cols(), "d" + this->name() + "/d" + variable->variableName());
            derivative = nullDerivative;
            return derivative;
        }

        if (column == 0) {

            m_derivativeBuffer.setZero();
            m_derivativeBuffer(1,2) = 1;
            m_derivativeBuffer(2,1) = -1;

            sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> col1(m_derivativeBuffer, "LeviCivita_ij0");

            derivative = col1 * m_expression.getColumnDerivative(0, variable);
            return derivative;

        }

        if (column == 1) {

            m_derivativeBuffer.setZero();
            m_derivativeBuffer(0,2) = -1;
            m_derivativeBuffer(2,0) = 1;

            sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> col2(m_derivativeBuffer, "LeviCivita_ij1");

            derivative = col2 * m_expression.getColumnDerivative(0, variable);
            return derivative;

        }

        if (column == 2) {

            m_derivativeBuffer.setZero();
            m_derivativeBuffer(0,1) = 1;
            m_derivativeBuffer(1,0) = -1;

            sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> col3(m_derivativeBuffer, "LeviCivita_ij2");

            derivative = col3 * m_expression.getColumnDerivative(0, variable);
            return derivative;

        }

        return derivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }

};

#endif // SDIFF_OPERATORS_H
