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

template <typename Scalar_lhs, typename Scalar_rhs>
struct sDiff::scalar_sum_return {
    typedef decltype (std::declval<Scalar_lhs>() + std::declval<Scalar_rhs>()) type;
};

template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
struct sDiff::matrix_sum_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<sDiff::is_valid_sum<lhsRows, lhsCols, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_sum_return<Scalar_lhs, Scalar_rhs>::type, std::min(lhsRows, rhsRows), std::min(lhsCols, rhsCols)> type; //here we assume that Eigen::Dynamic == -1, thus, given that it is a valid sum, the minimum will be -1 if present, thus using Eigen::Dynamic as output
};

template<typename Scalar_lhs, typename Scalar_rhs>
struct sDiff::matrix_sum_return<Scalar_lhs, Scalar_rhs,
        typename std::enable_if<std::is_arithmetic<Scalar_lhs>::value && std::is_arithmetic<Scalar_rhs>::value>::type> {
    typedef typename sDiff::scalar_sum_return<Scalar_lhs, Scalar_rhs>::type type;
};

template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols>
struct sDiff::matrix_sum_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<std::is_arithmetic<Scalar>::value && sDiff::is_valid_sum<1,1, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_sum_return<Scalar, Scalar_rhs>::type, rhsRows, rhsCols> type;
};

template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols>
struct sDiff::matrix_sum_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Scalar,
        typename std::enable_if<std::is_arithmetic<Scalar>::value && sDiff::is_valid_sum<1,1, lhsRows, lhsCols>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_sum_return<Scalar, Scalar_lhs>::type, lhsRows, lhsCols> type;
};

template <typename Scalar_lhs, typename Scalar_rhs>
struct sDiff::scalar_product_return {
    typedef decltype (std::declval<Scalar_lhs>() * std::declval<Scalar_rhs>()) type;
};

template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
struct sDiff::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<sDiff::is_valid_product<lhsRows, lhsCols, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_product_return<Scalar_lhs, Scalar_rhs>::type, lhsRows, rhsCols> type;
};

template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols>
struct sDiff::matrix_product_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_product_return<Scalar, Scalar_rhs>::type, rhsRows, rhsCols> type;
};

template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols>
struct sDiff::matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Scalar,
        typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Eigen::Matrix<typename sDiff::scalar_product_return<Scalar, Scalar_lhs>::type, lhsRows, lhsCols> type;
};

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

        sumDerivative = m_lhs.getColumnDerivative(column, variable) + m_rhs.getColumnDerivative(column, variable);

        return sumDerivative;
    }

};

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

        sumDerivative = m_lhs.getColumnDerivative(column, variable) - m_rhs.getColumnDerivative(column, variable);

        return sumDerivative;
    }

};

template <class LeftEvaluable, class RightEvaluable>
class sDiff::ProductEvaluable : public sDiff::Evaluable<typename sDiff::matrix_product_return<
        typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type>{

    sDiff::ExpressionComponent<LeftEvaluable> m_lhs;
    sDiff::ExpressionComponent<RightEvaluable> m_rhs;

public:

    typedef typename sDiff::matrix_product_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type product_type;

    ProductEvaluable(const sDiff::ExpressionComponent<LeftEvaluable>& lhs, const sDiff::ExpressionComponent<RightEvaluable>& rhs)
        : sDiff::Evaluable<product_type>(lhs.rows(), rhs.cols(), lhs.name() + " * " + rhs.name())
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    virtual const product_type& evaluate() final {

        this->m_evaluationBuffer = m_lhs.evaluate() * m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

//    virtual std::shared_ptr<typename sDiff::Evaluable<product_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
//                                                                                                               std::shared_ptr<sDiff::VariableBase> variable) final {
//        return this->m_derivative;
//    }

};

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

};

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
};

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
        assert(column == m_col);
        sDiff::ExpressionComponent<typename sDiff::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> newDerivative;

        newDerivative = sDiff::ExpressionComponent<sDiff::RowEvaluable<typename EvaluableT::derivative_evaluable>>(m_expression.getColumnDerivative(m_col, variable), m_row);

        return newDerivative;
    }
};

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
};

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
};

#endif // SDIFF_OPERATORS_H
