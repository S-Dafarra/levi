/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_ACCESSOREVALUABLES_H
#define LEVI_ACCESSOREVALUABLES_H

#include <levi/BasicEvaluables.h>
#include <levi/ForwardDeclarations.h>
#include <levi/Expression.h>
#include <levi/OperatorsBase.h>


/**
 * Helper struct for determining the type resulting from a block extraction. Specialization for a matrix.
 */
template<typename Matrix>
struct levi::dynamic_block_return<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> {
    typedef Eigen::Matrix<typename Matrix::value_type, Eigen::Dynamic, Eigen::Dynamic> type;
};

/**
 * Helper struct for determining the type resulting from a block extraction. Specialization for a scalar.
 */
template<typename Scalar>
struct levi::dynamic_block_return<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Scalar type;
};

/**
 * Helper struct for determining the type resulting from a transposition. Specialization for a matrix.
 */
template<typename EvaluableT>
struct levi::transpose_type<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type> {
    typedef Eigen::Matrix<typename EvaluableT::value_type, EvaluableT::cols_at_compile_time, EvaluableT::rows_at_compile_time> type;
};

/**
 * Helper struct for determining the type resulting from a transposition. Specialization for a scalar.
 */
template<typename EvaluableT>
struct levi::transpose_type<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type> {
    typedef typename EvaluableT::matrix_type type;
};



/**
 * @brief The RowEvaluable. To retrieve a specified row from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class levi::RowEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::UnaryOperator<typename EvaluableT::row_type, EvaluableT>
{
    Eigen::Index m_row;

public:

    RowEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index row)
        : levi::UnaryOperator<typename EvaluableT::row_type, EvaluableT>(expression, 1, expression.cols(), "(" + expression.name() + ")(" + std::to_string(row) + ",:)")
        , m_row(row)
    { }

    virtual const typename EvaluableT::row_type& evaluate() final {
        this->m_evaluationBuffer = this->m_expression.evaluate().row(m_row);

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::row_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                                            std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::row_type>::derivative_evaluable> newDerivative;

        newDerivative = levi::ExpressionComponent<levi::RowEvaluable<typename EvaluableT::derivative_evaluable>>(this->m_expression.getColumnDerivative(column, variable), m_row);

        return newDerivative;
    }

};

/**
 * @brief The RowEvaluable. To retrieve a specified row from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class levi::RowEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::UnaryOperator<typename EvaluableT::row_type, EvaluableT>
{

public:

    RowEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index row)
        : levi::UnaryOperator<typename EvaluableT::row_type, EvaluableT>(expression, expression.name())
    {
        levi::unused(row);
        assert(row == 0);
    }

    virtual const typename EvaluableT::row_type& evaluate() final {
        this->m_evaluationBuffer = this->m_expression.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                        std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);

        return this->m_expression.getColumnDerivative(0, variable);
    }

};

/**
 * @brief The ColEvaluable. To retrieve a specified column from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class levi::ColEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::UnaryOperator<typename EvaluableT::col_type, EvaluableT>
{
    Eigen::Index m_col;

public:

    ColEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index col)
        : levi::UnaryOperator<typename EvaluableT::col_type, EvaluableT>(expression, expression.rows(), 1, "(" + expression.name() + ")(:," + std::to_string(col) + ")")
        , m_col(col)
    { }

    virtual const typename EvaluableT::col_type& evaluate() final {
        this->m_evaluationBuffer = this->m_expression.evaluate().col(m_col);

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::col_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                                            std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);
        levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::col_type>::derivative_evaluable> newDerivative;

        newDerivative = this->m_expression.getColumnDerivative(m_col, variable);

        return newDerivative;
    }

};

/**
 * @brief The ColEvaluable. To retrieve a specified column from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class levi::ColEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::UnaryOperator<typename EvaluableT::col_type, EvaluableT>
{

public:

    ColEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index col)
        : levi::UnaryOperator<typename EvaluableT::col_type, EvaluableT>(expression, expression.name())
    {
        levi::unused(col);
        assert(col == 0);
    }

    virtual const typename EvaluableT::col_type& evaluate() final {
        this->m_evaluationBuffer = this->m_expression.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                        std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);

        return this->m_expression.getColumnDerivative(0, variable);
    }

};

/**
 * @brief The ElementEvaluable. To retrieve a specified element from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class levi::ElementEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::UnaryOperator<typename EvaluableT::value_type, EvaluableT>
{
    Eigen::Index m_row, m_col;

public:

    ElementEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index row, Eigen::Index col)
        : levi::UnaryOperator<typename EvaluableT::value_type, EvaluableT>(expression, "[" + expression.name() + "](" + std::to_string(row) + ", " + std::to_string(col) + ")")
        , m_row(row)
        , m_col(col)
    { }

    virtual const typename EvaluableT::value_type& evaluate() final {
        this->m_evaluationBuffer = this->m_expression.evaluate()(m_row, m_col);

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                                              std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);
        levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> newDerivative;

        newDerivative = levi::ExpressionComponent<levi::RowEvaluable<typename EvaluableT::derivative_evaluable>>(this->m_expression.getColumnDerivative(m_col, variable), m_row);

        return newDerivative;
    }

};

/**
 * @brief The ElementEvaluable. To retrieve a specified element from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class levi::ElementEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::UnaryOperator<typename EvaluableT::value_type, EvaluableT> {

public:

    ElementEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index row, Eigen::Index col)
        : levi::UnaryOperator<typename EvaluableT::value_type, EvaluableT>(expression, expression.name())
    {
        levi::unused(row, col);
        assert(row == 0 && col == 0);
    }

    virtual const typename EvaluableT::value_type& evaluate() final {
        this->m_evaluationBuffer = this->m_expression.evaluate();
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                        std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);

        return this->m_expression.getColumnDerivative(0, variable);
    }
};

/**
 * @brief The BlockEvaluable. To retrieve a specified block from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class levi::BlockEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::UnaryOperator<typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type, EvaluableT>
{
    Eigen::Index m_startRow, m_startCol;

public:

    typedef typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type block_type;

    BlockEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index numberOfRows, Eigen::Index numberOfCols)
        : levi::UnaryOperator<block_type, EvaluableT>(expression, numberOfRows, numberOfCols,
                                                      "[" + expression.name() + "](" + std::to_string(startRow) + ":" + std::to_string(startRow + numberOfRows) + ", " + std::to_string(startCol) + ":" + std::to_string(startCol + numberOfCols) + ")")
        , m_startRow(startRow)
        , m_startCol(startCol)
    {
        assert(((startRow + numberOfRows) <= expression.rows()) && ((startCol + numberOfCols) <= expression.cols()));
    }

    virtual const block_type& evaluate() final {
        this->m_evaluationBuffer = this->m_expression.evaluate().block(m_startRow, m_startCol, this->rows(), this->cols());

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<block_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename levi::Evaluable<block_type>::derivative_evaluable> newDerivative;

        newDerivative = this->m_expression.getColumnDerivative(m_startCol + column, variable).block(m_startRow, 0, this->rows(), variable->dimension());

        return newDerivative;
    }

};

/**
 * @brief The BlockEvaluable. To retrieve a specified block from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class levi::BlockEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::UnaryOperator<typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type, EvaluableT> {

public:

    typedef typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type block_type;

    BlockEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index numberOfRows, Eigen::Index numberOfCols)
        : levi::UnaryOperator<block_type, EvaluableT>(expression, expression.name())
    {
        levi::unused(startRow, startCol, numberOfRows, numberOfCols);
        assert(startRow == 0 && startCol == 0 && numberOfRows == 1 && numberOfCols == 1);
    }

    virtual const block_type& evaluate() final {
        this->m_evaluationBuffer = this->m_expression.evaluate();
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<block_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                         std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);

        return this->m_expression.getColumnDerivative(0, variable);
    }

};


/**
 * @brief The CastEvaluable.
 *
 * It allows to assign an evaluable to an expression whose pointer cannot be directly casted. It assumes that the two evaluation buffers can be casted.
 */
template <class LeftEvaluable, class RightEvaluable>
class levi::CastEvaluable : public levi::UnaryOperator<typename LeftEvaluable::matrix_type, RightEvaluable> {

public:

    CastEvaluable(const levi::ExpressionComponent<RightEvaluable>& rhs)
        : levi::UnaryOperator<typename LeftEvaluable::matrix_type, RightEvaluable>(rhs, rhs.rows(), rhs.cols(), rhs.name())
    { }

    virtual const typename LeftEvaluable::matrix_type& evaluate() final {
        this->m_evaluationBuffer = this->m_expression.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename LeftEvaluable::matrix_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                                                  std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename RightEvaluable::derivative_evaluable> rightDerivative = this->m_expression.getColumnDerivative(column, variable);

        levi::ExpressionComponent<CastEvaluable<typename levi::Evaluable<typename LeftEvaluable::matrix_type>::derivative_evaluable, typename RightEvaluable::derivative_evaluable>> newCast(rightDerivative);

        return newCast;
    }
};

#endif // LEVI_ACCESSOREVALUABLES_H
