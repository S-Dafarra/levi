/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_ACCESSOREVALUABLES_H
#define SDIFF_ACCESSOREVALUABLES_H

#include <sDiff/BasicEvaluables.h>
#include <sDiff/ForwardDeclarations.h>
#include <sDiff/Expression.h>


/**
 * Helper struct for determining the type resulting from a block extraction. Specialization for a matrix.
 */
template<typename Matrix>
struct sDiff::dynamic_block_return<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> {
    typedef Eigen::Matrix<typename Matrix::value_type, Eigen::Dynamic, Eigen::Dynamic> type;
};

/**
 * Helper struct for determining the type resulting from a block extraction. Specialization for a scalar.
 */
template<typename Scalar>
struct sDiff::dynamic_block_return<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Scalar type;
};

/**
 * Helper struct for determining the type resulting from a transposition. Specialization for a matrix.
 */
template<typename EvaluableT>
struct sDiff::transpose_type<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type> {
    typedef Eigen::Matrix<typename EvaluableT::value_type, EvaluableT::cols_at_compile_time, EvaluableT::rows_at_compile_time> type;
};

/**
 * Helper struct for determining the type resulting from a transposition. Specialization for a scalar.
 */
template<typename EvaluableT>
struct sDiff::transpose_type<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type> {
    typedef typename EvaluableT::matrix_type type;
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

#endif // SDIFF_ACCESSOREVALUABLES_H
