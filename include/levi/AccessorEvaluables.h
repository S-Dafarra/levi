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
        : public levi::Evaluable<typename EvaluableT::row_type>
{
    levi::ExpressionComponent<EvaluableT> m_expression;
    Eigen::Index m_row;

public:

    RowEvaluable(const levi::ExpressionComponent<EvaluableT>& evaluable, Eigen::Index row)
        : levi::Evaluable<typename EvaluableT::row_type>(1, evaluable.cols(), "(" + evaluable.name() + ")(" + std::to_string(row) + ",:)")
        , m_expression(evaluable)
        , m_row(row)
    { }

    virtual bool isNew(size_t callerID) final{
        if (m_expression.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual const typename EvaluableT::row_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate().row(m_row);

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::row_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                         std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::row_type>::derivative_evaluable> newDerivative;

        newDerivative = levi::ExpressionComponent<levi::RowEvaluable<typename EvaluableT::derivative_evaluable>>(m_expression.getColumnDerivative(column, variable), m_row);

        return newDerivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }

};

/**
 * @brief The RowEvaluable. To retrieve a specified row from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class levi::RowEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::Evaluable<typename EvaluableT::row_type>
{
    levi::ExpressionComponent<EvaluableT> m_expression;

public:

    RowEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index row)
        : levi::Evaluable<typename EvaluableT::row_type>(expression.name())
        , m_expression(expression)
    {
        levi::unused(row);
        assert(row == 0);
    }

    virtual bool isNew(size_t callerID) final{
        if (m_expression.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual const typename EvaluableT::row_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                     std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);

        return m_expression.getColumnDerivative(0, variable);
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The ColEvaluable. To retrieve a specified column from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class levi::ColEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::Evaluable<typename EvaluableT::col_type>
{
    levi::ExpressionComponent<EvaluableT> m_expression;
    Eigen::Index m_col;

public:

    ColEvaluable(const levi::ExpressionComponent<EvaluableT>& evaluable, Eigen::Index col)
        : levi::Evaluable<typename EvaluableT::col_type>(evaluable.rows(), 1, "(" + evaluable.name() + ")(:," + std::to_string(col) + ")")
        , m_expression(evaluable)
        , m_col(col)
    { }

    virtual bool isNew(size_t callerID) final{
        if (m_expression.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual const typename EvaluableT::col_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate().col(m_col);

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::col_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                         std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);
        levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::col_type>::derivative_evaluable> newDerivative;

        newDerivative = m_expression.getColumnDerivative(m_col, variable);

        return newDerivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }

};

/**
 * @brief The ColEvaluable. To retrieve a specified column from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class levi::ColEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::Evaluable<typename EvaluableT::col_type>
{
    levi::ExpressionComponent<EvaluableT> m_expression;

public:

    ColEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index col)
        : levi::Evaluable<typename EvaluableT::col_type>(expression.name())
        , m_expression(expression)
    {
        levi::unused(col);
        assert(col == 0);
    }

    virtual bool isNew(size_t callerID) final{
        if (m_expression.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual const typename EvaluableT::col_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                     std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);

        return m_expression.getColumnDerivative(0, variable);
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The ElementEvaluable. To retrieve a specified element from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class levi::ElementEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::Evaluable<typename EvaluableT::value_type>
{
    levi::ExpressionComponent<EvaluableT> m_expression;
    Eigen::Index m_row, m_col;

public:

    ElementEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index row, Eigen::Index col)
        : levi::Evaluable<typename EvaluableT::value_type>("[" + expression.name() + "](" + std::to_string(row) + ", " + std::to_string(col) + ")")
        , m_expression(expression)
        , m_row(row)
        , m_col(col)
    { }

    virtual bool isNew(size_t callerID) final{
        if (m_expression.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual const typename EvaluableT::value_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate()(m_row, m_col);

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                           std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);
        levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::value_type>::derivative_evaluable> newDerivative;

        newDerivative = levi::ExpressionComponent<levi::RowEvaluable<typename EvaluableT::derivative_evaluable>>(m_expression.getColumnDerivative(m_col, variable), m_row);

        return newDerivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The ElementEvaluable. To retrieve a specified element from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class levi::ElementEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::Evaluable<typename EvaluableT::value_type> {

    levi::ExpressionComponent<EvaluableT> m_expression;

public:

    ElementEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index row, Eigen::Index col)
        : levi::Evaluable<typename EvaluableT::value_type>(expression.name())
        , m_expression(expression)
    {
        levi::unused(row, col);
        assert(row == 0 && col == 0);
    }

    virtual bool isNew(size_t callerID) final{
        if (m_expression.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual const typename EvaluableT::value_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate();
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                     std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);

        return m_expression.getColumnDerivative(0, variable);
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The BlockEvaluable. To retrieve a specified block from an evaluable. Specialization for matrix valued evaluables.
 */
template <typename EvaluableT>
class levi::BlockEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::Evaluable<typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type>
{
    levi::ExpressionComponent<EvaluableT> m_expression;
    Eigen::Index m_startRow, m_startCol;

public:

    typedef typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type block_type;

    BlockEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index numberOfRows, Eigen::Index numberOfCols)
        : levi::Evaluable<block_type>(numberOfRows, numberOfCols, "[" + expression.name() + "](" + std::to_string(startRow) + ":" + std::to_string(startRow + numberOfRows) + ", " + std::to_string(startCol) + ":" + std::to_string(startCol + numberOfCols) + ")")
        , m_expression(expression)
        , m_startRow(startRow)
        , m_startCol(startCol)
    {
        assert(((startRow + numberOfRows) <= expression.rows()) && ((startCol + numberOfCols) <= expression.cols()));
    }

    virtual bool isNew(size_t callerID) final{
        if (m_expression.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual const block_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate().block(m_startRow, m_startCol, this->rows(), this->cols());

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<block_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename levi::Evaluable<block_type>::derivative_evaluable> newDerivative;

        newDerivative = m_expression.getColumnDerivative(m_startCol + column, variable).block(m_startRow, 0, this->rows(), variable->dimension());

        return newDerivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

/**
 * @brief The BlockEvaluable. To retrieve a specified block from an evaluable. Specialization for scalar valued evaluables.
 */
template <typename EvaluableT>
class levi::BlockEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>
        : public levi::Evaluable<typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type> {

    levi::ExpressionComponent<EvaluableT> m_expression;

public:

    typedef typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type block_type;

    BlockEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index startRow, Eigen::Index startCol, Eigen::Index numberOfRows, Eigen::Index numberOfCols)
        : levi::Evaluable<block_type>(expression.name())
        , m_expression(expression)
    {
        levi::unused(startRow, startCol, numberOfRows, numberOfCols);
        assert(startRow == 0 && startCol == 0 && numberOfRows == 1 && numberOfCols == 1);
    }

    virtual bool isNew(size_t callerID) final{
        if (m_expression.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual const block_type& evaluate() final {
        this->m_evaluationBuffer = m_expression.evaluate();
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<block_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                      std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);

        return m_expression.getColumnDerivative(0, variable);
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};


/**
 * @brief The CastEvaluable.
 *
 * It allows to assign an evaluable to an expression whose pointer cannot be directly casted. It assumes that the two evaluation buffers can be casted.
 */
template <class LeftEvaluable, class RightEvaluable>
class levi::CastEvaluable : public levi::Evaluable<typename LeftEvaluable::matrix_type> {

    levi::ExpressionComponent<RightEvaluable> m_rhs;

public:

    CastEvaluable(const levi::ExpressionComponent<RightEvaluable>& rhs)
        : levi::Evaluable<typename LeftEvaluable::matrix_type>(rhs.rows(), rhs.cols(), rhs.name())
        , m_rhs(rhs)
    { }

    virtual bool isNew(size_t callerID) final{
        if (m_rhs.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual const typename LeftEvaluable::matrix_type& evaluate() final {
        this->m_evaluationBuffer = m_rhs.evaluate();

        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename LeftEvaluable::matrix_type>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                                               std::shared_ptr<levi::VariableBase> variable) final {
        levi::ExpressionComponent<typename RightEvaluable::derivative_evaluable> rightDerivative = m_rhs.getColumnDerivative(column, variable);

        levi::ExpressionComponent<CastEvaluable<typename levi::Evaluable<typename LeftEvaluable::matrix_type>::derivative_evaluable, typename RightEvaluable::derivative_evaluable>> newCast(rightDerivative);

        return newCast;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_rhs.isDependentFrom(variable);
    }
};

#endif // LEVI_ACCESSOREVALUABLES_H
