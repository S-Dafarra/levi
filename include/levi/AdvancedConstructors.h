/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_ADVANCEDCONSTRUCTORS_H
#define LEVI_ADVANCEDCONSTRUCTORS_H

#include <levi/HelpersForwardDeclarations.h>
#include <levi/ForwardDeclarations.h>
#include <levi/Evaluable.h>
#include <levi/Variable.h>
#include <levi/OperatorsBase.h>
#include <levi/Expression.h>

/**
 * The ConstructorByRows.
 *
 * Constructs an evaluable by stacking the specified rows in the constructor.
 */
template <typename EvaluableT>
class levi::ConstructorByRows : public levi::Evaluable<typename EvaluableT::matrix_type> {

    std::vector<levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::row_type>>> m_rows;
    std::vector<levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::row_type>::derivative_evaluable>> m_derivatives;

public:

    ConstructorByRows(const std::vector<levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::row_type>>>& rows, std::string name)
        : levi::Evaluable<typename EvaluableT::matrix_type>(name)
    {
        assert(rows.size() != 0);
        assert((EvaluableT::rows_at_compile_time == Eigen::Dynamic) || (EvaluableT::rows_at_compile_time == rows.size()));
        Eigen::Index nCols;

        m_rows.push_back(rows.front());
        nCols = m_rows.front().cols();

        for (size_t i = 1; i < rows.size(); ++i) {
            m_rows.push_back(rows[i]);
            assert(m_rows[i].cols() == nCols);
        }

        this->resize(m_rows.size(), nCols);

        m_derivatives.resize(m_rows.size());
    }

    virtual bool isNew(size_t callerID) final{
        bool newVal = false;

        for (size_t i = 0; i < m_rows.size(); ++i) {
            newVal = newVal || m_rows[i].isNew();
        }

        if (newVal) {
            this->resetEvaluationRegister();
        }

        return !this->m_evaluationRegister[callerID];
    }

    virtual levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::row_type>> row(Eigen::Index row) final {
        return m_rows[row];
    }

    virtual levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::value_type>> element(Eigen::Index row, Eigen::Index col) final {
        return m_rows[row](0, col);
    }

    virtual const typename EvaluableT::matrix_type& evaluate() final {
        for (size_t i = 0; i < m_rows.size(); ++i) {
            this->m_evaluationBuffer.row(i) = m_rows[i].evaluate();
        }
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getNewColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) final {

        for (size_t i = 0; i < m_rows.size(); ++i) {
            m_derivatives[i] = m_rows[i](0, column).getColumnDerivative(0, variable); //the i-th row of the column derivative corresponds to the (only) column derivative of the element (i, column)
        }

        levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> derivative;

        derivative = levi::ExpressionComponent<typename EvaluableT::derivative_evaluable>::ComposeByRows(m_derivatives, "d(" + this->name() + ")/d" + variable->variableName());

        return derivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) {
        bool isDependent = false;

        for (size_t i = 0; i < m_rows.size(); ++i) {
            isDependent = isDependent || m_rows[i].isDependentFrom(variable);
        }

        return isDependent;
    }

};

/**
 * The ConstructorByCols.
 *
 * Constructs an evaluable by aligning the columns specified in the constructor.
 */
template <typename EvaluableT>
class levi::ConstructorByCols : public levi::Evaluable<typename EvaluableT::matrix_type> {

    std::vector<levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::col_type>>> m_cols;

public:

    ConstructorByCols(const std::vector<levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::col_type>>>& cols, std::string name)
        : levi::Evaluable<typename EvaluableT::matrix_type>(name)
    {
        assert(cols.size() != 0);
        assert((EvaluableT::cols_at_compile_time == Eigen::Dynamic) || (EvaluableT::cols_at_compile_time == cols.size()));
        Eigen::Index nRows;

        m_cols.push_back(cols.front());
        nRows = m_cols.front().rows();

        for (size_t i = 1; i < cols.size(); ++i) {
            m_cols.push_back(cols[i]);
            assert(m_cols[i].rows() == nRows);
        }

        this->resize(nRows, m_cols.size());
    }

    virtual levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::col_type>> col(Eigen::Index col) final {
        return m_cols[col];
    }

    virtual levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::value_type>> element(Eigen::Index row, Eigen::Index col) final {
        return m_cols[col](row, 0);
    }

    virtual bool isNew(size_t callerID) final{
        bool newVal = false;

        for (size_t i = 0; i < m_cols.size(); ++i) {
            newVal = newVal || m_cols[i].isNew();
        }

        if (newVal) {
            this->resetEvaluationRegister();
        }

        return !this->m_evaluationRegister[callerID];
    }

    virtual const typename EvaluableT::matrix_type& evaluate() final {
        for (size_t i = 0; i < m_cols.size(); ++i) {
            this->m_evaluationBuffer.col(i) = m_cols[i].evaluate();
        }
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getNewColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) final {

        levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> derivative;

        derivative = m_cols[column].getColumnDerivative(0, variable);

        return derivative;
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) {
        bool isDependent = false;

        for (size_t i = 0; i < m_cols.size(); ++i) {
            isDependent = isDependent || m_cols[i].isDependentFrom(variable);
        }

        return isDependent;
    }

};

template <typename EvaluableT>
class levi::VariableFromExpressionEvaluable : public levi::EvaluableVariable<typename EvaluableT::col_type> {
    levi::ExpressionComponent<EvaluableT> m_expression;

public:

    VariableFromExpressionEvaluable(const levi::ExpressionComponent<EvaluableT>& expression, int)
        : levi::EvaluableVariable<typename EvaluableT::col_type>(expression.rows(), expression.name())
          , m_expression(expression)
    {
        static_assert (EvaluableT::cols_at_compile_time == 1 || EvaluableT::cols_at_compile_time == Eigen::Dynamic, "Can't obtain a variable from a matrix evaluable" );
        assert(expression.cols() == 1 && "Can't obtain a variable from a matrix evaluable");

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

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension())) || m_expression.isDependentFrom(variable);
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<typename EvaluableT::col_type>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                                            std::shared_ptr<levi::VariableBase> variable) final {
        levi::unused(column);
        assert(column == 0);
        if ((this->variableName() == variable->variableName()) && (this->dimension() == variable->dimension())) {
            return this->m_identityDerivative;
        } else if (m_expression.isDependentFrom(variable)) {
            return m_expression.getColumnDerivative(column, variable);
        } else {
            return levi::ExpressionComponent<levi::NullEvaluable<typename levi::Evaluable<typename EvaluableT::col_type>::derivative_evaluable::matrix_type>>(this->dimension(), variable->dimension(),
                                                                                                                                                              "d " + this->variableName() + "/(d " + variable->variableName() + ")");
        }
    }

};

template <typename CompositeEvaluable, typename LeftEvaluable, typename RightEvaluable>
class levi::HorzcatEvaluable : public levi::BinaryOperator<typename CompositeEvaluable::matrix_type, LeftEvaluable, RightEvaluable>
{
public:

    HorzcatEvaluable(const levi::ExpressionComponent<LeftEvaluable>& lhs,
                     const levi::ExpressionComponent<RightEvaluable>& rhs, const std::string& name)
        : levi::BinaryOperator<typename CompositeEvaluable::matrix_type, LeftEvaluable, RightEvaluable>(lhs, rhs, lhs.rows(),
                                                                                                        lhs.cols() + rhs.cols(), name)
    { }

    virtual levi::ExpressionComponent<levi::Evaluable<typename CompositeEvaluable::row_type>> row(Eigen::Index row) final {
        return levi::ExpressionComponent<levi::Evaluable<typename CompositeEvaluable::row_type>>::Horzcat(this->m_lhs.row(row),
                                                                                                          this->m_rhs.row(row),
                                                                                                          "[" + this->name() + "](" +
                                                                                                              std::to_string(row) + ",:)");
    }

    virtual levi::ExpressionComponent<levi::Evaluable<typename CompositeEvaluable::col_type>> col(Eigen::Index col) final {
        if (col < this->m_lhs.cols()) {
            return this->m_lhs.col(col);
        } else {
            return this->m_rhs.col(col - this->m_lhs.cols());
        }
    }

    virtual levi::ExpressionComponent<levi::Evaluable<typename CompositeEvaluable::value_type>> element(Eigen::Index row, Eigen::Index col) final {
        if (col < this->m_lhs.cols()) {
            return this->m_lhs(row, col);
        } else {
            return this->m_rhs(row, col - this->m_lhs.cols());
        }
    }

    virtual const typename CompositeEvaluable::matrix_type& evaluate() final {
        this->m_evaluationBuffer.leftCols(this->m_lhs.cols()) = this->m_lhs.evaluate();
        this->m_evaluationBuffer.rightCols(this->m_rhs.cols()) = this->m_rhs.evaluate();
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename CompositeEvaluable::derivative_evaluable>
    getNewColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {
        if (column < this->m_lhs.cols()) {
            return this->m_lhs.getColumnDerivative(column, variable);
        } else {
            return this->m_rhs.getColumnDerivative(column - this->m_lhs.cols(), variable);
        }
    }
};

template <typename CompositeEvaluable, typename TopEvaluable, typename BottomEvaluable>
class levi::VertcatEvaluable : public levi::BinaryOperator<typename CompositeEvaluable::matrix_type, TopEvaluable, BottomEvaluable>
{
public:

    VertcatEvaluable(const levi::ExpressionComponent<TopEvaluable>& top,
                     const levi::ExpressionComponent<BottomEvaluable>& bottom, const std::string& name)
        : levi::BinaryOperator<typename CompositeEvaluable::matrix_type, TopEvaluable, BottomEvaluable>(top, bottom, top.rows() + bottom.rows(),
                                                                                                        top.cols(), name)
    { }

    virtual levi::ExpressionComponent<levi::Evaluable<typename CompositeEvaluable::row_type>> row(Eigen::Index row) final {

        if (row < this->m_lhs.rows()) {
            return this->m_lhs.row(row);
        } else {
            return this->m_rhs.row(row - this->m_lhs.rows());
        }
    }

    virtual levi::ExpressionComponent<levi::Evaluable<typename CompositeEvaluable::col_type>> col(Eigen::Index col) final {
        return levi::ExpressionComponent<levi::Evaluable<typename CompositeEvaluable::col_type>>::Vertcat(this->m_lhs.col(col),
                                                                                                          this->m_rhs.col(col),
                                                                                                          "[" + this->name() + "](:," +
                                                                                                              std::to_string(col) + ")");
    }

    virtual levi::ExpressionComponent<levi::Evaluable<typename CompositeEvaluable::value_type>> element(Eigen::Index row, Eigen::Index col) final {
        if (row < this->m_lhs.rows()) {
            return this->m_lhs(row, col);
        } else {
            return this->m_rhs(row - this->m_lhs.rows(), col);
        }
    }

    virtual const typename CompositeEvaluable::matrix_type& evaluate() final {
        this->m_evaluationBuffer.topRows(this->m_lhs.rows()) = this->m_lhs.evaluate();
        this->m_evaluationBuffer.bottomRows(this->m_rhs.rows()) = this->m_rhs.evaluate();
        return this->m_evaluationBuffer;
    }

    virtual levi::ExpressionComponent<typename CompositeEvaluable::derivative_evaluable>
    getNewColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {
        return levi::ExpressionComponent<
            typename CompositeEvaluable::derivative_evaluable>::Vertcat(this->m_lhs.getColumnDerivative(column, variable),
                                                                        this->m_rhs.getColumnDerivative(column, variable),
                                                                        "d[" + this->name() + "(:," + std::to_string(column) + ")]/d"
                                                                            + variable->variableName());
    }
};

#endif // LEVI_ADVANCEDCONSTRUCTORS_H
