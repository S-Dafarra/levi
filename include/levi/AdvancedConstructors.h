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
#include <levi/VariableBase.h>
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

    using DerivativeMap = std::unordered_map<std::string, std::vector<levi::ExpressionComponent<typename EvaluableT::derivative_evaluable>>>;

    using DerivativeMapKey = std::pair<std::string, std::vector<levi::ExpressionComponent<typename EvaluableT::derivative_evaluable>>>;

    DerivativeMap m_derivativeBuffer;

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

    virtual const typename EvaluableT::matrix_type& evaluate() final {
        for (size_t i = 0; i < m_rows.size(); ++i) {
            this->m_evaluationBuffer.row(i) = m_rows[i].evaluate();
        }
        return this->m_evaluationBuffer;
    }

    levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getNewColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {

        for (size_t i = 0; i < m_rows.size(); ++i) {
            m_derivatives[i] = m_rows[i](0, column).getColumnDerivative(0, variable); //the i-th row of the column derivative corresponds to the (only) column derivative of the element (i, column)
        }

        levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> derivative;

        derivative = levi::ExpressionComponent<typename EvaluableT::derivative_evaluable>::ComposeByRows(m_derivatives, "d(" + this->name() + ")/d" + variable->variableName());

        return derivative;
    }

    virtual levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                     std::shared_ptr<levi::VariableBase> variable) final {
        typename DerivativeMap::iterator element = m_derivativeBuffer.find(variable->variableName());

        levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> emptyExpression;

        if (element == m_derivativeBuffer.end()) {
            DerivativeMapKey newPair;
            newPair.first = variable->variableName();
            newPair.second.resize(this->cols(), emptyExpression);
            auto insertedElement = m_derivativeBuffer.insert(newPair);
            element = insertedElement.first;
        }

        if (!(element->second.at(column).isValidExpression())) {
            element->second.at(column) = getNewColumnDerivative(column, variable);
        }

        return element->second.at(column);
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

    using DerivativeMap = std::unordered_map<std::string, std::vector<levi::ExpressionComponent<typename EvaluableT::derivative_evaluable>>>;

    using DerivativeMapKey = std::pair<std::string, std::vector<levi::ExpressionComponent<typename EvaluableT::derivative_evaluable>>>;

    DerivativeMap m_derivativeBuffer;

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

    levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getNewColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) {

        levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> derivative;

        derivative = m_cols[column].getColumnDerivative(0, variable);

        return derivative;
    }

    virtual levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                     std::shared_ptr<levi::VariableBase> variable) final {
        typename DerivativeMap::iterator element = m_derivativeBuffer.find(variable->variableName());

        levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> emptyExpression;

        if (element == m_derivativeBuffer.end()) {
            DerivativeMapKey newPair;
            newPair.first = variable->variableName();
            newPair.second.resize(this->cols(), emptyExpression);
            auto insertedElement = m_derivativeBuffer.insert(newPair);
            element = insertedElement.first;
        }

        if (!(element->second.at(column).isValidExpression())) {
            element->second.at(column) = getNewColumnDerivative(column, variable);
        }

        return element->second.at(column);
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) {
        bool isDependent = false;

        for (size_t i = 0; i < m_cols.size(); ++i) {
            isDependent = isDependent || m_cols[i].isDependentFrom(variable);
        }

        return isDependent;
    }

};

#endif // LEVI_ADVANCEDCONSTRUCTORS_H
