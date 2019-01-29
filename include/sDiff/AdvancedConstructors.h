/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_ADVANCEDCONSTRUCTORS_H
#define SDIFF_ADVANCEDCONSTRUCTORS_H

#include <sDiff/ForwardDeclarations.h>
#include <sDiff/Evaluable.h>
#include <sDiff/VariableBase.h>
#include <sDiff/Expression.h>

/**
 * The ConstructorByRows.
 *
 * Constructs an evaluable by stacking the specified rows in the constructor.
 */
template <typename EvaluableT>
class sDiff::ConstructorByRows : public sDiff::Evaluable<typename EvaluableT::matrix_type> {

    std::vector<sDiff::ExpressionComponent<sDiff::Evaluable<typename EvaluableT::row_type>>> m_rows;
    std::vector<sDiff::ExpressionComponent<typename sDiff::Evaluable<typename EvaluableT::row_type>::derivative_evaluable>> m_derivatives;

public:

    ConstructorByRows(const std::vector<sDiff::ExpressionComponent<sDiff::Evaluable<typename EvaluableT::row_type>>>& rows, std::string name)
        : sDiff::Evaluable<typename EvaluableT::matrix_type>(name)
        , m_rows(rows)
    {
        assert(m_rows.size() != 0);
        assert((EvaluableT::rows_at_compile_time == Eigen::Dynamic) || (EvaluableT::rows_at_compile_time == m_rows.size()));
        Eigen::Index nCols;

        nCols = m_rows.front().cols();

        for (size_t i = 1; i < m_rows.size(); ++i) {
            assert(m_rows[i].cols() == nCols);
        }

        this->resize(m_rows.size(), nCols);

        m_derivatives.resize(m_rows.size());
    }

    virtual const typename EvaluableT::matrix_type& evaluate() final {
        for (size_t i = 0; i < m_rows.size(); ++i) {
            this->m_evaluationBuffer.row(i) = m_rows[i].evaluate();
        }
        return this->m_evaluationBuffer;
    }

    virtual sDiff::ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column, std::shared_ptr<sDiff::VariableBase> variable) {

        for (size_t i = 0; i < m_rows.size(); ++i) {
            m_derivatives[i] = m_rows[i](0, column).getColumnDerivative(0, variable); //the i-th row of the column derivative corresponds to the (only) column derivative of the element (i, column)
        }

        sDiff::ExpressionComponent<typename EvaluableT::derivative_evaluable> derivative;

        derivative = sDiff::ExpressionComponent<typename EvaluableT::derivative_evaluable>::ComposeByRows(m_derivatives, "d(" + this->name() + ")/d" + variable->variableName());
    }

    virtual bool isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable) {
        bool isDependent = false;

        for (size_t i = 0; i < m_rows.size(); ++i) {
            isDependent = isDependent || m_rows[i].isDependentFrom(variable);
        }

        return isDependent;
    }

};

#endif // SDIFF_ADVANCEDCONSTRUCTORS_H
