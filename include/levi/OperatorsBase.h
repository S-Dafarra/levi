/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_OPERATORSBASE_H
#define LEVI_OPERATORSBASE_H

#include <levi/ForwardDeclarations.h>
#include <levi/Expression.h>

template <typename MatrixType, typename EvaluableT>
class levi::UnaryOperator : public levi::Evaluable<MatrixType> {
protected:

    levi::ExpressionComponent<EvaluableT> m_expression;

public:

    UnaryOperator(const levi::ExpressionComponent<EvaluableT>& expression, const std::string& name)
        : levi::Evaluable<MatrixType>(name)
        , m_expression(expression)
    { }

    UnaryOperator(const levi::ExpressionComponent<EvaluableT>& expression, Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : levi::Evaluable<MatrixType>(rows, cols, name)
        , m_expression(expression)
    { }

    UnaryOperator(const levi::ExpressionComponent<EvaluableT>& expression, const MatrixType& initialValue, const std::string& name)
        : levi::Evaluable<MatrixType>(initialValue, name)
        , m_expression(expression)
    { }

    virtual bool isNew(size_t callerID) final{
        if (m_expression.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_expression.isDependentFrom(variable);
    }
};

#endif // LEVI_OPERATORSBASE_H
