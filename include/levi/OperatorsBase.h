/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_OPERATORSBASE_H
#define LEVI_OPERATORSBASE_H

#include <levi/HelpersForwardDeclarations.h>
#include <levi/ForwardDeclarations.h>
#include <levi/Expression.h>
#include <levi/VariableBase.h>


template <typename MatrixType, typename EvaluableT>
class levi::UnaryOperator : public levi::Evaluable<MatrixType> {
protected:

    levi::ExpressionComponent<EvaluableT> m_expression;

    using DerivativeMap = std::unordered_map<std::string, std::vector<levi::ExpressionComponent<typename levi::Evaluable<MatrixType>::derivative_evaluable>>>;

    using DerivativeMapKey = std::pair<std::string, std::vector<levi::ExpressionComponent<typename levi::Evaluable<MatrixType>::derivative_evaluable>>>;

    DerivativeMap m_derivativeBuffer;

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

    virtual levi::ExpressionComponent<typename levi::Evaluable<MatrixType>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                         std::shared_ptr<levi::VariableBase> variable) = 0;

    virtual levi::ExpressionComponent<typename levi::Evaluable<MatrixType>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                      std::shared_ptr<levi::VariableBase> variable) final {
        typename DerivativeMap::iterator element = m_derivativeBuffer.find(variable->variableName());

        levi::ExpressionComponent<typename levi::Evaluable<MatrixType>::derivative_evaluable> emptyExpression;

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

};

template <typename MatrixType, class LeftEvaluable, class RightEvaluable>
class levi::BinaryOperator : public levi::Evaluable<MatrixType> {
protected:

    levi::ExpressionComponent<LeftEvaluable> m_lhs;
    levi::ExpressionComponent<RightEvaluable> m_rhs;

    using DerivativeMap = std::unordered_map<std::string, std::vector<levi::ExpressionComponent<typename levi::Evaluable<MatrixType>::derivative_evaluable>>>;

    using DerivativeMapKey = std::pair<std::string, std::vector<levi::ExpressionComponent<typename levi::Evaluable<MatrixType>::derivative_evaluable>>>;

    DerivativeMap m_derivativeBuffer;

public:

    BinaryOperator(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs, const std::string& name)
        : levi::Evaluable<MatrixType>(name)
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    BinaryOperator(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs, Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : levi::Evaluable<MatrixType>(rows, cols, name)
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    BinaryOperator(const levi::ExpressionComponent<LeftEvaluable>& lhs, const levi::ExpressionComponent<RightEvaluable>& rhs, const MatrixType& initialValue, const std::string& name)
        : levi::Evaluable<MatrixType>(initialValue, name)
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    virtual bool isNew(size_t callerID) final{
        if (m_lhs.isNew() || m_rhs.isNew()) {
            this->resetEvaluationRegister();
        }
        return !this->m_evaluationRegister[callerID];
    }

    virtual bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable) final{
        return m_lhs.isDependentFrom(variable) || m_rhs.isDependentFrom(variable);
    }

    virtual levi::ExpressionComponent<typename levi::Evaluable<MatrixType>::derivative_evaluable> getNewColumnDerivative(Eigen::Index column,
                                                                                                                         std::shared_ptr<levi::VariableBase> variable) = 0;

    virtual levi::ExpressionComponent<typename levi::Evaluable<MatrixType>::derivative_evaluable> getColumnDerivative(Eigen::Index column,
                                                                                                                      std::shared_ptr<levi::VariableBase> variable) final {
        typename DerivativeMap::iterator element = m_derivativeBuffer.find(variable->variableName());

        levi::ExpressionComponent<typename levi::Evaluable<MatrixType>::derivative_evaluable> emptyExpression;

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
};

#endif // LEVI_OPERATORSBASE_H
