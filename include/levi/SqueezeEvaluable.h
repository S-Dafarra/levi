/*
* Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
* Authors: Stefano Dafarra
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/
#ifndef LEVI_SQUEEZEEVALUABLE_H
#define LEVI_SQUEEZEEVALUABLE_H

#include <levi/HelpersForwardDeclarations.h>
#include <levi/ForwardDeclarations.h>
#include <levi/Expression.h>
#include <levi/TypeDetector.h>
#include <levi/TreeExpander.h>

template<typename EvaluableT>
class levi::SqueezeEvaluable
    : public levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, Eigen::Dynamic, Eigen::Dynamic>> {

public:

    using SqueezedMatrix = typename levi::TreeExpander<EvaluableT>::SqueezedMatrix;

private:

    using Type = levi::EvaluableType;

    levi::ExpressionComponent<EvaluableT> m_fullExpression;
    levi::TreeExpander<EvaluableT> m_tree;

public:

    SqueezeEvaluable(const levi::ExpressionComponent<EvaluableT>& fullExpression, const std::string& name)
        : levi::Evaluable<SqueezedMatrix> (fullExpression.rows(), fullExpression.cols(), name)
          , m_fullExpression(fullExpression)
          , m_tree(fullExpression)
    { }

    ~SqueezeEvaluable();

    virtual const SqueezedMatrix& evaluate() final {

        for (size_t generic : m_tree.generics) {
            m_tree.expandedExpression[generic].buffer = m_tree.expandedExpression[generic].partialExpression.evaluate(false); //first evaluate generics
        }

        levi::EvaluableType type;

        for(typename std::vector<levi::TreeComponent<EvaluableT>>::reverse_iterator i = m_tree.expandedExpression.rbegin();
             i != m_tree.expandedExpression.rend(); ++i) {
            type = i->type;

            if (type == Type::Sum) {
                i->buffer.lazyAssign(m_tree.expandedExpression[i->lhsIndex].buffer + m_tree.expandedExpression[i->rhsIndex].buffer); //this is why I need to evaluate the expanded expression in reverse order
            } else if (type == Type::Subtraction) {
                i->buffer.lazyAssign(m_tree.expandedExpression[i->lhsIndex].buffer - m_tree.expandedExpression[i->rhsIndex].buffer);
            } else if (type == Type::Product) {

                if (m_tree.expandedExpression[i->lhsIndex].buffer.cols() != m_tree.expandedExpression[i->rhsIndex].buffer.rows()) {
                    if (m_tree.expandedExpression[i->lhsIndex].buffer.rows() == 1 && m_tree.expandedExpression[i->lhsIndex].buffer.cols() == 1) {
                        i->buffer.lazyAssign(m_tree.expandedExpression[i->lhsIndex].buffer(0,0) * m_tree.expandedExpression[i->rhsIndex].buffer);
                    } else {
                        i->buffer.lazyAssign(m_tree.expandedExpression[i->lhsIndex].buffer * m_tree.expandedExpression[i->rhsIndex].buffer(0,0));
                    }
                } else {
                    i->buffer.lazyAssign(m_tree.expandedExpression[i->lhsIndex].buffer * m_tree.expandedExpression[i->rhsIndex].buffer);
                }

            } else if (type == Type::Division) {
                i->buffer.lazyAssign(m_tree.expandedExpression[i->lhsIndex].buffer / m_tree.expandedExpression[i->rhsIndex].buffer(0,0));
            } else if (type == Type::InvertedSign) {
                i->buffer.lazyAssign(-m_tree.expandedExpression[i->lhsIndex].buffer);
            } else if (type == Type::Pow) {
                i->buffer(0,0) = std::pow(m_tree.expandedExpression[i->lhsIndex].buffer(0,0), i->exponent);
            } else if (type == Type::Transpose) {
                i->buffer.lazyAssign(m_tree.expandedExpression[i->lhsIndex].buffer.transpose());
            } else if (type == Type::Row) {
                i->buffer.lazyAssign(m_tree.expandedExpression[i->lhsIndex].buffer.row(i->block.startRow));
            } else if (type == Type::Column) {
                i->buffer.lazyAssign(m_tree.expandedExpression[i->lhsIndex].buffer.col(i->block.startCol));
            } else if (type == Type::Element) {
                i->buffer(0,0) = m_tree.expandedExpression[i->lhsIndex].buffer(i->block.startRow, i->block.startCol);
            } else if (type == Type::Block) {
                i->buffer.lazyAssign(m_tree.expandedExpression[i->lhsIndex].buffer.block(i->block.startRow, i->block.startCol, i->block.rows, i->block.cols));
            }
        }

        this->m_evaluationBuffer = m_tree.expandedExpression[0].buffer;

        return this->m_evaluationBuffer;
    }

    virtual bool isNew(size_t callerID) final {

        bool newVal = false;
        bool isNew;

        for (size_t i = 0; i < m_tree.generics.size(); ++i) {
            isNew = m_tree.expandedExpression[m_tree.generics[i]].partialExpression.isNew();
            newVal = isNew || newVal;
        }

        if (newVal) {
            this->resetEvaluationRegister();
        }

        return !this->m_evaluationRegister[callerID];
    }

};

template<typename EvaluableT>
levi::SqueezeEvaluable<EvaluableT>::~SqueezeEvaluable() {}

#endif // LEVI_SQUEEZEEVALUABLE_H
