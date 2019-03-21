/*
* Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
* Authors: Stefano Dafarra
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/
#ifndef LEVI_TREEEXPANDER_H
#define LEVI_TREEEXPANDER_H

#include <levi/HelpersForwardDeclarations.h>
#include <levi/ForwardDeclarations.h>
#include <levi/Expression.h>
#include <levi/TypeDetector.h>

namespace levi {

    template<typename EvaluableT>
    size_t expandTree(const levi::ExpressionComponent<levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, Eigen::Dynamic, Eigen::Dynamic>>>& node,
                      std::vector<levi::TreeComponent<EvaluableT>>& expandedExpression,
                      std::vector<size_t>& generics);

}

template<typename EvaluableT>
class levi::TreeComponent {

    using Type = levi::EvaluableType;

public:

    typedef Eigen::Matrix<typename EvaluableT::value_type, Eigen::Dynamic, Eigen::Dynamic> SqueezedMatrix;

    levi::ExpressionComponent<levi::Evaluable<SqueezedMatrix>> partialExpression;

    Type type;

    SqueezedMatrix buffer;

    size_t lhsIndex;
    size_t rhsIndex;

    levi::BlockType block;
    typename EvaluableT::value_type exponent;


    TreeComponent(const levi::ExpressionComponent<levi::Evaluable<SqueezedMatrix>>& expression)
        : partialExpression(expression)
          , type(expression.info().type)
          , lhsIndex(0)
          , rhsIndex(0)
    {
        buffer.resize(expression.rows(), expression.cols());

        if (type != Type::Generic) {
            block = expression.info().block;
            exponent = expression.info().exponent;
        }

        if (type == Type::Null) {
            buffer.setZero();
        }

        if (type == Type::Identity) {
            buffer.setIdentity();
        }
    }

};

template<typename EvaluableT>
size_t levi::expandTree(const levi::ExpressionComponent<levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, Eigen::Dynamic, Eigen::Dynamic>>>& node,
                        std::vector<levi::TreeComponent<EvaluableT>>& expandedExpression,
                        std::vector<size_t>& generics) {

    using Type = levi::EvaluableType;

    //Returns the position in which the node has been saved
    assert(node.isValidExpression());

    levi::EvaluableType type;

    type = node.info().type;

    expandedExpression.emplace_back(node);

    size_t currentIndex = currentIndex = expandedExpression.size() - 1;

    if (type == Type::Generic) {
        for (size_t generic : generics) {
            if (expandedExpression[generic].partialExpression == expandedExpression[currentIndex].partialExpression) {
                // This is the case where the same Generic has been added before.
                expandedExpression.pop_back();
                return generic;
            }
        }
        generics.push_back(currentIndex);
        return currentIndex;
    }

    if (type == Type::Null || type == Type::Identity) {
        return currentIndex;
    }

    if (type == Type::Sum || type == Type::Subtraction || type == Type::Product || type == Type::Division) {
        size_t lhsIndex = expandTree(node.info().lhs, expandedExpression, generics);
        expandedExpression[currentIndex].lhsIndex = lhsIndex;
        size_t rhsIndex = expandTree(node.info().rhs, expandedExpression, generics);
        expandedExpression[currentIndex].rhsIndex = rhsIndex;
        return currentIndex;
    }

    if (type == Type::InvertedSign || type == Type::Pow || type == Type::Transpose || type == Type::Row ||
        type == Type::Column || type == Type::Element || type == Type::Block) {
        size_t lhsIndex = expandTree(node.info().lhs, expandedExpression, generics);
        expandedExpression[currentIndex].lhsIndex = lhsIndex;
        return currentIndex;
    }

    assert(false && "Case not considered.");
    return expandedExpression.size();
}

#endif // LEVI_TREEEXPANDER_H
