/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_FORWARDDECLARATIONS_H
#define LEVI_FORWARDDECLARATIONS_H

#include <levi/HelpersForwardDeclarations.h>

namespace levi {

    template<typename Matrix, class Enabler = void>
    class Evaluable;

    template <typename Matrix>
    class Evaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type>;

    template <typename Scalar>
    class Evaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    class VariableBase;

    template <typename Vector, class Enabler = void>
    class EvaluableVariable;

    template <typename Vector>
    class EvaluableVariable<Vector, typename std::enable_if<!std::is_arithmetic<Vector>::value>::type>;

    template <typename Scalar>
    class EvaluableVariable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    template <class Evaluable>
    class ExpressionComponent;

    template <typename Matrix, class Enabler = void>
    class ConstantEvaluable;

    template <typename Matrix>
    class ConstantEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type>;

    template <typename Scalar>
    class ConstantEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    template <typename Matrix, class Enabler = void>
    class NullEvaluable;

    template <typename Matrix>
    class NullEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type>;

    template <typename Scalar>
    class NullEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    template <typename Matrix, class Enabler = void>
    class IdentityEvaluable;

    template <typename Matrix>
    class IdentityEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type>;

    template <typename Scalar>
    class IdentityEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    template <typename MatrixType, typename EvaluableT>
    class UnaryOperator;

    template <class LeftEvaluable, class RightEvaluable>
    class SumEvaluable;

    template <class LeftEvaluable, class RightEvaluable>
    class SubtractionEvaluable;

    template <class EvaluableT>
    class SignInvertedEvaluable;

    template <class LeftEvaluable, class RightEvaluable>
    class ProductEvaluable;

    template <class EvaluableT>
    class PowEvaluable;

    template <class LeftEvaluable, class RightEvaluable>
    class DivisionEvaluable;

    template<class Evaluable, class Enabler = void>
    class RowEvaluable;

    template <typename EvaluableT>
    class RowEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>;

    template <typename EvaluableT>
    class RowEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>;

    template<class Evaluable, class Enabler = void>
    class ColEvaluable;

    template <typename EvaluableT>
    class ColEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>;

    template <typename EvaluableT>
    class ColEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>;

    template<class EvaluableT, class Enabler = void>
    class ElementEvaluable;

    template <typename EvaluableT>
    class ElementEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>;

    template <typename EvaluableT>
    class ElementEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>;

    template<class EvaluableT, class Enabler = void>
    class BlockEvaluable;

    template <typename EvaluableT>
    class BlockEvaluable<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>;

    template <typename EvaluableT>
    class BlockEvaluable<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>;

    template <class LeftEvaluable, class RightEvaluable>
    class CastEvaluable;

    template <typename EvaluableT>
    class SkewEvaluable;

    template <typename EvaluableT>
    class TransposeEvaluable;

    template <typename EvaluableT>
    class ConstructorByRows;

    template <typename EvaluableT>
    class ConstructorByCols;

    typedef ExpressionComponent<Evaluable<Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic >>> Expression;

    typedef ExpressionComponent<Evaluable<double>> ScalarExpression;

    typedef ExpressionComponent<Evaluable<Eigen::Matrix< double , Eigen::Dynamic , 1 >>> ColumnExpression;

    typedef ExpressionComponent<EvaluableVariable<Eigen::Matrix< double , Eigen::Dynamic , 1>>> Variable;

    typedef ExpressionComponent<ConstantEvaluable<Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic>>> Constant;

    typedef ExpressionComponent<IdentityEvaluable<Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic>>> Identity;

    typedef ExpressionComponent<ConstantEvaluable<double>> Scalar;
}

#endif // LEVI_FORWARDDECLARATIONS_H
