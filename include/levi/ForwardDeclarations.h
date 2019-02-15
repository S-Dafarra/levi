/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_FORWARDDECLARATIONS_H
#define LEVI_FORWARDDECLARATIONS_H

#ifndef LEVI_DEFAULT_MATRIX_TYPE
#define LEVI_DEFAULT_MATRIX_TYPE levi::INVALID_TYPE
#endif

#ifndef LEVI_DEFAULT_VECTOR_TYPE
#define LEVI_DEFAULT_VECTOR_TYPE levi::INVALID_TYPE
#endif

#include <type_traits>


namespace levi {

    class INVALID_TYPE;

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

    template <typename MatrixType, class LeftEvaluable, class RightEvaluable>
    class BinaryOperator;

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

    template <typename EvaluableT>
    class VariableFromExpressionEvaluable;

    template <typename CompositeEvaluable, typename leftEvaluable, typename rightEvaluable>
    class HorzcatEvaluable;

    template <typename CompositeEvaluable, typename TopEvaluable, typename BottomEvaluable>
    class VertcatEvaluable;

    typedef Evaluable<LEVI_DEFAULT_MATRIX_TYPE> DefaultEvaluable;

    typedef ExpressionComponent<DefaultEvaluable> Expression;

    typedef ExpressionComponent<Evaluable<double>> ScalarExpression;

    typedef ExpressionComponent<Evaluable<LEVI_DEFAULT_VECTOR_TYPE>> ColumnExpression;

    typedef EvaluableVariable<LEVI_DEFAULT_VECTOR_TYPE> DefaultVariableEvaluable;

    typedef ExpressionComponent<DefaultVariableEvaluable> Variable;

    typedef ExpressionComponent<EvaluableVariable<double>> ScalarVariable;

    typedef ExpressionComponent<ConstantEvaluable<LEVI_DEFAULT_MATRIX_TYPE>> Constant;

    typedef ExpressionComponent<IdentityEvaluable<LEVI_DEFAULT_MATRIX_TYPE>> Identity;

    typedef ExpressionComponent<NullEvaluable<LEVI_DEFAULT_MATRIX_TYPE>> Null;

    typedef ExpressionComponent<ConstantEvaluable<double>> Scalar;
}

#endif // LEVI_FORWARDDECLARATIONS_H
