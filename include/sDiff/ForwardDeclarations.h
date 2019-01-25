/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_FORWARDDECLARATIONS_H
#define SDIFF_FORWARDDECLARATIONS_H

#include <Eigen/Core>

#include <type_traits>
#include <memory>
#include <string>
#include <cassert>

namespace sDiff {

    template<bool T>
    struct bool_value { };

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

    template <typename Scalar_lhs, typename Scalar_rhs>
    struct scalar_sum_return;

    template<int lhsRows, int lhsCols, int rhsRows, int rhsCols, class Enabler = void>
    struct is_valid_sum : std::false_type {};

    template<int lhsRows, int lhsCols, int rhsRows, int rhsCols>
    struct is_valid_sum<lhsRows, lhsCols, rhsRows, rhsCols,
            typename std::enable_if<((lhsRows == rhsRows) || (lhsRows == Eigen::Dynamic) || (rhsRows == Eigen::Dynamic)) &&
                                    ((lhsCols == rhsCols) || (lhsCols == Eigen::Dynamic) || (rhsCols == Eigen::Dynamic))>::type> : std::true_type {};

    template <typename Matrix_lhs, typename Matrix_rhs, class Enabler = void>
    struct matrix_sum_return;

    template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
    struct matrix_sum_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
            typename std::enable_if<is_valid_sum<lhsRows, lhsCols, rhsRows, rhsCols>::value>::type>;

    template<typename Scalar_lhs, typename Scalar_rhs>
    struct matrix_sum_return<Scalar_lhs, Scalar_rhs,
            typename std::enable_if<std::is_arithmetic<Scalar_lhs>::value && std::is_arithmetic<Scalar_rhs>::value>::type>;

    template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols>
    struct matrix_sum_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
            typename std::enable_if<std::is_arithmetic<Scalar>::value && is_valid_sum<1,1, rhsRows, rhsCols>::value>::type>;

    template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols>
    struct matrix_sum_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Scalar,
            typename std::enable_if<std::is_arithmetic<Scalar>::value && is_valid_sum<1,1, lhsRows, lhsCols>::value>::type>;

    template<int lhsRows, int lhsCols, int rhsRows, int rhsCols, class Enabler = void>
    struct is_valid_product : std::false_type {};

    template<int lhsRows, int lhsCols, int rhsRows, int rhsCols>
    struct is_valid_product<lhsRows, lhsCols, rhsRows, rhsCols,
            typename std::enable_if<lhsCols == Eigen::Dynamic || rhsRows == Eigen::Dynamic || lhsCols == rhsRows>::type> : std::true_type {};

    template <typename Scalar_lhs, typename Scalar_rhs>
    struct scalar_product_return;

    template <typename Matrix_lhs, typename Matrix_rhs, class Enabler = void>
    struct matrix_product_return;

    template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
    struct matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
            typename std::enable_if<is_valid_product<lhsRows, lhsCols, rhsRows, rhsCols>::value>::type>;

    template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols>
    struct matrix_product_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
            typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols>
    struct matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Scalar,
            typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    template<typename Scalar_lhs, typename Scalar_rhs>
    struct matrix_product_return<Scalar_lhs, Scalar_rhs,
            typename std::enable_if<std::is_arithmetic<Scalar_lhs>::value && std::is_arithmetic<Scalar_rhs>::value>::type>;

    template <class LeftEvaluable, class RightEvaluable>
    class SumEvaluable;

    template <class LeftEvaluable, class RightEvaluable>
    class SubtractionEvaluable;

    template <class LeftEvaluable, class RightEvaluable>
    class ProductEvaluable;

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

    template <class LeftEvaluable, class RightEvaluable>
    class CastEvaluable;

    typedef ExpressionComponent<Evaluable<Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic >>> Expression;

    typedef ExpressionComponent<Evaluable<Eigen::Matrix< double , Eigen::Dynamic , 1 >>> ColumnExpression;

    typedef ExpressionComponent<EvaluableVariable<Eigen::Matrix< double , Eigen::Dynamic , 1>>> Variable;

    typedef ExpressionComponent<ConstantEvaluable<Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic>>> Constant;

    typedef ExpressionComponent<ConstantEvaluable<double>> Scalar;
}

#endif // SDIFF_FORWARDDECLARATIONS_H
