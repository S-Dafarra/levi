/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_HELPERSFORWARDDECLARATIONS_H
#define LEVI_HELPERSFORWARDDECLARATIONS_H

#include <Eigen/Core>

#include <vector>
#include <type_traits>
#include <memory>
#include <string>
#include <cassert>
#include <cmath>

namespace levi {

    template<bool T>
    struct bool_value { };

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
            typename std::enable_if<is_valid_product<lhsRows, lhsCols, rhsRows, rhsCols>::value &&
            !(lhsRows == 1 && lhsCols == 1) && !(rhsRows == 1 && rhsCols == 1)>::type>;

    template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
    struct matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
            typename std::enable_if<lhsRows == 1 && lhsCols == 1>::type>;

    template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
    struct matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
            typename std::enable_if<rhsRows == 1 && rhsCols == 1>::type>;

    template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols>
    struct matrix_product_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
            typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols>
    struct matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Scalar,
            typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    template<typename Scalar_lhs, typename Scalar_rhs>
    struct matrix_product_return<Scalar_lhs, Scalar_rhs,
            typename std::enable_if<std::is_arithmetic<Scalar_lhs>::value && std::is_arithmetic<Scalar_rhs>::value>::type>;

    template<typename Matrix, class Enabler = void>
    struct dynamic_block_return;

    template<typename Matrix>
    struct dynamic_block_return<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type>;

    template<typename Scalar>
    struct dynamic_block_return<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    template<typename EvaluableT, class Enabler = void>
    struct transpose_type;

    template<typename EvaluableT>
    struct transpose_type<EvaluableT, typename std::enable_if<!std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>;

    template<typename EvaluableT>
    struct transpose_type<EvaluableT, typename std::enable_if<std::is_arithmetic<typename EvaluableT::matrix_type>::value>::type>;

}

#endif // LEVI_HELPERSFORWARDDECLARATIONS_H