/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_OPERATORS_EVALUABLES_H
#define SDIFF_OPERATORS_EVALUABLES_H

#include <sDiff/Evaluable.h>
#include <type_traits>
#include <Eigen/Core>
#include <memory>

template<int lhsRows, int lhsCols, int rhsRows, int rhsCols, class Enabler = void>
struct is_valid_product : std::false_type {};

template<int lhsRows, int lhsCols, int rhsRows, int rhsCols>
struct is_valid_product<lhsRows, lhsCols, rhsRows, rhsCols,
        typename std::enable_if<lhsCols == Eigen::Dynamic || rhsRows == Eigen::Dynamic || lhsCols == rhsRows>::type> : std::true_type {};

template <typename Scalar_lhs, typename Scalar_rhs>
struct scalar_product_return {
    typedef decltype (std::declval<Scalar_lhs>() * std::declval<Scalar_rhs>()) type;
};

template <typename Matrix_lhs, typename Matrix_rhs, class Enabler = void>
struct matrix_product_return;

template<typename Scalar_lhs, int lhsRows, int lhsCols, typename Scalar_rhs, int rhsRows, int rhsCols>
struct matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<is_valid_product<lhsRows, lhsCols, rhsRows, rhsCols>::value>::type> {
    typedef Eigen::Matrix<typename scalar_product_return<Scalar_lhs, Scalar_rhs>::type, lhsRows, rhsCols> type;
};

template<typename Scalar, typename Scalar_rhs, int rhsRows, int rhsCols>
struct matrix_product_return<Scalar, Eigen::Matrix<Scalar_rhs, rhsRows, rhsCols>,
        typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Eigen::Matrix<typename scalar_product_return<Scalar, Scalar_rhs>::type, rhsRows, rhsCols> type;
};

template<typename Scalar, typename Scalar_lhs, int lhsRows, int lhsCols>
struct matrix_product_return<Eigen::Matrix<Scalar_lhs, lhsRows, lhsCols>, Scalar,
        typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {
    typedef Eigen::Matrix<typename scalar_product_return<Scalar, Scalar_lhs>::type, lhsRows, lhsCols> type;
};


namespace sDiff {

    template <typename Matrix, class Enabler = void>
class ConstantEvaluable { };

    template <typename Matrix>
    class ConstantEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type>;

    template <typename Scalar>
    class ConstantEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;

    template <class LeftEvaluable, class RightEvaluable, typename Matrix = Eigen::MatrixXd>
    class SumEvaluable;

    template <class LeftEvaluable, class RightEvaluable>
    class ProductEvaluable;
}

template <typename Matrix>
class sDiff::ConstantEvaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> : public sDiff::Evaluable<Matrix>{
public:

    ConstantEvaluable(const Matrix& constant, std::string name)
        : Evaluable<Matrix>(constant, name)
    { }

    ConstantEvaluable(Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : Evaluable<Matrix>(rows, cols, name)
    { }

    virtual const Matrix& evaluate() final {
        return this->m_evaluationBuffer;
    }

    void operator=(const Matrix& rhs) {
        this->m_evaluationBuffer = rhs;
    }
};

template <typename Scalar>
class sDiff::ConstantEvaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> : public sDiff::Evaluable<Scalar>{
public:

    ConstantEvaluable(const Scalar& constant)
        : Evaluable<Scalar>(constant)
    { }

    virtual const Scalar& evaluate() final {
        return this->m_evaluationBuffer;
    }

    void operator=(const Scalar& rhs) {
        this->m_evaluationBuffer = rhs;
    }
};

template <class LeftEvaluable, class RightEvaluable, typename Matrix>
class sDiff::SumEvaluable : public sDiff::Evaluable<Matrix>{

    std::shared_ptr<LeftEvaluable> m_lhs;
    std::shared_ptr<RightEvaluable> m_rhs;

public:

    SumEvaluable(std::shared_ptr<LeftEvaluable> lhs, std::shared_ptr<RightEvaluable> rhs)
        : Evaluable<Matrix>(lhs->rows(), lhs->cols(), lhs->name() + " + " + rhs->name())
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    virtual const Matrix& evaluate() final {
        this->m_evaluationBuffer = m_lhs->evaluate() + m_rhs->evaluate();

        return this->m_evaluationBuffer;
    }

};

template <class LeftEvaluable, class RightEvaluable>
class sDiff::ProductEvaluable : public sDiff::Evaluable<typename matrix_product_return<
        typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type>{

    std::shared_ptr<LeftEvaluable> m_lhs;
    std::shared_ptr<RightEvaluable> m_rhs;

public:

    typedef typename matrix_product_return<typename LeftEvaluable::matrix_type, typename RightEvaluable::matrix_type>::type product_type;

    ProductEvaluable(std::shared_ptr<LeftEvaluable> lhs, std::shared_ptr<RightEvaluable> rhs)
        : Evaluable<product_type>(lhs->rows(), rhs->cols(), "(" + lhs->name() + ")" + " * " + "(" + rhs->name() + ")")
        , m_lhs(lhs)
        , m_rhs(rhs)
    { }

    virtual const product_type& evaluate() final {
        this->m_evaluationBuffer = m_lhs->evaluate() * m_rhs->evaluate();

        return this->m_evaluationBuffer;
    }

};

#endif // SDIFF_OPERATORS_H