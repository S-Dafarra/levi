/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_EXPRESSION_COMPONENT_H
#define SDIFF_EXPRESSION_COMPONENT_H

#include <sDiff/ForwardDeclarations.h>
#include <sDiff/OperatorsEvaluables.h>
#include <sDiff/Variable.h>

template<bool T>
struct bool_value { static const bool value = T; };

namespace sDiff {

    template<bool value, typename T>
    static ExpressionComponent<ConstantEvaluable<T>> build_constant(bool_value<value>, const T& rhs) { }

    template<typename T>
    static ExpressionComponent<ConstantEvaluable<T>> build_constant(bool_value<true>, const T& rhs) {
        ExpressionComponent<ConstantEvaluable<T>> constant(rhs);
        return constant;
    }

    template<typename T>
    static ExpressionComponent<ConstantEvaluable<T>> build_constant(bool_value<false>, const T& rhs) {
        ExpressionComponent<ConstantEvaluable<T>> constant(rhs, "UnnamedMatrix");
        return constant;
    }

}

template <class EvaluableT>
class sDiff::ExpressionComponent {

    template <class OtherEvaluable>
    friend class ExpressionComponent;

    std::shared_ptr<EvaluableT> m_evaluable;

    template<bool value>
    void default_constructor(bool_value<value>);

    void default_constructor(bool_value<true>);

    void default_constructor(bool_value<false>);

    //See https://stackoverflow.com/questions/1005476/how-to-detect-whether-there-is-a-specific-member-variable-in-class
    template<class Matrix, typename = int>
    struct has_equal_to_constant_operator : std::false_type { };

    template<class Matrix>
    struct has_equal_to_constant_operator<Matrix, decltype(std::declval<EvaluableT>().operator=(std::declval<Matrix>()), 0)> : std::true_type { };

    template<bool value, typename OtherEvaluable>
    void casted_assignement(bool_value<value>, std::shared_ptr<OtherEvaluable> other);

    template<typename OtherEvaluable>
    void casted_assignement(bool_value<true>, std::shared_ptr<OtherEvaluable> other);

    template<typename OtherEvaluable>
    void casted_assignement(bool_value<false>, std::shared_ptr<OtherEvaluable> other);

public:

    ExpressionComponent();

    template<class EvaluableOther>
    ExpressionComponent(const ExpressionComponent<EvaluableOther>& other);

    template<class EvaluableOther>
    ExpressionComponent(ExpressionComponent<EvaluableOther>&& other);

    template<class... Args >
    ExpressionComponent(Args&&... args);

    std::weak_ptr<EvaluableT> evaluable();

    std::string name() const;

    Eigen::Index rows() const;

    Eigen::Index cols() const;

    const Eigen::MatrixBase<typename EvaluableT::matrix_type>& evaluate();

    template<class EvaluableRhs>
    ExpressionComponent<sDiff::Evaluable<
    typename matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator+(const ExpressionComponent<EvaluableRhs>& rhs);

    template <typename Matrix>
    ExpressionComponent<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator+(const Matrix& rhs);

    template<class EvaluableRhs>
    ExpressionComponent<sDiff::Evaluable<
    typename matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator-(const ExpressionComponent<EvaluableRhs>& rhs);

    template <typename Matrix>
    ExpressionComponent<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator-(const Matrix& rhs);

    template<class EvaluableRhs>
    ExpressionComponent<sDiff::Evaluable<
    typename matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator*(const ExpressionComponent<EvaluableRhs>& rhs);

    template <typename Matrix>
    ExpressionComponent<sDiff::Evaluable<typename matrix_product_return<typename EvaluableT::matrix_type, Matrix>::type>> operator*(const Matrix& rhs);

    template<class EvaluableRhs>
    ExpressionComponent<EvaluableT>& operator=(const ExpressionComponent<EvaluableRhs>& rhs);

    template<class EvaluableRhs>
    ExpressionComponent<EvaluableT>& operator=(const ExpressionComponent<EvaluableRhs>&& rhs);

    //assign from a constant
    template<typename Matrix>
    void operator=(const Matrix& rhs);

    ExpressionComponent<sDiff::Evaluable<typename RowEvaluable<EvaluableT>::row_type>> row(Eigen::Index row);

};

template <typename Matrix, class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator+(const Matrix& lhs, const sDiff::ExpressionComponent<EvaluableT> &rhs);

template <typename Matrix, class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator-(const Matrix& lhs, const sDiff::ExpressionComponent<EvaluableT> &rhs);

template <typename Matrix, class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename matrix_product_return<Matrix, typename EvaluableT::matrix_type>::type>> operator*(const Matrix& lhs, const sDiff::ExpressionComponent<EvaluableT> &rhs);


#endif // SDIFF_EXPRESSION_COMPONENT_H
