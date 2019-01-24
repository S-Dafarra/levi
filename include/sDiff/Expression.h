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
    void default_constructor(bool_value<value>) {}

    void default_constructor(bool_value<true>) {
        m_evaluable = std::make_shared<EvaluableT>();
    }

    void default_constructor(bool_value<false>) {
        m_evaluable = nullptr;
    }

    //See https://stackoverflow.com/questions/1005476/how-to-detect-whether-there-is-a-specific-member-variable-in-class
    template<class Matrix, typename = int>
    struct has_equal_to_constant_operator : std::false_type { };

    template<class Matrix>
    struct has_equal_to_constant_operator<Matrix, decltype(std::declval<EvaluableT>().operator=(std::declval<Matrix>()), 0)> : std::true_type { };

    template<bool value, typename OtherEvaluable>
    void casted_assignement(bool_value<value>, std::shared_ptr<OtherEvaluable> other) {}

    template<typename OtherEvaluable>
    void casted_assignement(bool_value<true>, std::shared_ptr<OtherEvaluable> other) {
        m_evaluable = other;
    }

    template<typename OtherEvaluable>
    void casted_assignement(bool_value<false>, std::shared_ptr<OtherEvaluable> other) {
        m_evaluable = std::make_shared<sDiff::CastEvaluable<EvaluableT, OtherEvaluable>>(other);
    }

public:

    ExpressionComponent()
    {
        default_constructor(bool_value<std::is_constructible<EvaluableT>::value>());
    }

    template<class EvaluableOther>
    ExpressionComponent(const ExpressionComponent<EvaluableOther>& other) {
        this = other;
    }

    template<class EvaluableOther>
    ExpressionComponent(ExpressionComponent<EvaluableOther>&& other) {
        *this = other;
    }

    template<class... Args >
    ExpressionComponent(Args&&... args)
        : m_evaluable(std::make_shared<EvaluableT>(args...))
    { }

    std::weak_ptr<EvaluableT> evaluable() {
        return m_evaluable;
    }

    std::string name() const {
        assert(m_evaluable);
        return m_evaluable->name();
    }

    Eigen::Index rows() const {
        assert(m_evaluable);
        return m_evaluable->rows();
    }

    Eigen::Index cols() const {
        assert(m_evaluable);
        return m_evaluable->cols();
    }

    const Eigen::MatrixBase<typename EvaluableT::matrix_type>& evaluate() {
        assert(m_evaluable);
        return m_evaluable->evaluate();
    }

    template<class EvaluableRhs>
    ExpressionComponent<sDiff::Evaluable<
    typename matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator+(const ExpressionComponent<EvaluableRhs>& rhs) {
        assert(rows() == rhs.rows());
        assert(cols() == rhs.cols());
        assert(m_evaluable);
        assert(rhs.m_evaluable);

        ExpressionComponent<sDiff::Evaluable<
                typename matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

        newExpression = ExpressionComponent<SumEvaluable<EvaluableT, EvaluableRhs>>(this->m_evaluable, rhs.m_evaluable);

        return newExpression;
    }

    template <typename Matrix>
    ExpressionComponent<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator+(const Matrix& rhs) {
        ExpressionComponent<ConstantEvaluable<Matrix>> constant = build_constant(bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

        return operator+(constant);
    }

    template<class EvaluableRhs>
    ExpressionComponent<sDiff::Evaluable<
    typename matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator-(const ExpressionComponent<EvaluableRhs>& rhs) {
        assert(rows() == rhs.rows());
        assert(cols() == rhs.cols());
        assert(m_evaluable);
        assert(rhs.m_evaluable);

        ExpressionComponent<sDiff::Evaluable<
                typename matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

        newExpression = ExpressionComponent<SubtractionEvaluable<EvaluableT, EvaluableRhs>>(this->m_evaluable, rhs.m_evaluable);

        return newExpression;
    }

    template <typename Matrix>
    ExpressionComponent<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator-(const Matrix& rhs) {
        ExpressionComponent<ConstantEvaluable<Matrix>> constant = build_constant(bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

        return operator-(constant);
    }

    template<class EvaluableRhs>
    ExpressionComponent<sDiff::Evaluable<
    typename matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator*(const ExpressionComponent<EvaluableRhs>& rhs) {
        assert((cols() == 1 && rows() == 1) || (rhs.cols() == 1 && rhs.rows() == 1) || (cols() == rhs.rows()) && "Dimension mismatch for product.");
        assert(m_evaluable);
        assert(rhs.m_evaluable);

        ExpressionComponent<sDiff::Evaluable<typename matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

        newExpression = ExpressionComponent<ProductEvaluable<EvaluableT, EvaluableRhs>>(this->m_evaluable, rhs.m_evaluable);

        return newExpression;
    }

    template <typename Matrix>
    ExpressionComponent<sDiff::Evaluable<typename matrix_product_return<typename EvaluableT::matrix_type, Matrix>::type>> operator*(const Matrix& rhs) {
        ExpressionComponent<ConstantEvaluable<Matrix>> constant = build_constant(bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

        return operator*(constant);
    }

    template<class EvaluableRhs>
    ExpressionComponent<EvaluableT>& operator=(const ExpressionComponent<EvaluableRhs>& rhs) {
        static_assert (!std::is_base_of<sDiff::EvaluableVariable<typename EvaluableT::matrix_type>, EvaluableT>::value, "Cannot assign an expression to a variable." );
        casted_assignement(bool_value<std::is_base_of<EvaluableT, EvaluableRhs>::value>(), rhs.m_evaluable);
        return *this;
    }

    template<class EvaluableRhs>
    ExpressionComponent<EvaluableT>& operator=(const ExpressionComponent<EvaluableRhs>&& rhs) {
        static_assert (!std::is_base_of<sDiff::EvaluableVariable<typename EvaluableT::matrix_type>, EvaluableT>::value, "Cannot assign an expression to a variable." );
        casted_assignement(bool_value<std::is_base_of<EvaluableT, EvaluableRhs>::value>(), rhs.m_evaluable);
        return *this;
    }

    //assign from a constant
    template<typename Matrix>
    void operator=(const Matrix& rhs) {
        static_assert (has_equal_to_constant_operator<Matrix>(), "This expression cannot be set equal to a constant.");
        assert(m_evaluable && "This expression cannot be set because the constructor was not called properly.");
        (*m_evaluable) = rhs;
    }

    ExpressionComponent<sDiff::Evaluable<typename RowEvaluable<EvaluableT>::row_type>> row(Eigen::Index row) {
        assert(row < this->rows());
        assert(m_evaluable && "Cannot extract a row from this expression");

        ExpressionComponent<RowEvaluable<EvaluableT>> selectedRow(m_evaluable, row);
        assert(selectedRow.m_evaluable);

        return selectedRow;
    }

};

template <typename Matrix, class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator+(const Matrix& lhs, const sDiff::ExpressionComponent<EvaluableT> &rhs) {
    sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Matrix>> constant =
            sDiff::build_constant(bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return constant + rhs;
}

template <typename Matrix, class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator-(const Matrix& lhs, const sDiff::ExpressionComponent<EvaluableT> &rhs) {
    sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Matrix>> constant =
            sDiff::build_constant(bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return constant - rhs;
}

template <typename Matrix, class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename matrix_product_return<Matrix, typename EvaluableT::matrix_type>::type>> operator*(const Matrix& lhs, const sDiff::ExpressionComponent<EvaluableT> &rhs) {

    sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Matrix>> newConstant =
            sDiff::build_constant<Matrix>(bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return newConstant * rhs;
}


#endif // SDIFF_EXPRESSION_COMPONENT_H
