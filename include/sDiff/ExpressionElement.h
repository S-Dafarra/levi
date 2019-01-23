/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_EXPRESSION_ELEMENT_H
#define SDIFF_EXPRESSION_ELEMENT_H

#include <sDiff/ForwardDeclarations.h>
#include <sDiff/OperatorsEvaluables.h>
#include <sDiff/Variable.h>

template<bool T>
struct bool_value { static const bool value = T; };

namespace sDiff {

    template<bool value, typename T>
    static ExpressionElement<ConstantEvaluable<T>> build_constant(bool_value<value>, const T& rhs) { }

    template<typename T>
    static ExpressionElement<ConstantEvaluable<T>> build_constant(bool_value<true>, const T& rhs) {
        ExpressionElement<ConstantEvaluable<T>> constant(rhs);
        return constant;
    }

    template<typename T>
    static ExpressionElement<ConstantEvaluable<T>> build_constant(bool_value<false>, const T& rhs) {
        ExpressionElement<ConstantEvaluable<T>> constant(rhs, "UnnamedMatrix");
        return constant;
    }

}

template <class EvaluableT>
class sDiff::ExpressionElement {

    template <class OtherEvaluable>
    friend class ExpressionElement;

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

public:

    ExpressionElement()
    {
        default_constructor(bool_value<std::is_constructible<EvaluableT>::value>());
    }

    template<class EvaluableOther>
    ExpressionElement(const ExpressionElement<EvaluableOther>& other) {
        this = other;
    }

    template<class EvaluableOther>
    ExpressionElement(ExpressionElement<EvaluableOther>&& other) {
        *this = other;
    }

    template<class... Args >
    ExpressionElement(Args&&... args)
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
    ExpressionElement<sDiff::Evaluable<
    typename matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator+(const ExpressionElement<EvaluableRhs>& rhs) {
        assert(rows() == rhs.rows());
        assert(cols() == rhs.cols());
        assert(m_evaluable);
        assert(rhs.m_evaluable);

        ExpressionElement<sDiff::Evaluable<
                typename matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

        newExpression = ExpressionElement<SumEvaluable<EvaluableT, EvaluableRhs>>(this->m_evaluable, rhs.m_evaluable);

        return newExpression;
    }

    template <typename Matrix>
    ExpressionElement<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator+(const Matrix& rhs) {
        ExpressionElement<ConstantEvaluable<Matrix>> constant = build_constant(bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

        return operator+(constant);
    }

    template<class EvaluableRhs>
    ExpressionElement<sDiff::Evaluable<
    typename matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator-(const ExpressionElement<EvaluableRhs>& rhs) {
        assert(rows() == rhs.rows());
        assert(cols() == rhs.cols());
        assert(m_evaluable);
        assert(rhs.m_evaluable);

        ExpressionElement<sDiff::Evaluable<
                typename matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

        newExpression = ExpressionElement<SubtractionEvaluable<EvaluableT, EvaluableRhs>>(this->m_evaluable, rhs.m_evaluable);

        return newExpression;
    }

    template <typename Matrix>
    ExpressionElement<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator-(const Matrix& rhs) {
        ExpressionElement<ConstantEvaluable<Matrix>> constant = build_constant(bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

        return operator-(constant);
    }

    template<class EvaluableRhs>
    ExpressionElement<sDiff::Evaluable<
    typename matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator*(const ExpressionElement<EvaluableRhs>& rhs) {
        assert((cols() == 1 && rows() == 1) || (rhs.cols() == 1 && rhs.rows() == 1) || (cols() == rhs.rows()) && "Dimension mismatch for product.");
        assert(m_evaluable);
        assert(rhs.m_evaluable);

        ExpressionElement<sDiff::Evaluable<typename matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

        newExpression = ExpressionElement<ProductEvaluable<EvaluableT, EvaluableRhs>>(this->m_evaluable, rhs.m_evaluable);

        return newExpression;
    }

    template <typename Matrix>
    ExpressionElement<sDiff::Evaluable<typename matrix_product_return<typename EvaluableT::matrix_type, Matrix>::type>> operator*(const Matrix& rhs) {
        ExpressionElement<ConstantEvaluable<Matrix>> constant = build_constant(bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

        return operator*(constant);
    }

    template<class EvaluableRhs>
    void operator=(const ExpressionElement<EvaluableRhs>& rhs) {
        static_assert (!std::is_base_of<sDiff::EvaluableVariable<typename EvaluableT::matrix_type>, EvaluableT>::value, "Cannot assign an expression to a variable." );
        this->m_evaluable = std::dynamic_pointer_cast<EvaluableT>(rhs.m_evaluable);
    }

    template<class EvaluableRhs>
    ExpressionElement<EvaluableT>& operator=(const ExpressionElement<EvaluableRhs>&& rhs) {
        static_assert (!std::is_base_of<sDiff::EvaluableVariable<typename EvaluableT::matrix_type>, EvaluableT>::value, "Cannot assign an expression to a variable." );
        this->m_evaluable = std::dynamic_pointer_cast<EvaluableT>(rhs.m_evaluable);
    }

    //assign from a constant
    template<typename Matrix>
    void operator=(const Matrix& rhs) {
        static_assert (has_equal_to_constant_operator<Matrix>(), "This expression cannot be set equal to a constant.");
        assert(m_evaluable && "This expression cannot be set because the constructor was not called properly.");
        (*m_evaluable) = rhs;
    }

};

template <typename Matrix, class EvaluableT>
sDiff::ExpressionElement<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator+(const Matrix& lhs, const sDiff::ExpressionElement<EvaluableT> &rhs) {
    sDiff::ExpressionElement<sDiff::ConstantEvaluable<Matrix>> constant =
            sDiff::build_constant(bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return constant + rhs;
}

template <typename Matrix, class EvaluableT>
sDiff::ExpressionElement<sDiff::Evaluable<typename matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator-(const Matrix& lhs, const sDiff::ExpressionElement<EvaluableT> &rhs) {
    sDiff::ExpressionElement<sDiff::ConstantEvaluable<Matrix>> constant =
            sDiff::build_constant(bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return constant - rhs;
}

template <typename Matrix, class EvaluableT>
sDiff::ExpressionElement<sDiff::Evaluable<typename matrix_product_return<Matrix, typename EvaluableT::matrix_type>::type>> operator*(const Matrix& lhs, const sDiff::ExpressionElement<EvaluableT> &rhs) {

    sDiff::ExpressionElement<sDiff::ConstantEvaluable<Matrix>> newConstant =
            sDiff::build_constant<Matrix>(bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return newConstant * rhs;
}


#endif // SDIFF_EXPRESSION_ELEMENT_H
