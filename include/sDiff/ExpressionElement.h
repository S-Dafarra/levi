/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_EXPRESSION_ELEMENT_H
#define SDIFF_EXPRESSION_ELEMENT_H

#include <sDiff/OperatorsEvaluables.h>
#include <memory>
#include <type_traits>
#include <Eigen/Core>
#include <string>
#include <cassert>

namespace sDiff {

    template <class Evaluable>
    class ExpressionElement;

}

template<bool T>
struct bool_value { static const bool value = T; };

template <class Evaluable>
class sDiff::ExpressionElement {

    template <class OtherEvaluable>
    friend class ExpressionElement;

    std::shared_ptr<Evaluable> m_evaluable;

    template<bool value>
    void construct(bool_value<value>) {}

    void construct(bool_value<true>) {
        m_evaluable = std::make_shared<Evaluable>();
    }

    void construct(bool_value<false>) {
        m_evaluable = nullptr;
    }

    //See https://stackoverflow.com/questions/1005476/how-to-detect-whether-there-is-a-specific-member-variable-in-class
    template<class Matrix, typename = int>
    struct has_equal_to_constant_operator : std::false_type { };

    template<class Matrix>
    struct has_equal_to_constant_operator<Matrix, decltype(std::declval<Evaluable>().operator=(std::declval<Matrix>()), 0)> : std::true_type { };

public:

    ExpressionElement()
    {
        construct(bool_value<std::is_constructible<Evaluable>::value>());
    }

    template<class EvaluableOther>
    ExpressionElement(const ExpressionElement<EvaluableOther>& other) = delete;

    template<class EvaluableOther>
    ExpressionElement(ExpressionElement<EvaluableOther>&& other) = delete;

    template<class... Args >
    ExpressionElement(Args&&... args)
        : m_evaluable(std::make_shared<Evaluable>(args...))
    { }

    std::weak_ptr<Evaluable> evaluable() {
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

    const Eigen::MatrixBase<typename Evaluable::matrix_type>& evaluate() {
        assert(m_evaluable);
        return m_evaluable->evaluate();
    }

    template<class EvaluableRhs>
    ExpressionElement<SumEvaluable<Evaluable, EvaluableRhs, typename Evaluable::matrix_type>> operator+(const ExpressionElement<EvaluableRhs>& rhs) {
        assert(rows() == rhs.rows());
        assert(cols() == rhs.cols());
        assert(m_evaluable);
        assert(rhs.m_evaluable);

        return ExpressionElement<SumEvaluable<Evaluable, EvaluableRhs, typename Evaluable::matrix_type>>(this->m_evaluable, rhs.m_evaluable);
    }

    template<class EvaluableRhs>
    void operator=(const ExpressionElement<EvaluableRhs>& rhs) {
        static_assert (!Evaluable::is_variable, "Cannot assign an expression to a variable." );
        this->m_evaluable = rhs.m_evaluable;
    }

    //assign from a constant
    template<typename Matrix>
    void operator=(const Matrix& rhs) {
        static_assert (has_equal_to_constant_operator<Matrix>(), "This expression cannot be set equal to a constant.");
        assert(m_evaluable && "This expression cannot be set because the constructor was not called properly.");
        m_evaluable->operator=(rhs);
    }

};


#endif // SDIFF_EXPRESSION_ELEMENT_H
