/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_EXPRESSION_H
#define SDIFF_EXPRESSION_H

#include <sDiff/OperatorsEvaluables.h>
#include <memory>
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
        this->m_evaluable = rhs.m_evaluable;
    }


};


#endif // SDIFF_EXPRESSION_H
