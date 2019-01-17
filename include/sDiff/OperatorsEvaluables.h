/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_OPERATORS_EVALUABLES_H
#define SDIFF_OPERATORS_EVALUABLES_H

#include <sDiff/Evaluable.h>
#include <Eigen/Core>
#include <memory>

namespace sDiff {

    template <typename Matrix = Eigen::MatrixXd>
    class NullEvaluable;

    template <typename Matrix = Eigen::MatrixXd>
    class ConstantEvaluable;

    template <class LeftEvaluable, class RightEvaluable, typename Matrix = Eigen::MatrixXd>
    class SumEvaluable;
}

template <typename Matrix>
class sDiff::NullEvaluable : public sDiff::Evaluable<Matrix>{
public:

    NullEvaluable()
        : Evaluable<Matrix>(0, 0, "")
    { }

    virtual const Matrix& evaluate() final {
        return this->m_evaluationBuffer;
    }
};

template <typename Matrix>
class sDiff::ConstantEvaluable : public sDiff::Evaluable<Matrix>{
public:

    ConstantEvaluable(const Matrix& constant, std::string name)
        : Evaluable<Matrix>(constant.rows(), constant.cols(), name)
    {
        this->m_evaluationBuffer = constant;
    }

    ConstantEvaluable(Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : Evaluable<Matrix>(rows, cols, name)
    {
        this->m_evaluationBuffer.setZero();
    }

    virtual const Matrix& evaluate() final {
        return this->m_evaluationBuffer;
    }

    void operator=(const Matrix& rhs) {
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

#endif // SDIFF_OPERATORS_H
