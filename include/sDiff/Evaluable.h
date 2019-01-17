/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_EVALUABLE_H
#define SDIFF_EVALUABLE_H

#include<Eigen/Core>
#include<string>

namespace sDiff {
    template<typename Matrix = Eigen::MatrixXd>
    class Evaluable;
}

template<typename Matrix>
class sDiff::Evaluable {

    std::string m_name;

protected:

    Matrix m_evaluationBuffer;

public:

    Evaluable() = delete;

    Evaluable(Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : m_name(name)
        , m_evaluationBuffer(rows, cols)
    { }

    Evaluable(const Evaluable& other) = delete;

    Evaluable(Evaluable&& other) = delete;

    virtual ~Evaluable() { }

    Eigen::Index rows() const {
        return m_evaluationBuffer.rows();
    }

    Eigen::Index cols() const {
        return m_evaluationBuffer.cols();
    }

    std::string name() const {
        return m_name;
    }

    virtual const Matrix& evaluate() = 0;

    Evaluable<Matrix>& operator=(const Evaluable& other) = delete;

    void operator=(Evaluable&& other) = delete;

    Evaluable<Matrix>& operator+(const Evaluable& other) const  = delete;

    Evaluable<Matrix>& operator-(const Evaluable& other) const = delete;

    Evaluable<Matrix>& operator*(const Evaluable& other) const = delete;

    typedef Matrix matrix_type;

    static const bool is_variable = false;

// manca la derivata
};

#endif // SDIFF_EVALUABLE_H
