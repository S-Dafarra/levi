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
#include <type_traits>

namespace sDiff {
    template<typename Matrix, class Enabler = void>
    class Evaluable { };

    template <typename Matrix>
    class Evaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type>;

    template <typename Scalar>
    class Evaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>;
}

template<typename Matrix>
class sDiff::Evaluable<Matrix, typename std::enable_if<!std::is_arithmetic<Matrix>::value>::type> {

    std::string m_name;

protected:

    Matrix m_evaluationBuffer;

public:

    Evaluable() = delete;

    Evaluable(const std::string& name)
        : m_name(name)
    { }

    Evaluable(Eigen::Index rows, Eigen::Index cols, const std::string& name)
        : m_name(name)
        , m_evaluationBuffer(rows, cols)
    {
        m_evaluationBuffer.setZero();
    }

    Evaluable(const Matrix& initialValue, const std::string& name)
        : m_name(name)
        , m_evaluationBuffer(initialValue)
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

    void resize(Eigen::Index newRows, Eigen::Index newCols) {
        m_evaluationBuffer.resize(newRows, newCols);
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

template <typename Scalar>
class sDiff::Evaluable<Scalar, typename std::enable_if<std::is_arithmetic<Scalar>::value>::type> {

    std::string m_name;

protected:

    Scalar m_evaluationBuffer;

public:

    Evaluable() = delete;

    Evaluable(const std::string& name)
        : m_name(name)
    { }

    Evaluable(const Scalar& initialValue, const std::string& name)
        : m_name(name)
        , m_evaluationBuffer(initialValue)
    { }

    Evaluable(const Scalar& initialValue)
        : m_name(std::to_string(initialValue))
        , m_evaluationBuffer(initialValue)
    { }

    Evaluable(const Evaluable& other) = delete;

    Evaluable(Evaluable&& other) = delete;

    virtual ~Evaluable() { }

    Eigen::Index rows() const {
        return 1;
    }

    Eigen::Index cols() const {
        return 1;
    }

    std::string name() const {
        return m_name;
    }

    virtual const Scalar& evaluate() = 0;

    Evaluable<Scalar>& operator=(const Evaluable& other) = delete;

    void operator=(Evaluable&& other) = delete;

    Evaluable<Scalar>& operator+(const Evaluable& other) const  = delete;

    Evaluable<Scalar>& operator-(const Evaluable& other) const = delete;

    Evaluable<Scalar>& operator*(const Evaluable& other) const = delete;

    typedef Scalar matrix_type;

    static const bool is_variable = false;

// manca la derivata
};

#endif // SDIFF_EVALUABLE_H
