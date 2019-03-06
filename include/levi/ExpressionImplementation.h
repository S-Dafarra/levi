/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_EXPRESSIONIMPLEMENTATION_H
#define LEVI_EXPRESSIONIMPLEMENTATION_H

#include <levi/Expression.h>
#include <levi/OperatorsEvaluables.h>
#include <levi/AccessorEvaluables.h>
#include <levi/AdvancedConstructors.h>
#include <levi/SqueezeEvaluable.h>
#include <levi/Variable.h>

template<typename EvaluableT>
template<typename OtherInfo>
void levi::ExpressionComponent<EvaluableT>::EvaluableInfo::copy(const OtherInfo& other) {
    type = other.type;
    block = other.block;
    exponent = other.exponent;
    lhs = other.lhs;
    rhs = other.rhs;
}

template<bool value, typename T>
static levi::ExpressionComponent<levi::ConstantEvaluable<T>> levi::build_constant(levi::bool_value<value>, const T& ) { }

template<typename T>
static levi::ExpressionComponent<levi::ConstantEvaluable<T>> levi::build_constant(levi::bool_value<true>, const T& rhs) {
    levi::ExpressionComponent<ConstantEvaluable<T>> constant(rhs);
    return constant;
}

template<typename T>
static levi::ExpressionComponent<levi::ConstantEvaluable<T>> levi::build_constant(levi::bool_value<false>, const T& rhs) {
    levi::ExpressionComponent<levi::ConstantEvaluable<T>> constant(rhs, "UnnamedMatrix");
    return constant;
}

template<int rowsNumber, class EvaluableT>
levi::ExpressionComponent<typename levi::ConstructorByRows<EvaluableT>::composite_evaluable>
levi::ComposeByRows(const std::vector<ExpressionComponent<EvaluableT>>& rows, const std::string& name)
{
    static_assert (EvaluableT::rows_at_compile_time == 1, "ComposeByRows can compose only from expressions which have 1 row at compile time.");
    return levi::ExpressionComponent<levi::ConstructorByRows<EvaluableT, rowsNumber>>(rows, name);
}

template<int colsNumber, typename EvaluableT>
levi::ExpressionComponent<typename levi::ConstructorByCols<EvaluableT>::composite_evaluable>
levi::ComposeByCols(const std::vector<levi::ExpressionComponent<EvaluableT>>& cols, const std::string& name)
{
    static_assert (EvaluableT::cols_at_compile_time == 1, "ComposeByCols can compose only from expressions which have 1 col at compile time.");
    return levi::ExpressionComponent<levi::ConstructorByCols<EvaluableT, colsNumber>>(cols, name);
}

template <class EvaluableT>
template<bool value>
void levi::ExpressionComponent<EvaluableT>::default_constructor(levi::bool_value<value>)
{ }

template <class EvaluableT>
void levi::ExpressionComponent<EvaluableT>::default_constructor(levi::bool_value<true>) {
    m_evaluable = std::make_shared<EvaluableT>();
    m_callerID = m_evaluable->getNewCallerID();
}

template <class EvaluableT>
void levi::ExpressionComponent<EvaluableT>::default_constructor(levi::bool_value<false>) {
    m_evaluable = nullptr;
}

template <class EvaluableT>
template<bool value, typename OtherEvaluable>
void levi::ExpressionComponent<EvaluableT>::casted_assignement(levi::bool_value<value>, const levi::ExpressionComponent<OtherEvaluable>&) {}

template <class EvaluableT>
template<typename OtherEvaluable>
void levi::ExpressionComponent<EvaluableT>::casted_assignement(levi::bool_value<true>, const levi::ExpressionComponent<OtherEvaluable>& other) {
    m_evaluable = other.m_evaluable;
    m_callerID = m_evaluable->getNewCallerID();
}

template <class EvaluableT>
template<typename OtherEvaluable>
void levi::ExpressionComponent<EvaluableT>::casted_assignement(levi::bool_value<false>, const levi::ExpressionComponent<OtherEvaluable>& other) {
    static_assert (!(std::is_base_of<levi::VariableBase, EvaluableT>::value && !std::is_base_of<levi::VariableBase, OtherEvaluable>::value),
                  "Cannot cast an expression to a variable. Try to use the asVariable() method.");
    m_evaluable = std::make_shared<levi::CastEvaluable<EvaluableT, OtherEvaluable>>(other);
    m_callerID = m_evaluable->getNewCallerID();
}

template <class EvaluableT>
template <typename EvaluableOut>
levi::ExpressionComponent<EvaluableOut> levi::ExpressionComponent<EvaluableT>::return_this(levi::bool_value<true>) const {
    return *this;
}

template <class EvaluableT>
template <typename EvaluableOut>
levi::ExpressionComponent<EvaluableOut> levi::ExpressionComponent<EvaluableT>::return_this(levi::bool_value<false>) const {
    return levi::ExpressionComponent<EvaluableOut>();
}

template <class EvaluableT>
template<typename EvaluableOut, typename EvaluableRhs>
levi::ExpressionComponent<EvaluableOut> levi::ExpressionComponent<EvaluableT>::return_rhs(levi::bool_value<true>,
                                                                                          const levi::ExpressionComponent<EvaluableRhs>& rhs) const {
    return rhs;
}

template <class EvaluableT>
template<typename EvaluableOut, typename EvaluableRhs>
levi::ExpressionComponent<EvaluableOut> levi::ExpressionComponent<EvaluableT>::return_rhs(levi::bool_value<false>,
                                                                                          const levi::ExpressionComponent<EvaluableRhs>& rhs) const {
    levi::unused(rhs);
    return levi::ExpressionComponent<EvaluableOut>();
}

template <class EvaluableT>
template <typename OtherInfo>
void levi::ExpressionComponent<EvaluableT>::copyInfo(const OtherInfo* other) {
    if (other) {
        if (!m_info) {
            m_info = new EvaluableInfo;
        }
        m_info->copy(*other);
    }
}

template <class EvaluableT>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent()
{
    default_constructor(levi::bool_value<std::is_constructible<EvaluableT>::value>());
    if (m_evaluable) {
        m_info = new EvaluableInfo;
        m_info->type = levi::detectType(*this);
    } else {
        m_info = nullptr;
    }
}

template <class EvaluableT>
template<class EvaluableOther, typename>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(const ExpressionComponent<EvaluableOther>& other)
    : m_info(new EvaluableInfo)
{

    copyInfo(other.m_info);
    if (other.isValidExpression()) {
        casted_assignement(levi::bool_value<std::is_base_of<EvaluableT, EvaluableOther>::value>(), other);
    } else {
        m_evaluable = nullptr;
    }
}

template <class EvaluableT>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(const ExpressionComponent<EvaluableT>& other)
    : m_info(new EvaluableInfo)
{
    copyInfo(other.m_info);
    m_evaluable = other.m_evaluable;
    if (m_evaluable) {
        m_callerID = m_evaluable->getNewCallerID();
    }
}

template <class EvaluableT>
template<class EvaluableOther, typename>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(ExpressionComponent<EvaluableOther>&& other)
    : m_info(new EvaluableInfo)
{
    copyInfo(other.m_info);
    if (other.isValidExpression()) {
        casted_assignement(levi::bool_value<std::is_base_of<EvaluableT, EvaluableOther>::value>(), other);
    } else {
        m_evaluable = nullptr;
    }
}

template <class EvaluableT>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(ExpressionComponent<EvaluableT>&& other)
    : m_info(new EvaluableInfo)
{
    copyInfo(other.m_info);
    m_evaluable = other.m_evaluable;
    if (m_evaluable) {
        m_callerID = m_evaluable->getNewCallerID();
    }
}

template <class EvaluableT>
template<class... Args, typename>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(Args&&... args)
    : m_evaluable(std::make_shared<EvaluableT>(std::forward<Args>(args)...))
      , m_info(new EvaluableInfo)
{
    m_callerID = m_evaluable->getNewCallerID();
    m_info->type = levi::detectType(*this);
}

template <class EvaluableT>
template<class... Args, typename>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(const levi::ExpressionComponent<EvaluableT>::EvaluableInfo& info, Args&&... args)
    : m_evaluable(std::make_shared<EvaluableT>(std::forward<Args>(args)...))
      , m_info(new EvaluableInfo(info))
{
    m_callerID = m_evaluable->getNewCallerID();
    m_info->type = levi::detectType(*this);
}

template<class EvaluableT>
levi::ExpressionComponent<EvaluableT>::~ExpressionComponent()
{
    if (m_evaluable) {
        m_evaluable->deleteID(m_callerID);
    }

    if (m_info) {
        delete m_info;
        m_info = nullptr;
    }
}

template <class EvaluableT>
std::weak_ptr<EvaluableT> levi::ExpressionComponent<EvaluableT>::evaluable() const {
    return m_evaluable;
}

template <class EvaluableT>
std::string levi::ExpressionComponent<EvaluableT>::name() const {
    assert(m_evaluable && "This expression is empty.");
    return m_evaluable->name();
}

template <class EvaluableT>
Eigen::Index levi::ExpressionComponent<EvaluableT>::rows() const {
    assert(m_evaluable && "This expression is empty.");
    return m_evaluable->rows();
}

template <class EvaluableT>
Eigen::Index levi::ExpressionComponent<EvaluableT>::cols() const {
    assert(m_evaluable && "This expression is empty.");
    return m_evaluable->cols();
}

template <class EvaluableT>
bool levi::ExpressionComponent<EvaluableT>::isNew() const {
    if (!m_evaluable)
        return false;

    return m_evaluable->isNew(m_callerID);
}

template <class EvaluableT>
bool levi::ExpressionComponent<EvaluableT>::isNull() const {
    if (!m_evaluable)
        return true;

    return m_info->type == levi::EvaluableType::Null;
}

template <class EvaluableT>
levi::EvaluableType levi::ExpressionComponent<EvaluableT>::type() const {
    if (m_info) {
        return m_info->type;
    } else {
        return levi::EvaluableType::Generic;
    }
}

template <class EvaluableT>
const typename EvaluableT::EvaluableInfo& levi::ExpressionComponent<EvaluableT>::info() const {
    assert(m_evaluable && "Cannot get infos from this expression.");
    return m_evaluable->info();
}


template <class EvaluableT>
const typename EvaluableT::matrix_type &levi::ExpressionComponent<EvaluableT>::evaluate(bool checkDependencies) {
    assert(m_evaluable && "This expression is empty.");
    return m_evaluable->evaluateID(m_callerID, checkDependencies);
}

template <class EvaluableT>
template<class EvaluableRhs>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> levi::ExpressionComponent<EvaluableT>::operator+(const levi::ExpressionComponent<EvaluableRhs>& rhs) const {
    assert(rows() == rhs.rows());
    assert(cols() == rhs.cols());
    assert(m_evaluable && "This expression is empty.");
    assert(rhs.m_evaluable);

    if (m_info->type == levi::EvaluableType::Null) {
        return rhs;
    }

    if (rhs.m_info->type == levi::EvaluableType::Null) {
        return *this;
    }

    typename ExpressionComponent<levi::SumEvaluable<EvaluableT, EvaluableRhs>>::EvaluableInfo info;
    info.lhs = *this;
    info.rhs = rhs;

    return levi::ExpressionComponent<levi::SumEvaluable<EvaluableT, EvaluableRhs>>(info, *this, rhs);
}

template <class EvaluableT>
template <typename Matrix>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> levi::ExpressionComponent<EvaluableT>::operator+(const Matrix& rhs) const {
    levi::ExpressionComponent<levi::ConstantEvaluable<Matrix>> constant = levi::build_constant(levi::bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

    return operator+(constant);
}

template <class EvaluableT>
template<class EvaluableRhs>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> levi::ExpressionComponent<EvaluableT>::operator-(const levi::ExpressionComponent<EvaluableRhs>& rhs) const {
    assert(rows() == rhs.rows());
    assert(cols() == rhs.cols());
    assert(m_evaluable && "This expression is empty.");
    assert(rhs.m_evaluable);

    if (m_info->type == levi::EvaluableType::Null) {
        return -rhs;
    }

    if (rhs.m_info->type == levi::EvaluableType::Null) {
        return *this;
    }

    typename ExpressionComponent<levi::SubtractionEvaluable<EvaluableT, EvaluableRhs>>::EvaluableInfo info;
    info.lhs = *this;
    info.rhs = rhs;

    return levi::ExpressionComponent<levi::SubtractionEvaluable<EvaluableT, EvaluableRhs>>(info, *this, rhs);
}

template <class EvaluableT>
template <typename Matrix>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> levi::ExpressionComponent<EvaluableT>::operator-(const Matrix& rhs) const {
    levi::ExpressionComponent<levi::ConstantEvaluable<Matrix>> constant = levi::build_constant(levi::bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

    return operator-(constant);
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::matrix_type>> levi::ExpressionComponent<EvaluableT>::operator-() const
{
    assert(m_evaluable && "This expression is empty.");
    levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::matrix_type>> newExpression;

    if (m_info->type == levi::EvaluableType::Null) {
        return *this;
    }

    typename ExpressionComponent<levi::SignInvertedEvaluable<EvaluableT>>::EvaluableInfo info;
    info.lhs = *this;

    newExpression = levi::ExpressionComponent<levi::SignInvertedEvaluable<EvaluableT>>(info, *this, 0);

    return newExpression;
}

template <class EvaluableT>
template<class EvaluableRhs>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> levi::ExpressionComponent<EvaluableT>::operator*(const levi::ExpressionComponent<EvaluableRhs>& rhs) const {
    assert((cols() == 1 && rows() == 1) || (rhs.cols() == 1 && rhs.rows() == 1) || (cols() == rhs.rows()) && "Dimension mismatch for product.");
    assert(m_evaluable && "This expression is empty.");
    assert(rhs.m_evaluable);

    if (m_info->type == levi::EvaluableType::Null || rhs.m_info->type == levi::EvaluableType::Null) {
        return levi::ExpressionComponent<levi::NullEvaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>>((rows() == 1 && cols() == 1 && rhs.rows() != 1)? rhs.rows() : rows(),
                                                                                                                                                                                (rhs.rows() == 1 && rhs.cols() == 1 && cols() != 1)? cols() : rhs.cols());
    }

    typedef levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type> return_evaluable;

    if (rhs.m_info->type == levi::EvaluableType::Identity && (rhs.rows() == rhs.cols()) && ((rhs.rows() == 1) || (cols() == rhs.rows()))) { //i.e. it's not the case of scalar * I
        return return_this<return_evaluable>(bool_value<(((return_evaluable::rows_at_compile_time == EvaluableT::rows_at_compile_time) ||
                                                          (return_evaluable::rows_at_compile_time * EvaluableT::rows_at_compile_time < 0)) &&
                                                         ((return_evaluable::cols_at_compile_time == EvaluableT::cols_at_compile_time) ||
                                                          (return_evaluable::cols_at_compile_time * EvaluableT::cols_at_compile_time < 0)))>());
    }

    if (m_info->type == levi::EvaluableType::Identity && (rows() == cols()) && ((rows() == 1) || (cols() == rhs.rows()))) {
        return return_rhs<return_evaluable>(bool_value<(((return_evaluable::rows_at_compile_time == EvaluableRhs::rows_at_compile_time) ||
                                                         (return_evaluable::rows_at_compile_time * EvaluableRhs::rows_at_compile_time < 0)) &&
                                                        ((return_evaluable::cols_at_compile_time == EvaluableRhs::cols_at_compile_time) ||
                                                         (return_evaluable::cols_at_compile_time * EvaluableRhs::cols_at_compile_time < 0)))>(), rhs);
    }

    typename ExpressionComponent<levi::ProductEvaluable<EvaluableT, EvaluableRhs>>::EvaluableInfo info;
    info.lhs = *this;
    info.rhs = rhs;

    return levi::ExpressionComponent<levi::ProductEvaluable<EvaluableT, EvaluableRhs>>(info, *this, rhs);
}

template <class EvaluableT>
template <typename Matrix>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, Matrix>::type>> levi::ExpressionComponent<EvaluableT>::operator*(const Matrix& rhs) const {
    levi::ExpressionComponent<levi::ConstantEvaluable<Matrix>> constant = levi::build_constant(levi::bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

    return operator*(constant);
}

template <class EvaluableT>
template<class EvaluableRhs>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::value_type>::type>>
levi::ExpressionComponent<EvaluableT>::operator/(const levi::ExpressionComponent<EvaluableRhs> &rhs) const
{
    static_assert ((EvaluableRhs::rows_at_compile_time == Eigen::Dynamic || EvaluableRhs::rows_at_compile_time == 1) && (EvaluableRhs::cols_at_compile_time == Eigen::Dynamic || EvaluableRhs::cols_at_compile_time == 1),
                   "The operator/ can be used only when the rhs is a scalar or a 1x1 matrix.");

    assert(rhs.rows() == 1 && rhs.cols() == 1 && "The operator/ can be used only when the rhs is a scalar or a 1x1 matrix.");
    assert(m_evaluable && "This expression is empty.");
    assert(rhs.m_evaluable);

    typename ExpressionComponent<levi::DivisionEvaluable<EvaluableT, EvaluableRhs>>::EvaluableInfo info;
    info.lhs = *this;
    info.rhs = rhs;

    return levi::ExpressionComponent<levi::DivisionEvaluable<EvaluableT, EvaluableRhs>>(info, *this, rhs);
}

template <class EvaluableT>
template<typename Scalar>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, Scalar>::type>> levi::ExpressionComponent<EvaluableT>::operator/(const Scalar &rhs) const
{
    static_assert (std::is_arithmetic<Scalar>::value, "The rhs has to be a scalar.");
    assert(m_evaluable && "This expression is empty.");

    levi::ExpressionComponent<levi::ConstantEvaluable<Scalar>> constant(rhs);

    return operator/(constant);
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::value_type> > levi::ExpressionComponent<EvaluableT>::pow(typename EvaluableT::value_type exponent) const
{
    static_assert ((EvaluableT::rows_at_compile_time == Eigen::Dynamic || EvaluableT::rows_at_compile_time == 1) && (EvaluableT::cols_at_compile_time == Eigen::Dynamic || EvaluableT::cols_at_compile_time == 1),
                   "pow can be used only with scalars or 1x1 matrices.");
    assert(rows() == 1 && cols() == 1 && "pow can be used only with scalars or 1x1 matrices.");

    typename ExpressionComponent<levi::PowEvaluable<EvaluableT>>::EvaluableInfo info;
    info.lhs = *this;
    info.exponent = exponent;

    return ExpressionComponent<levi::PowEvaluable<EvaluableT>>(info, *this, exponent);
}

template <class EvaluableT>
template<class EvaluableRhs, typename>
void levi::ExpressionComponent<EvaluableT>::operator=(const levi::ExpressionComponent<EvaluableRhs>& rhs) {
    static_assert (!std::is_base_of<levi::VariableBase, EvaluableT>::value, "Cannot assign an expression to a variable." );
    copyInfo(rhs.m_info);

    if (m_evaluable) {
        m_evaluable->deleteID(m_callerID);
    }

    if (rhs.isValidExpression()) {
        casted_assignement(levi::bool_value<std::is_base_of<EvaluableT, EvaluableRhs>::value>(), rhs);
    }
}

template <class EvaluableT>
void levi::ExpressionComponent<EvaluableT>::operator=(const levi::ExpressionComponent<EvaluableT>& rhs) {
    copyInfo(rhs.m_info);

    if (m_evaluable) {
        m_evaluable->deleteID(m_callerID);
    }

    m_evaluable = rhs.m_evaluable;
    if (m_evaluable) {
        m_callerID = m_evaluable->getNewCallerID();
    }
}

template <class EvaluableT>
template<class EvaluableRhs, typename>
void levi::ExpressionComponent<EvaluableT>::operator=(const levi::ExpressionComponent<EvaluableRhs>&& rhs) {
    static_assert (!std::is_base_of<levi::VariableBase, EvaluableT>::value, "Cannot assign an expression to a variable." );
    copyInfo(rhs.m_info);

    if (m_evaluable) {
        m_evaluable->deleteID(m_callerID);
    }

    if (rhs.isValidExpression()) {
        casted_assignement(levi::bool_value<std::is_base_of<EvaluableT, EvaluableRhs>::value>(), rhs);
    }
}

template <class EvaluableT>
void levi::ExpressionComponent<EvaluableT>::operator=(const levi::ExpressionComponent<EvaluableT>&& rhs) {
    copyInfo(rhs.m_info);


    if (m_evaluable) {
        m_evaluable->deleteID(m_callerID);
    }

    m_evaluable = rhs.m_evaluable;
    if (m_evaluable) {
        m_callerID = m_evaluable->getNewCallerID();
    }
}

//assign from a constant
template <class EvaluableT>
template<typename Matrix>
void levi::ExpressionComponent<EvaluableT>::operator=(const Matrix& rhs) {
    static_assert (has_equal_to_constant_operator<Matrix>(), "This expression cannot be set equal to a constant.");
    assert(m_evaluable && "This expression cannot be set because the constructor was not called properly.");
    (*m_evaluable) = rhs;
}

template <class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::row_type>> levi::ExpressionComponent<EvaluableT>::row(Eigen::Index row) const {
    assert(row < this->rows());
    assert(m_evaluable && "Cannot extract a row from this expression");

    if (rows() == 1) {
        return return_this<levi::Evaluable<typename EvaluableT::row_type>>(levi::bool_value<(EvaluableT::rows_at_compile_time == 1) || (EvaluableT::rows_at_compile_time == Eigen::Dynamic)>());
    }

    levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::row_type>> rowExpr =
        m_evaluable->row(row);

    if(rowExpr.isValidExpression()) {
        return rowExpr;
    }

    typename ExpressionComponent<levi::RowEvaluable<EvaluableT>>::EvaluableInfo info;
    info.lhs = *this;
    info.block.startRow = row;

    return levi::ExpressionComponent<levi::RowEvaluable<EvaluableT>>(info, *this, row);
}

template <class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::col_type>> levi::ExpressionComponent<EvaluableT>::col(Eigen::Index col) const {
    assert(col < this->cols());
    assert(m_evaluable && "Cannot extract a column from this expression");

    if (cols() == 1) {
        return return_this<levi::Evaluable<typename EvaluableT::col_type>>(levi::bool_value<(EvaluableT::cols_at_compile_time == 1) || (EvaluableT::cols_at_compile_time == Eigen::Dynamic)>());
    }

    levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::col_type>> colExpr =
        m_evaluable->col(col);

    if(colExpr.isValidExpression()) {
        return colExpr;
    }

    typename ExpressionComponent<levi::ColEvaluable<EvaluableT>>::EvaluableInfo info;
    info.lhs = *this;
    info.block.startCol = col;

    return levi::ExpressionComponent<levi::ColEvaluable<EvaluableT>>(info, *this, col);
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::value_type>> levi::ExpressionComponent<EvaluableT>::operator()(Eigen::Index row, Eigen::Index col) const
{
    assert(row < this->rows());
    assert(col < this->cols());
    assert(m_evaluable && "Cannot extract an element from this expression");

    if (cols() == 1 && rows() == 1) {
        return return_this<levi::Evaluable<typename EvaluableT::value_type>>(levi::bool_value<((EvaluableT::rows_at_compile_time == 1) || (EvaluableT::rows_at_compile_time == Eigen::Dynamic)) &&
                                                                             ((EvaluableT::cols_at_compile_time == 1) || (EvaluableT::cols_at_compile_time == Eigen::Dynamic))>());
    }

    levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::value_type>> element =
        m_evaluable->element(row, col);

    if(element.isValidExpression()) {
        return element;
    }

    typename ExpressionComponent<levi::ElementEvaluable<EvaluableT>>::EvaluableInfo info;
    info.lhs = *this;
    info.block.startRow = row;
    info.block.startCol = col;

    return levi::ExpressionComponent<levi::ElementEvaluable<EvaluableT>> (info, *this, row, col);
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type>> levi::ExpressionComponent<EvaluableT>::block(Eigen::Index startRow, Eigen::Index startCol, Eigen::Index numberOfRows, Eigen::Index numberOfCols) const
{
    assert(m_evaluable && "Cannot extract a block from this expression");
    assert(((startRow + numberOfRows) <= rows()) && ((startCol + numberOfCols) <= cols()) && "Invalid block settings.");

    if (m_info->type == levi::EvaluableType::Null) {
        return levi::ExpressionComponent<levi::NullEvaluable<typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type>>(numberOfRows, numberOfCols);
    }

    typename ExpressionComponent<levi::BlockEvaluable<EvaluableT, levi::Evaluable<typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type>>>::EvaluableInfo info;
    info.lhs = *this;
    info.block.startRow = startRow;
    info.block.startCol = startCol;
    info.block.rows = numberOfRows;
    info.block.cols = numberOfCols;

    return levi::ExpressionComponent<levi::BlockEvaluable<EvaluableT, levi::Evaluable<typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type>>>(info, *this, startRow, startCol, numberOfRows, numberOfCols);
}

template<class EvaluableT>
template<unsigned int numberOfRows, unsigned int numberOfCols>
levi::ExpressionComponent<levi::Evaluable<typename levi::fixed_block_return<typename EvaluableT::matrix_type, numberOfRows, numberOfCols>::type> > levi::ExpressionComponent<EvaluableT>::block(Eigen::Index startRow, Eigen::Index startCol) const
{
    assert(m_evaluable && "Cannot extract a block from this expression");
    assert(((startRow + numberOfRows) <= rows()) && ((startCol + numberOfCols) <= cols()) && "Invalid block settings.");

    typedef typename levi::fixed_block_return<typename EvaluableT::matrix_type, numberOfRows, numberOfCols>::type block_type;

    if (m_info->type == levi::EvaluableType::Null) {
        return levi::ExpressionComponent<levi::NullEvaluable<block_type>>();
    }

    typename ExpressionComponent<levi::BlockEvaluable<EvaluableT, levi::Evaluable<block_type>>>::EvaluableInfo info;
    info.lhs = *this;
    info.block.startRow = startRow;
    info.block.startCol = startCol;
    info.block.rows = numberOfRows;
    info.block.cols = numberOfCols;

    return levi::ExpressionComponent<levi::BlockEvaluable<EvaluableT, levi::Evaluable<block_type>>> (info, *this, startRow, startCol, numberOfRows, numberOfCols);
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> levi::ExpressionComponent<EvaluableT>::skew() const
{
    static_assert (((EvaluableT::rows_at_compile_time == Eigen::Dynamic) ||(EvaluableT::rows_at_compile_time == 3)) &&
                   ((EvaluableT::cols_at_compile_time == Eigen::Dynamic) ||(EvaluableT::cols_at_compile_time == 1)) , "Skew can be applied only to three dimensional vectors.");
    assert(m_evaluable && "Cannot compute the skew symmetric matrix from this expression.");
    assert(rows() == 3 && cols() == 1 && "skew can be applied only to three dimensional vectors.");

    return levi::ExpressionComponent<levi::SkewEvaluable<EvaluableT>>(*this, 0);
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::transpose_type<EvaluableT>::type> > levi::ExpressionComponent<EvaluableT>::transpose() const
{
    assert(m_evaluable && "Cannot compute the transpose from this expression.");

    typename ExpressionComponent<levi::TransposeEvaluable<EvaluableT>>::EvaluableInfo info;
    info.lhs = *this;

    return levi::ExpressionComponent<levi::TransposeEvaluable<EvaluableT>>(info, *this, 0);
}


template<class EvaluableT>
levi::ExpressionComponent<levi::EvaluableVariable<typename EvaluableT::col_type> > levi::ExpressionComponent<EvaluableT>::asVariable() const
{
    assert(m_evaluable && "The expression is empty.");

    return levi::ExpressionComponent<levi::VariableFromExpressionEvaluable<EvaluableT>>(*this, 0);
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, Eigen::Dynamic, Eigen::Dynamic>>>
levi::ExpressionComponent<EvaluableT>::squeeze(const std::string& name) const {
    assert(m_evaluable && "Cannot squeeze this expression. It is empty.");

    return levi::ExpressionComponent<levi::SqueezeEvaluable<EvaluableT>>(*this, name);
}

template<typename EvaluableT>
template<typename VariableType>
levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> levi::ExpressionComponent<EvaluableT>::getColumnDerivative(Eigen::Index column,
                                                                                                                                const levi::ExpressionComponent<levi::EvaluableVariable<VariableType>> &variable) const
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(column < cols());
    assert(variable.m_evaluable && "Invalid variable.");

    return m_evaluable->getColumnDerivative(column, variable.m_evaluable);
}

template<typename EvaluableT>
levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> levi::ExpressionComponent<EvaluableT>::getColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable) const
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(column < cols());
    assert(variable && "Empty variable pointer.");

    return m_evaluable->getColumnDerivative(column, variable);
}

template<class EvaluableT>
void levi::ExpressionComponent<EvaluableT>::clearDerivativesCache()
{
    if (m_evaluable) {
        m_evaluable->clearDerivativesCache();
    }
}

template<typename EvaluableT>
template<typename VariableType>
bool levi::ExpressionComponent<EvaluableT>::isDependentFrom(const levi::ExpressionComponent<levi::EvaluableVariable<VariableType>> &variable) const
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(variable.m_evaluable && "Invalid variable.");

    return m_evaluable->isDependentFrom(variable.m_evaluable);
}

template<typename EvaluableT>
bool levi::ExpressionComponent<EvaluableT>::isDependentFrom(std::shared_ptr<levi::VariableBase> variable) const
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(variable && "Empty variable pointer.");

    return m_evaluable->isDependentFrom(variable);
}

template<class EvaluableT>
bool levi::ExpressionComponent<EvaluableT>::isValidExpression() const
{
    return m_evaluable.use_count();
}

template<class EvaluableT>
levi::ExpressionComponent<EvaluableT> levi::ExpressionComponent<EvaluableT>::ComposeByRows(const std::vector<levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::row_type>>> &rows, std::string name)
{
    return levi::ExpressionComponent<levi::ConstructorByRows<levi::Evaluable<typename EvaluableT::row_type>, EvaluableT::rows_at_compile_time>>(rows, name);
}

template<class EvaluableT>
levi::ExpressionComponent<EvaluableT> levi::ExpressionComponent<EvaluableT>::ComposeByCols(const std::vector<levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::col_type>>> &cols, std::string name)
{
    return levi::ExpressionComponent<levi::ConstructorByCols<levi::Evaluable<typename EvaluableT::col_type>, EvaluableT::cols_at_compile_time>>(cols, name);
}

template<class EvaluableT>
template <typename LeftEvaluable, typename RightEvaluable>
levi::ExpressionComponent<EvaluableT> levi::ExpressionComponent<EvaluableT>::Horzcat(const levi::ExpressionComponent<LeftEvaluable>& lhs,
                                                                                     const levi::ExpressionComponent<RightEvaluable>& rhs,
                                                                                     const std::string &name) {
    static_assert(typename LeftEvaluable::value_type() == typename RightEvaluable::value_type(), "You are mixing matrices of different types.");
    static_assert ((LeftEvaluable::rows_at_compile_time == RightEvaluable::rows_at_compile_time) ||
                      (LeftEvaluable::rows_at_compile_time * RightEvaluable::rows_at_compile_time < 0), "The two evaluables have different number of columns");
    assert(lhs.rows() == rhs.rows() && "The two evaluables have different number of columns");
    return levi::ExpressionComponent<levi::HorzcatEvaluable<EvaluableT, LeftEvaluable, RightEvaluable>>(lhs, rhs, name);
}

template<class EvaluableT>
template <typename TopEvaluable, typename BottomEvaluable>
levi::ExpressionComponent<EvaluableT> levi::ExpressionComponent<EvaluableT>::Vertcat(const levi::ExpressionComponent<TopEvaluable>& top,
                                                                                     const levi::ExpressionComponent<BottomEvaluable>& bottom,
                                                                                     const std::string& name) {
    static_assert(typename TopEvaluable::value_type() == typename BottomEvaluable::value_type(), "You are mixing matrices of different types.");
    static_assert ((TopEvaluable::cols_at_compile_time == BottomEvaluable::cols_at_compile_time) ||
                      (TopEvaluable::cols_at_compile_time * BottomEvaluable::cols_at_compile_time < 0), "The two evaluables have different number of columns");
    assert(top.cols() == bottom.cols() && "The two evaluables have different number of columns");
    return levi::ExpressionComponent<levi::VertcatEvaluable<EvaluableT, TopEvaluable, BottomEvaluable>>(top, bottom, name);
}

//end of ExpressionComponent implementation

template <typename Matrix, class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator+(const Matrix& lhs, const levi::ExpressionComponent<EvaluableT> &rhs) {

    levi::ExpressionComponent<levi::ConstantEvaluable<Matrix>> constant =
            levi::build_constant(levi::bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return constant + rhs;
}

template <typename Matrix, class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator-(const Matrix& lhs, const levi::ExpressionComponent<EvaluableT> &rhs) {

    levi::ExpressionComponent<levi::ConstantEvaluable<Matrix>> constant =
            levi::build_constant(levi::bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return constant - rhs;
}

template <typename Matrix, class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<Matrix, typename EvaluableT::matrix_type>::type>> operator*(const Matrix& lhs, const levi::ExpressionComponent<EvaluableT> &rhs) {

    levi::ExpressionComponent<levi::ConstantEvaluable<Matrix>> newConstant =
            levi::build_constant<Matrix>(levi::bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return newConstant * rhs;
}

template <typename Matrix, class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<Matrix, typename EvaluableT::value_type>::type>> operator/(const Matrix& lhs, const levi::ExpressionComponent<EvaluableT> &rhs) {

    levi::ExpressionComponent<levi::ConstantEvaluable<Matrix>> newConstant =
            levi::build_constant<Matrix>(levi::bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return newConstant / rhs;
}

#endif // LEVI_EXPRESSIONIMPLEMENTATION_H
