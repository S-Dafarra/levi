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
#include <levi/Variable.h>

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
    m_evaluable = std::make_shared<levi::CastEvaluable<EvaluableT, OtherEvaluable>>(other);
    m_callerID = m_evaluable->getNewCallerID();
}

template <class EvaluableT>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent()
{
    default_constructor(levi::bool_value<std::is_constructible<EvaluableT>::value>());
}

template <class EvaluableT>
template<class EvaluableOther, typename>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(const ExpressionComponent<EvaluableOther>& other) {

    if (other.isValidExpression()) {
        casted_assignement(levi::bool_value<std::is_base_of<EvaluableT, EvaluableOther>::value>(), other);
    }
}

template <class EvaluableT>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(const ExpressionComponent<EvaluableT>& other) {
    m_evaluable = other.m_evaluable;
    if (m_evaluable) {
        m_callerID = m_evaluable->getNewCallerID();
    }
}

template <class EvaluableT>
template<class EvaluableOther, typename>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(ExpressionComponent<EvaluableOther>&& other) {

    if (other.isValidExpression()) {
        casted_assignement(levi::bool_value<std::is_base_of<EvaluableT, EvaluableOther>::value>(), other);
    }
}

template <class EvaluableT>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(ExpressionComponent<EvaluableT>&& other) {
    m_evaluable = other.m_evaluable;
    if (m_evaluable) {
        m_callerID = m_evaluable->getNewCallerID();
    }
}

template <class EvaluableT>
template<class... Args, typename>
levi::ExpressionComponent<EvaluableT>::ExpressionComponent(Args&&... args)
    : m_evaluable(std::make_shared<EvaluableT>(std::forward<Args>(args)...))
{
    m_callerID = m_evaluable->getNewCallerID();
}

template<class EvaluableT>
levi::ExpressionComponent<EvaluableT>::~ExpressionComponent()
{
    if (m_evaluable) {
        m_evaluable->deleteID(m_callerID);
    }
}

template <class EvaluableT>
std::weak_ptr<EvaluableT> levi::ExpressionComponent<EvaluableT>::evaluable() {
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
bool levi::ExpressionComponent<EvaluableT>::isNew() {
    if (!m_evaluable)
        return false;

    return m_evaluable->isNew(m_callerID);
}

template <class EvaluableT>
const typename EvaluableT::matrix_type &levi::ExpressionComponent<EvaluableT>::evaluate() {
    assert(m_evaluable && "This expression is empty.");
    return m_evaluable->evaluateID(m_callerID);
}

template <class EvaluableT>
template<class EvaluableRhs>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> levi::ExpressionComponent<EvaluableT>::operator+(const levi::ExpressionComponent<EvaluableRhs>& rhs) {
    assert(rows() == rhs.rows());
    assert(cols() == rhs.cols());
    assert(m_evaluable && "This expression is empty.");
    assert(rhs.m_evaluable);

    levi::ExpressionComponent<levi::Evaluable<
            typename levi::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

    newExpression = levi::ExpressionComponent<levi::SumEvaluable<EvaluableT, EvaluableRhs>>(*this, rhs);

    return newExpression;
}

template <class EvaluableT>
template <typename Matrix>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> levi::ExpressionComponent<EvaluableT>::operator+(const Matrix& rhs) {
    levi::ExpressionComponent<levi::ConstantEvaluable<Matrix>> constant = levi::build_constant(levi::bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

    return operator+(constant);
}

template <class EvaluableT>
template<class EvaluableRhs>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> levi::ExpressionComponent<EvaluableT>::operator-(const levi::ExpressionComponent<EvaluableRhs>& rhs) {
    assert(rows() == rhs.rows());
    assert(cols() == rhs.cols());
    assert(m_evaluable && "This expression is empty.");
    assert(rhs.m_evaluable);

    levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

    newExpression = levi::ExpressionComponent<levi::SubtractionEvaluable<EvaluableT, EvaluableRhs>>(*this, rhs);

    return newExpression;
}

template <class EvaluableT>
template <typename Matrix>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> levi::ExpressionComponent<EvaluableT>::operator-(const Matrix& rhs) {
    levi::ExpressionComponent<levi::ConstantEvaluable<Matrix>> constant = levi::build_constant(levi::bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

    return operator-(constant);
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::matrix_type>> levi::ExpressionComponent<EvaluableT>::operator-()
{
    assert(m_evaluable && "This expression is empty.");
    levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::matrix_type>> newExpression;

    newExpression = levi::ExpressionComponent<levi::SignInvertedEvaluable<EvaluableT>>(*this);

    return newExpression;
}

template <class EvaluableT>
template<class EvaluableRhs>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> levi::ExpressionComponent<EvaluableT>::operator*(const levi::ExpressionComponent<EvaluableRhs>& rhs) {
    assert((cols() == 1 && rows() == 1) || (rhs.cols() == 1 && rhs.rows() == 1) || (cols() == rhs.rows()) && "Dimension mismatch for product.");
    assert(m_evaluable && "This expression is empty.");
    assert(rhs.m_evaluable);

    levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

    newExpression = levi::ExpressionComponent<levi::ProductEvaluable<EvaluableT, EvaluableRhs>>(*this, rhs);

    return newExpression;
}

template <class EvaluableT>
template <typename Matrix>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, Matrix>::type>> levi::ExpressionComponent<EvaluableT>::operator*(const Matrix& rhs) {
    levi::ExpressionComponent<levi::ConstantEvaluable<Matrix>> constant = levi::build_constant(levi::bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

    return operator*(constant);
}

template <class EvaluableT>
template<class EvaluableRhs>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::value_type>::type>>
levi::ExpressionComponent<EvaluableT>::operator/(const levi::ExpressionComponent<EvaluableRhs> &rhs)
{
    static_assert ((EvaluableRhs::rows_at_compile_time == Eigen::Dynamic || EvaluableRhs::rows_at_compile_time == 1) && (EvaluableRhs::cols_at_compile_time == Eigen::Dynamic || EvaluableRhs::cols_at_compile_time == 1),
                   "The operator/ can be used only when the rhs is a scalar or a 1x1 matrix.");

    assert(rhs.rows() == 1 && rhs.cols() == 1 && "The operator/ can be used only when the rhs is a scalar or a 1x1 matrix.");
    assert(m_evaluable && "This expression is empty.");
    assert(rhs.m_evaluable);

    levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::value_type>::type>> newExpression;

    newExpression = levi::ExpressionComponent<levi::DivisionEvaluable<EvaluableT, EvaluableRhs>>(*this, rhs);

    return newExpression;
}

template <class EvaluableT>
template<typename Scalar>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, Scalar>::type>> levi::ExpressionComponent<EvaluableT>::operator/(const Scalar &rhs)
{
    static_assert (std::is_arithmetic<Scalar>::value, "The rhs has to be a scalar.");
    assert(m_evaluable && "This expression is empty.");

    levi::ExpressionComponent<levi::ConstantEvaluable<Scalar>> constant(rhs);

    return operator/(constant);
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::value_type> > levi::ExpressionComponent<EvaluableT>::pow(typename EvaluableT::value_type exponent)
{
    static_assert ((EvaluableT::rows_at_compile_time == Eigen::Dynamic || EvaluableT::rows_at_compile_time == 1) && (EvaluableT::cols_at_compile_time == Eigen::Dynamic || EvaluableT::cols_at_compile_time == 1),
                   "pow can be used only with scalars or 1x1 matrices.");
    assert(rows() == 1 && cols() == 1 && "pow can be used only with scalars or 1x1 matrices.");
    return ExpressionComponent<levi::PowEvaluable<EvaluableT>>(*this, exponent);
}

template <class EvaluableT>
template<class EvaluableRhs, typename>
void levi::ExpressionComponent<EvaluableT>::operator=(const levi::ExpressionComponent<EvaluableRhs>& rhs) {
    static_assert (!std::is_base_of<levi::VariableBase, EvaluableT>::value, "Cannot assign an expression to a variable." );
    if (rhs.isValidExpression()) {
        casted_assignement(levi::bool_value<std::is_base_of<EvaluableT, EvaluableRhs>::value>(), rhs);
    }
}

template <class EvaluableT>
void levi::ExpressionComponent<EvaluableT>::operator=(const levi::ExpressionComponent<EvaluableT>& rhs) {
    m_evaluable = rhs.m_evaluable;
    if (m_evaluable) {
        m_callerID = m_evaluable->getNewCallerID();
    }
}

template <class EvaluableT>
template<class EvaluableRhs, typename>
void levi::ExpressionComponent<EvaluableT>::operator=(const levi::ExpressionComponent<EvaluableRhs>&& rhs) {
    static_assert (!std::is_base_of<levi::VariableBase, EvaluableT>::value, "Cannot assign an expression to a variable." );
    if (rhs.isValidExpression()) {
        casted_assignement(levi::bool_value<std::is_base_of<EvaluableT, EvaluableRhs>::value>(), rhs);
    }
}

template <class EvaluableT>
void levi::ExpressionComponent<EvaluableT>::operator=(const levi::ExpressionComponent<EvaluableT>&& rhs) {
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
levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::row_type>> levi::ExpressionComponent<EvaluableT>::row(Eigen::Index row) {
    assert(row < this->rows());
    assert(m_evaluable && "Cannot extract a row from this expression");

    levi::ExpressionComponent<levi::RowEvaluable<EvaluableT>> selectedRow(*this, row);
    assert(selectedRow.m_evaluable);

    return selectedRow;
}

template <class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::col_type>> levi::ExpressionComponent<EvaluableT>::col(Eigen::Index col) {
    assert(col < this->cols());
    assert(m_evaluable && "Cannot extract a column from this expression");

    levi::ExpressionComponent<levi::ColEvaluable<EvaluableT>> selectedCol(*this, col);
    assert(selectedCol.m_evaluable);

    return selectedCol;
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::value_type>> levi::ExpressionComponent<EvaluableT>::operator()(Eigen::Index row, Eigen::Index col)
{
    assert(row < this->rows());
    assert(col < this->cols());
    assert(m_evaluable && "Cannot extract an element from this expression");

    levi::ExpressionComponent<levi::ElementEvaluable<EvaluableT>> selectedElement(*this, row, col);
    assert(selectedElement.m_evaluable);

    return selectedElement;
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type>> levi::ExpressionComponent<EvaluableT>::block(Eigen::Index startRow, Eigen::Index startCol, Eigen::Index numberOfRows, Eigen::Index numberOfCols)
{
    assert(m_evaluable && "Cannot extract a block from this expression");
    assert(((startRow + numberOfRows) <= rows()) && ((startCol + numberOfCols) <= cols()) && "Invalid block settings.");

    levi::ExpressionComponent<levi::BlockEvaluable<EvaluableT>> selectedBlock(*this, startRow, startCol, numberOfRows, numberOfCols);

    return selectedBlock;
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> levi::ExpressionComponent<EvaluableT>::skew()
{
    static_assert (((EvaluableT::rows_at_compile_time == Eigen::Dynamic) ||(EvaluableT::rows_at_compile_time == 3)) &&
                   ((EvaluableT::cols_at_compile_time == Eigen::Dynamic) ||(EvaluableT::cols_at_compile_time == 1)) , "Skew can be applied only to three dimensional vectors.");
    assert(m_evaluable && "Cannot compute the skew symmetric matrix from this expression.");
    assert(rows() == 3 && cols() == 1 && "skew can be applied only to three dimensional vectors.");

    return levi::ExpressionComponent<levi::SkewEvaluable<EvaluableT>>(*this);
}

template<class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::transpose_type<EvaluableT>::type> > levi::ExpressionComponent<EvaluableT>::transpose()
{
    assert(m_evaluable && "Cannot compute the transpose from this expression.");

    return levi::ExpressionComponent<levi::TransposeEvaluable<EvaluableT>>(*this);
}


template<class EvaluableT>
levi::ExpressionComponent<levi::EvaluableVariable<typename EvaluableT::col_type> > levi::ExpressionComponent<EvaluableT>::asVariable()
{
    assert(m_evaluable && "The expression is empty.");

    return levi::ExpressionComponent<levi::VariableFromExpressionEvaluable<EvaluableT>>(*this);
}

template<typename EvaluableT>
template<typename VariableType>
levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> levi::ExpressionComponent<EvaluableT>::getColumnDerivative(Eigen::Index column,
                                                                                                                                const levi::ExpressionComponent<levi::EvaluableVariable<VariableType>> &variable)
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(column < cols());
    assert(variable.m_evaluable && "Invalid variable.");

    return m_evaluable->getColumnDerivative(column, variable.m_evaluable);
}

template<typename EvaluableT>
levi::ExpressionComponent<typename EvaluableT::derivative_evaluable> levi::ExpressionComponent<EvaluableT>::getColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable)
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(column < cols());
    assert(variable && "Empty variable pointer.");

    return m_evaluable->getColumnDerivative(column, variable);
}

template<typename EvaluableT>
template<typename VariableType>
bool levi::ExpressionComponent<EvaluableT>::isDependentFrom(const levi::ExpressionComponent<levi::EvaluableVariable<VariableType>> &variable)
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(variable.m_evaluable && "Invalid variable.");

    return m_evaluable->isDependentFrom(variable.m_evaluable);
}

template<typename EvaluableT>
bool levi::ExpressionComponent<EvaluableT>::isDependentFrom(std::shared_ptr<levi::VariableBase> variable)
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
    return levi::ExpressionComponent<levi::ConstructorByRows<EvaluableT>>(rows, name);
}

template<class EvaluableT>
levi::ExpressionComponent<EvaluableT> levi::ExpressionComponent<EvaluableT>::ComposeByCols(const std::vector<levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::col_type>>> &cols, std::string name)
{
    return levi::ExpressionComponent<levi::ConstructorByCols<EvaluableT>>(cols, name);
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
