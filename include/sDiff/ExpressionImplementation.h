/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_EXPRESSIONIMPLEMENTATION_H
#define SDIFF_EXPRESSIONIMPLEMENTATION_H

#include <sDiff/Expression.h>
#include <sDiff/OperatorsEvaluables.h>
#include <sDiff/Variable.h>

template<bool value, typename T>
static sDiff::ExpressionComponent<sDiff::ConstantEvaluable<T>> sDiff::build_constant(sDiff::bool_value<value>, const T& rhs) { }

template<typename T>
static sDiff::ExpressionComponent<sDiff::ConstantEvaluable<T>> sDiff::build_constant(sDiff::bool_value<true>, const T& rhs) {
    sDiff::ExpressionComponent<ConstantEvaluable<T>> constant(rhs);
    return constant;
}

template<typename T>
static sDiff::ExpressionComponent<sDiff::ConstantEvaluable<T>> sDiff::build_constant(sDiff::bool_value<false>, const T& rhs) {
    sDiff::ExpressionComponent<sDiff::ConstantEvaluable<T>> constant(rhs, "UnnamedMatrix");
    return constant;
}

template <class EvaluableT>
template<bool value>
void sDiff::ExpressionComponent<EvaluableT>::default_constructor(sDiff::bool_value<value>)
{ }

template <class EvaluableT>
void sDiff::ExpressionComponent<EvaluableT>::default_constructor(sDiff::bool_value<true>) {
    m_evaluable = std::make_shared<EvaluableT>();
}

template <class EvaluableT>
void sDiff::ExpressionComponent<EvaluableT>::default_constructor(sDiff::bool_value<false>) {
    m_evaluable = nullptr;
}

template <class EvaluableT>
template<bool value, typename OtherEvaluable>
void sDiff::ExpressionComponent<EvaluableT>::casted_assignement(sDiff::bool_value<value>, const sDiff::ExpressionComponent<OtherEvaluable>& other) {}

template <class EvaluableT>
template<typename OtherEvaluable>
void sDiff::ExpressionComponent<EvaluableT>::casted_assignement(sDiff::bool_value<true>, const sDiff::ExpressionComponent<OtherEvaluable>& other) {
    m_evaluable = other.m_evaluable;
}

template <class EvaluableT>
template<typename OtherEvaluable>
void sDiff::ExpressionComponent<EvaluableT>::casted_assignement(sDiff::bool_value<false>, const sDiff::ExpressionComponent<OtherEvaluable>& other) {
    m_evaluable = std::make_shared<sDiff::CastEvaluable<EvaluableT, OtherEvaluable>>(other);
}

template <class EvaluableT>
sDiff::ExpressionComponent<EvaluableT>::ExpressionComponent()
{
    default_constructor(sDiff::bool_value<std::is_constructible<EvaluableT>::value>());
}

template <class EvaluableT>
template<class EvaluableOther>
sDiff::ExpressionComponent<EvaluableT>::ExpressionComponent(const ExpressionComponent<EvaluableOther>& other) {
    this = other;
}

template <class EvaluableT>
template<class EvaluableOther>
sDiff::ExpressionComponent<EvaluableT>::ExpressionComponent(ExpressionComponent<EvaluableOther>&& other) {
    *this = other;
}

template <class EvaluableT>
template<class... Args >
sDiff::ExpressionComponent<EvaluableT>::ExpressionComponent(Args&&... args)
    : m_evaluable(std::make_shared<EvaluableT>(args...))
{ }

template <class EvaluableT>
std::weak_ptr<EvaluableT> sDiff::ExpressionComponent<EvaluableT>::evaluable() {
    return m_evaluable;
}

template <class EvaluableT>
std::string sDiff::ExpressionComponent<EvaluableT>::name() const {
    assert(m_evaluable);
    return m_evaluable->name();
}

template <class EvaluableT>
Eigen::Index sDiff::ExpressionComponent<EvaluableT>::rows() const {
    assert(m_evaluable);
    return m_evaluable->rows();
}

template <class EvaluableT>
Eigen::Index sDiff::ExpressionComponent<EvaluableT>::cols() const {
    assert(m_evaluable);
    return m_evaluable->cols();
}

template <class EvaluableT>
const typename EvaluableT::matrix_type &sDiff::ExpressionComponent<EvaluableT>::evaluate() {
    assert(m_evaluable);
    return m_evaluable->evaluate();
}

template <class EvaluableT>
template<class EvaluableRhs>
sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> sDiff::ExpressionComponent<EvaluableT>::operator+(const sDiff::ExpressionComponent<EvaluableRhs>& rhs) {
    assert(rows() == rhs.rows());
    assert(cols() == rhs.cols());
    assert(m_evaluable);
    assert(rhs.m_evaluable);

    sDiff::ExpressionComponent<sDiff::Evaluable<
            typename sDiff::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

    newExpression = sDiff::ExpressionComponent<sDiff::SumEvaluable<EvaluableT, EvaluableRhs>>(*this, rhs);

    return newExpression;
}

template <class EvaluableT>
template <typename Matrix>
sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> sDiff::ExpressionComponent<EvaluableT>::operator+(const Matrix& rhs) {
    sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Matrix>> constant = sDiff::build_constant(sDiff::bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

    return operator+(constant);
}

template <class EvaluableT>
template<class EvaluableRhs>
sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> sDiff::ExpressionComponent<EvaluableT>::operator-(const sDiff::ExpressionComponent<EvaluableRhs>& rhs) {
    assert(rows() == rhs.rows());
    assert(cols() == rhs.cols());
    assert(m_evaluable);
    assert(rhs.m_evaluable);

    sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

    newExpression = sDiff::ExpressionComponent<sDiff::SubtractionEvaluable<EvaluableT, EvaluableRhs>>(*this, rhs);

    return newExpression;
}

template <class EvaluableT>
template <typename Matrix>
sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> sDiff::ExpressionComponent<EvaluableT>::operator-(const Matrix& rhs) {
    sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Matrix>> constant = sDiff::build_constant(sDiff::bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

    return operator-(constant);
}

template<class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename EvaluableT::matrix_type>> sDiff::ExpressionComponent<EvaluableT>::operator-()
{
    assert(m_evaluable);
    sDiff::ExpressionComponent<sDiff::Evaluable<typename EvaluableT::matrix_type>> newExpression;

    newExpression = sDiff::ExpressionComponent<sDiff::SignInvertedEvaluable<EvaluableT>>(*this);

    return newExpression;
}

template <class EvaluableT>
template<class EvaluableRhs>
sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> sDiff::ExpressionComponent<EvaluableT>::operator*(const sDiff::ExpressionComponent<EvaluableRhs>& rhs) {
    assert((cols() == 1 && rows() == 1) || (rhs.cols() == 1 && rhs.rows() == 1) || (cols() == rhs.rows()) && "Dimension mismatch for product.");
    assert(m_evaluable);
    assert(rhs.m_evaluable);

    sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> newExpression;

    newExpression = sDiff::ExpressionComponent<sDiff::ProductEvaluable<EvaluableT, EvaluableRhs>>(*this, rhs);

    return newExpression;
}

template <class EvaluableT>
template <typename Matrix>
sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_product_return<typename EvaluableT::matrix_type, Matrix>::type>> sDiff::ExpressionComponent<EvaluableT>::operator*(const Matrix& rhs) {
    sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Matrix>> constant = sDiff::build_constant(sDiff::bool_value<std::is_arithmetic<Matrix>::value>(), rhs);

    return operator*(constant);
}

template <class EvaluableT>
template<class EvaluableRhs>
sDiff::ExpressionComponent<EvaluableT>& sDiff::ExpressionComponent<EvaluableT>::operator=(const sDiff::ExpressionComponent<EvaluableRhs>& rhs) {
    static_assert (!std::is_base_of<sDiff::EvaluableVariable<typename EvaluableT::matrix_type>, EvaluableT>::value, "Cannot assign an expression to a variable." );
    casted_assignement(sDiff::bool_value<std::is_base_of<EvaluableT, EvaluableRhs>::value>(), rhs);
    return *this;
}

template <class EvaluableT>
template<class EvaluableRhs>
sDiff::ExpressionComponent<EvaluableT>& sDiff::ExpressionComponent<EvaluableT>::operator=(const sDiff::ExpressionComponent<EvaluableRhs>&& rhs) {
    static_assert (!std::is_base_of<sDiff::EvaluableVariable<typename EvaluableT::matrix_type>, EvaluableT>::value, "Cannot assign an expression to a variable." );
    casted_assignement(sDiff::bool_value<std::is_base_of<EvaluableT, EvaluableRhs>::value>(), rhs);
    return *this;
}

//assign from a constant
template <class EvaluableT>
template<typename Matrix>
void sDiff::ExpressionComponent<EvaluableT>::operator=(const Matrix& rhs) {
    static_assert (has_equal_to_constant_operator<Matrix>(), "This expression cannot be set equal to a constant.");
    assert(m_evaluable && "This expression cannot be set because the constructor was not called properly.");
    (*m_evaluable) = rhs;
}

template <class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename EvaluableT::row_type>> sDiff::ExpressionComponent<EvaluableT>::row(Eigen::Index row) {
    assert(row < this->rows());
    assert(m_evaluable && "Cannot extract a row from this expression");

    sDiff::ExpressionComponent<sDiff::RowEvaluable<EvaluableT>> selectedRow(*this, row);
    assert(selectedRow.m_evaluable);

    return selectedRow;
}

template <class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename EvaluableT::col_type>> sDiff::ExpressionComponent<EvaluableT>::col(Eigen::Index col) {
    assert(col < this->cols());
    assert(m_evaluable && "Cannot extract a column from this expression");

    sDiff::ExpressionComponent<sDiff::ColEvaluable<EvaluableT>> selectedCol(*this, col);
    assert(selectedCol.m_evaluable);

    return selectedCol;
}

template<class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename EvaluableT::value_type>> sDiff::ExpressionComponent<EvaluableT>::operator()(Eigen::Index row, Eigen::Index col)
{
    assert(row < this->rows());
    assert(col < this->cols());
    assert(m_evaluable && "Cannot extract an element from this expression");

    sDiff::ExpressionComponent<sDiff::ElementEvaluable<EvaluableT>> selectedElement(*this, row, col);
    assert(selectedElement.m_evaluable);

    return selectedElement;
}

template<class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::dynamic_block_return<typename EvaluableT::matrix_type>::type>> sDiff::ExpressionComponent<EvaluableT>::block(Eigen::Index startRow, Eigen::Index startCol, Eigen::Index numberOfRows, Eigen::Index numberOfCols)
{
    assert(m_evaluable && "Cannot extract a block from this expression");
    assert(((startRow + numberOfRows) <= rows()) && ((startCol + numberOfCols) <= cols()) && "Invalid block settings.");

    sDiff::ExpressionComponent<sDiff::BlockEvaluable<EvaluableT>> selectedBlock(*this, startRow, startCol, numberOfRows, numberOfCols);

    return selectedBlock;
}

template<class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> sDiff::ExpressionComponent<EvaluableT>::skew()
{
    static_assert (((EvaluableT::rows_at_compile_time == Eigen::Dynamic) ||(EvaluableT::rows_at_compile_time == 3)) &&
                   ((EvaluableT::cols_at_compile_time == Eigen::Dynamic) ||(EvaluableT::cols_at_compile_time == 1)) , "Skew can be applied only to three dimensional vectors.");
    assert(m_evaluable && "Cannot compute the skew symmetric matrix from this expression.");
    assert(rows() == 3 && cols() == 1 && "skew can be applied only to three dimensional vectors.");

    return sDiff::ExpressionComponent<sDiff::SkewEvaluable<EvaluableT>>(*this);
}

template<typename EvaluableT>
template<typename VariableType>
sDiff::ExpressionComponent<typename EvaluableT::derivative_evaluable> sDiff::ExpressionComponent<EvaluableT>::getColumnDerivative(Eigen::Index column,
                                                                                                                                  const sDiff::ExpressionComponent<sDiff::EvaluableVariable<VariableType>> &variable)
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(column < cols());
    assert(variable.m_evaluable && "Invalid variable.");

    return m_evaluable->getColumnDerivative(column, variable.m_evaluable);
}

template<typename EvaluableT>
sDiff::ExpressionComponent<typename EvaluableT::derivative_evaluable> sDiff::ExpressionComponent<EvaluableT>::getColumnDerivative(Eigen::Index column, std::shared_ptr<sDiff::VariableBase> variable)
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(column < cols());
    assert(variable && "Empty variable pointer.");

    return m_evaluable->getColumnDerivative(column, variable);
}

template<typename EvaluableT>
template<typename VariableType>
bool sDiff::ExpressionComponent<EvaluableT>::isDependentFrom(const sDiff::ExpressionComponent<sDiff::EvaluableVariable<VariableType>> &variable)
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(variable.m_evaluable && "Invalid variable.");

    return m_evaluable->isDependentFrom(variable.m_evaluable);
}

template<typename EvaluableT>
bool sDiff::ExpressionComponent<EvaluableT>::isDependentFrom(std::shared_ptr<sDiff::VariableBase> variable)
{
    assert(m_evaluable && "Cannot compute the derivative of this expression.");
    assert(variable && "Empty variable pointer.");

    return m_evaluable->isDependentFrom(variable);
}

//end of ExpressionComponent implementation

template <typename Matrix, class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator+(const Matrix& lhs, const sDiff::ExpressionComponent<EvaluableT> &rhs) {

    sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Matrix>> constant =
            sDiff::build_constant(sDiff::bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return constant + rhs;
}

template <typename Matrix, class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator-(const Matrix& lhs, const sDiff::ExpressionComponent<EvaluableT> &rhs) {

    sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Matrix>> constant =
            sDiff::build_constant(sDiff::bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return constant - rhs;
}

template <typename Matrix, class EvaluableT>
sDiff::ExpressionComponent<sDiff::Evaluable<typename sDiff::matrix_product_return<Matrix, typename EvaluableT::matrix_type>::type>> operator*(const Matrix& lhs, const sDiff::ExpressionComponent<EvaluableT> &rhs) {

    sDiff::ExpressionComponent<sDiff::ConstantEvaluable<Matrix>> newConstant =
            sDiff::build_constant<Matrix>(sDiff::bool_value<std::is_arithmetic<Matrix>::value>(), lhs);

    return newConstant * rhs;
}

#endif // SDIFF_EXPRESSIONIMPLEMENTATION_H
