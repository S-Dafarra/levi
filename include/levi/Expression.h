/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef LEVI_EXPRESSION_COMPONENT_H
#define LEVI_EXPRESSION_COMPONENT_H

#include <levi/ForwardDeclarations.h>

namespace levi {

    template<bool value, typename T>
    static ExpressionComponent<ConstantEvaluable<T>> build_constant(bool_value<value>, const T& rhs);

    /**
    * @brief Return an expression containing a constant evaluable (arithmetic type)
    */
    template<typename T>
    static ExpressionComponent<ConstantEvaluable<T>> build_constant(bool_value<true>, const T& rhs);

    /**
    * @brief Return an expression containing a constant evaluable (matrix type)
    *
    * The name of the constant will be "UnnamedMatrix"
    */
    template<typename T>
    static ExpressionComponent<ConstantEvaluable<T>> build_constant(bool_value<false>, const T& rhs);

}

/**
 *@brief The ExpressionComponent class
 *
 * An ExpressionComponent stores a pointer to an evaluable.
 * This allows to perform operations between evaluable, without carrying around the evaluation buffers.
 */
template <class EvaluableT>
class levi::ExpressionComponent {

    template <class OtherEvaluable>
    friend class ExpressionComponent;

    /**
     * @brief Shared pointer to the evaluable.
     */
    std::shared_ptr<EvaluableT> m_evaluable;

    /**
     * @brief CallerID associated to this expression for the specific evaluable;
     */
    size_t m_callerID;

    /**
     * Template declaration for a helper method used when calling the default constructor of the ExpressionComponent
     */
    template<bool value>
    void default_constructor(levi::bool_value<value>);

    /**
     * Template specialization for the case in which the default constructor is available.
     */
    void default_constructor(levi::bool_value<true>);

    /**
     * Template specialization for the case in which the default constructor is *not* available.
     */
    void default_constructor(levi::bool_value<false>);

    /**
     * Template declaration of a struct used to detect whether the equal operator is available.
     */
    template<class Matrix, typename = int>
    struct has_equal_to_constant_operator : std::false_type { };

    /**
     * Template specialization of a struct used to detect whether the equal operator is available.
     * See https://stackoverflow.com/questions/1005476/how-to-detect-whether-there-is-a-specific-member-variable-in-class
     */
    template<class Matrix>
    struct has_equal_to_constant_operator<Matrix, decltype(std::declval<EvaluableT>().operator=(std::declval<Matrix>()), 0)> : std::true_type { };

    /**
     * Template declaration of a function used to perform casting when during assignements.
     */
    template<bool value, typename OtherEvaluable>
    void casted_assignement(levi::bool_value<value>, const ExpressionComponent<OtherEvaluable>& other);

    /**
     * Template specialization in case the two pointers are directly castable.
     */
    template<typename OtherEvaluable>
    void casted_assignement(levi::bool_value<true>, const levi::ExpressionComponent<OtherEvaluable>& other);

    /**
     * Template specialization in case the two pointers are not directly castable.
     */
    template<typename OtherEvaluable>
    void casted_assignement(levi::bool_value<false>, const levi::ExpressionComponent<OtherEvaluable>& other);

public:

    /**
     * @brief Default Constructor
     *
     * If the evaluable EvaluableT is not default constructible, then the shared pointer is equal to nullptr.
     */
    ExpressionComponent();

    /**
     * @brief Copy constructor
     */
    template<class EvaluableOther>
    ExpressionComponent(const ExpressionComponent<EvaluableOther>& other);

    ExpressionComponent(const ExpressionComponent<EvaluableT>& other);

    /**
     * @brief Move constructor
     */
    template<class EvaluableOther>
    ExpressionComponent(ExpressionComponent<EvaluableOther>&& other);

    ExpressionComponent(ExpressionComponent<EvaluableT>&& other);

    /**
     * @brief Constructor
     *
     * Use this constructor to instantiate all the custom evaluables.
     * All the input arguments will be passed to the constructor of the Evaluable pointed inside the ExpressionComponent.
     */
    template<class... Args, typename = typename std::enable_if<std::is_constructible<EvaluableT, Args...>::value>::type>
    ExpressionComponent(Args&&... args);

    /**
     * @brief Weak pointer to the evaluable
     *
     * Useful in case the evaluable has to be accessed after the allocation.
     *
     * @return a weak pointer to the evaluable
     */
    std::weak_ptr<EvaluableT> evaluable();

    /**
     * @brief Name of the expression
     *
     * Corresponds to the name of the pointed evaluable.
     *
     * @return The name of the expression
     */
    std::string name() const;

    /**
     * @brief Rows of the expression
     *
     * Corresponds to the number of rows of the pointed evaluable.
     *
     * @return The number of rows of the expression
     */
    Eigen::Index rows() const;

    /**
     * @brief Cols of the expression
     *
     * Corresponds to the number of cols of the pointed evaluable.
     *
     * @return The number of cols of the expression
     */
    Eigen::Index cols() const;

    /**
     * @brief Check if the expression has a new value
     * @return True if new
     */
    bool isNew();

    /**
     * @brief Evaluate the pointed evaluable.
     *
     * @warning An assert is used to check whether the evaluable can be evaluated. Test the code in debug mode first to avoid segfaults.
     *
     * @return A const reference to the evaluation buffer of the pointed evaluable.
     */
    const typename EvaluableT::matrix_type& evaluate();

    /**
     * @brief Operator +
     * @return An expression which points to an evaluable performing the additions.
     */
    template<class EvaluableRhs>
    ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator+(const ExpressionComponent<EvaluableRhs>& rhs);

    /**
     * @brief Operator +
     *
     * The other addend will be inserted in a ConstantEvaluable containing the rhs.
     *
     * @return An expression which points to an evaluable performing the addition.
     */
    template <typename Matrix>
    ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator+(const Matrix& rhs);

    /**
     * @brief Operator -
     *
     * @return An expression which points to an evaluable performing the subtraction.
     */
    template<class EvaluableRhs>
    ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator-(const ExpressionComponent<EvaluableRhs>& rhs);

    /**
     * @brief Operator -
     *
     * The other addend will be inserted in a ConstantEvaluable containing the rhs.
     *
     * @return An expression which points to an evaluable performing the subtraction.
     */
    template <typename Matrix>
    ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator-(const Matrix& rhs);

    /**
     * @brief Operator -
     *
     * @return An expression which points to an evaluable inverting the sign.
     */
    ExpressionComponent<levi::Evaluable<typename EvaluableT::matrix_type>> operator-();

    /**
     * @brief Operator *
     *
     * @return An expression which points to an evaluable performing the multiplication.
     */
    template<class EvaluableRhs>
    ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::matrix_type>::type>> operator*(const ExpressionComponent<EvaluableRhs>& rhs);

    /**
     * @brief Operator *
     *
     * The other addend will be inserted in a ConstantEvaluable containing the rhs.
     *
     * @return An expression which points to an evaluable performing the multiplication.
     */
    template <typename Matrix>
    ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, Matrix>::type>> operator*(const Matrix& rhs);

    /**
     * @brief Operator /
     *
     * @return An expression which points to an evaluable performing the division by a scalar.
     */
    template<class EvaluableRhs>
    ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, typename EvaluableRhs::value_type>::type>> operator/(const ExpressionComponent<EvaluableRhs>& rhs);

    /**
     * @brief Operator /
     *
     * The other addend will be inserted in a ConstantEvaluable containing the rhs.
     *
     * @return An expression which points to an evaluable performing the division.
     */
    template <typename Scalar>
    ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<typename EvaluableT::matrix_type, Scalar>::type>> operator/(const Scalar& rhs);

    /**
     * @brief Computes the power of the current evaluable
     * @param exponent The exponent of the power
     * @return An expression computing the power
     *
     * @note This can be used only with scalars or 1x1 matrices.
     */
    ExpressionComponent<levi::Evaluable<typename EvaluableT::value_type>> pow(typename EvaluableT::value_type exponent);

    /**
     * @brief Assignement operator
     *
     * Assigns the current expression to the rhs. If the EvaluableT is not a base class for EvaluableRhs,
     * a new evaluable will be created which will cast the evaluation buffers.
     *
     * @note This changes the pointer and does not affect the pointed evaluable. If no other expression points to the previous evaluable,
     * it will be deleted.
     */
    template<class EvaluableRhs>
    void operator=(const ExpressionComponent<EvaluableRhs>& rhs);

    void operator=(const ExpressionComponent<EvaluableT>& rhs);

    /**
     * @brief Move assignement operator
     *
     * Assigns the current expression to the rhs. If the EvaluableT is not a base class for EvaluableRhs,
     * a new evaluable will be created which will cast the evaluation buffers.
     */
    template<class EvaluableRhs>
    void operator=(const ExpressionComponent<EvaluableRhs>&& rhs);

    void operator=(const ExpressionComponent<EvaluableT>&& rhs);

    /**
     * @brief Assignement operator to a matrix
     *
     * Assigns the evaluable pointed by the current expression to the rhs. This method can be used only if the operator= is defined
     * in the pointed evaluable for the Matrix type.
     */
    template<typename Matrix>
    void operator=(const Matrix& rhs);

    /**
     * @brief Returns an expression corresponding to a row of the current evaluable
     * @param row The index of the row of interest
     * @return An expression pointing to a row of the current evaluable.
     *
     * @Note This has to be considered as read only accessor.
     */
    ExpressionComponent<levi::Evaluable<typename EvaluableT::row_type>> row(Eigen::Index row);

    /**
     * @brief Returns an expression corresponding to a column of the current evaluable
     * @param col The index of the column of interest
     * @return An expression pointing to a column of the current evaluable.
     *
     * @Note This has to be considered as read only accessor.
     */
    ExpressionComponent<levi::Evaluable<typename EvaluableT::col_type>> col(Eigen::Index col);

    /**
     * @brief Accessor to element (read only)
     * @param row The index of the row of interest
     * @param col The index of the column of interest
     * @return An expression pointing to an element of the current evaluable.
     *
     * @Note This has to be considered as read only accessor.
     *
     */
    ExpressionComponent<levi::Evaluable<typename EvaluableT::value_type>> operator()(Eigen::Index row, Eigen::Index col);

    /**
     * @brief Accessor to element (read only)
     * @param startRow The starting row to be considered in the block
     * @param startCol The starting column to be considered in the block
     * @param numberOfRows The number of rows of the block
     * @param numberOfCols The number of columns of the block
     * @return An expression pointing to an evaluable taking only the specified block out of the current evaluable.
     */
    ExpressionComponent<levi::Evaluable<typename levi::dynamic_block_return<typename EvaluableT::matrix_type>::type>> block(Eigen::Index startRow, Eigen::Index startCol, Eigen::Index numberOfRows, Eigen::Index numberOfCols);

    /**
     * @brief Computes the skew symmetric matrix out of the current evaluable
     * @return An expression pointing to an evaluable which computes the skew symmetric matrix
     * @note This works only with three dimensional vectors.
     */
    ExpressionComponent<levi::Evaluable<Eigen::Matrix<typename EvaluableT::value_type, 3, 3>>> skew();


    /**
     * @brief Computes the transpose of the current expression
     * @return An expression whose evaluable computes the transpose of the current evaluable.
     *
     * @note The column derivative needs to call the derivative of every column. This may be expensive.
     */
    ExpressionComponent<levi::Evaluable<typename levi::transpose_type<EvaluableT>::type>> transpose();

    /**
     * @brief Retrieve the column derivative with respect to the specified variable
     *
     * @param column The column of interest
     * @param variable Expression pointing to the variable of interest
     * @return An expression pointing to the column derivative.
     */
    template<typename VariableType>
    ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column, const ExpressionComponent<levi::EvaluableVariable<VariableType>>& variable);

    /**
     * @brief Retrieve the column derivative with respect to the specified variable
     *
     * @param column The column of interest
     * @param variable Shared pointer to the variable of interest
     * @return An expression pointing to the column derivative.
     */
    ExpressionComponent<typename EvaluableT::derivative_evaluable> getColumnDerivative(Eigen::Index column, std::shared_ptr<levi::VariableBase> variable);

    /**
     * @brief Check whether the pointed evaluable depends on a specified variable
     * @param variable The variable of interest
     * @return True if dependent
     */
    template<typename VariableType>
    bool isDependentFrom(const ExpressionComponent<levi::EvaluableVariable<VariableType>>& variable);

    /**
     * @brief Check whether the pointed evaluable depends on a specified variable
     * @param variable The variable of interest
     * @return True if dependent
     */
    bool isDependentFrom(std::shared_ptr<levi::VariableBase> variable);

    /**
     * @brief Create an epression from a vector of rows
     * @param rows The vector containing the expressionion composing the rows of the new expression
     * @param name The name of the new expression
     * @return An expression made of the specified rows
     */
    static ExpressionComponent<EvaluableT> ComposeByRows(const std::vector<levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::row_type>>>& rows, std::string name);

    /**
     * @brief Create an epression from a vector of cols
     * @param cols The vector containing the expressionion composing the columns of the new expression
     * @param name The name of the new expression
     * @return An expression made of the specified columns
     */
    static ExpressionComponent<EvaluableT> ComposeByCols(const std::vector<levi::ExpressionComponent<levi::Evaluable<typename EvaluableT::col_type>>>& cols, std::string name);
};


/**
 * @brief Operator +
 *
 * The other addend will be inserted in a ConstantEvaluable containing the lhs.
 *
 * @return An expression which points to an evaluable performing the addition.
 */
template <typename Matrix, class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator+(const Matrix& lhs, const levi::ExpressionComponent<EvaluableT> &rhs);

/**
 * @brief Operator -
 *
 * The other addend will be inserted in a ConstantEvaluable containing the lhs.
 *
 * @return An expression which points to an evaluable performing the subrraction.
 */
template <typename Matrix, class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_sum_return<typename EvaluableT::matrix_type, Matrix>::type>> operator-(const Matrix& lhs, const levi::ExpressionComponent<EvaluableT> &rhs);

/**
 * @brief Operator *
 *
 * The other addend will be inserted in a ConstantEvaluable containing the lhs.
 *
 * @return An expression which points to an evaluable performing the multiplication.
 */
template <typename Matrix, class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<Matrix, typename EvaluableT::matrix_type>::type>> operator*(const Matrix& lhs, const levi::ExpressionComponent<EvaluableT> &rhs);

/**
 * @brief Operator /
 *
 * The other addend will be inserted in a ConstantEvaluable containing the lhs.
 *
 * @return An expression which points to an evaluable performing the division.
 */
template <typename Matrix, class EvaluableT>
levi::ExpressionComponent<levi::Evaluable<typename levi::matrix_product_return<Matrix, typename EvaluableT::value_type>::type>> operator/(const Matrix& lhs, const levi::ExpressionComponent<EvaluableT> &rhs);


#endif // LEVI_EXPRESSION_COMPONENT_H
