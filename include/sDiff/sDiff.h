/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_H
#define SDIFF_H

#include "ExpressionElement.h"
#include "Evaluable.h"
#include "EvaluableVariable.h"

namespace sDiff {
    typedef ExpressionElement<Evaluable<Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic >>> Expression;

    typedef ExpressionElement<EvaluableVariable<Eigen::Matrix< double , Eigen::Dynamic , 1>>> Variable;

    typedef ExpressionElement<ConstantEvaluable<Eigen::Matrix< double , Eigen::Dynamic , Eigen::Dynamic>>> Constant;

    typedef ExpressionElement<ConstantEvaluable<double>> Scalar;
}

#endif // SDIFF_H
