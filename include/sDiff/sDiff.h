/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_H
#define SDIFF_H

#include "Expression.h"
#include "Evaluable.h"
#include "Variable.h"

namespace sDiff {
    typedef ExpressionElement<Evaluable<Eigen::MatrixXd>> Expression;
}

#endif // SDIFF_H
