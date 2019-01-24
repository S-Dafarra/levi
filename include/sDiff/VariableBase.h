/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */
#ifndef SDIFF_VARIABLEBASE_H
#define SDIFF_VARIABLEBASE_H

#include <sDiff/ForwardDeclarations.h>

class sDiff::VariableBase {

protected:
    std::string m_name;

    Eigen::Index m_dimension;

    VariableBase(Eigen::Index dimension, const std::string& name)
        : m_name(name)
        , m_dimension(dimension)
    { }

public:

    Eigen::Index dimension() const{
        return m_dimension;
    }

    std::string variableName() const {
        return m_name;
    }

};

#endif // SDIFF_VARIABLEBASE_H
