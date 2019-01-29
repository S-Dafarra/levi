/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 * Authors: Stefano Dafarra
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <sDiff/sDiff.h>
#include <iostream>


int main() {
    using namespace sDiff;

    Identity identity(3,3);

    Expression antiIdentity = Expression::ComposeByRows({identity.row(2), identity.row(1), identity.row(0)}, "Anti-identity");

    std::cerr << antiIdentity.evaluate() << std::endl;

    Variable x(2, "x");
    auto derivative = antiIdentity.getColumnDerivative(1, x);

    return 0;

}
