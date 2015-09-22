// FrictionOperatorUtilities.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef FRICTION_OPERATOR_UTILITIES_H
#define FRICTION_OPERATOR_UTILITIES_H

#include "scisim/Math/MathDefines.h"

#include <memory>

class Constraint;

namespace FrictionOperatorUtilities
{

  void formGeneralizedFrictionBasis( const VectorXs& q0, const VectorXs& v0, const std::vector<std::unique_ptr<Constraint>>& K, const int num_samples, SparseMatrixsc& D, VectorXs& drel );

  void formLinearFrictionDiskConstraint( const int num_samples, SparseMatrixsc& E );

  // TODO: Have code that forms the linear term in MDP

}

#endif
