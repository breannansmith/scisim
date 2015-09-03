// FrictionOperatorUtilities.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef FRICTION_OPERATOR_UTILITIES_H
#define FRICTION_OPERATOR_UTILITIES_H

#include "SCISim/Math/MathDefines.h"

#include <memory>

class Constraint;

namespace FrictionOperatorUtilities
{

  void formGeneralizedFrictionBasis( const VectorXs& q0, const VectorXs& v0, const std::vector<std::unique_ptr<Constraint>>& K, const int num_samples, SparseMatrixsc& D, VectorXs& drel );

  void formLinearFrictionDiskConstraint( const int num_samples, SparseMatrixsc& E );

  void computeMDPLambda( const VectorXs& vrel, VectorXs& lambda );

  void projectOnFrictionDisc( const VectorXs& disc_bounds, VectorXs& beta );

  // TODO: Have code that forms the linear term in MDP

}

#endif
