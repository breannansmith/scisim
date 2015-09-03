// QPTerminationOperator.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef QP_TERMINATION_OPERATOR_H
#define QP_TERMINATION_OPERATOR_H

#include "SCISim/Math/MathDefines.h"

class QPTerminationOperator
{

public:

  virtual ~QPTerminationOperator() = 0;
  // TODO: Eliminate the PermutationMatrix related parameters
  virtual scalar operator()( const VectorXs& x, const VectorXs& y, const bool permutation_required = false, const PermutationMatrix& permutation = PermutationMatrix() ) const = 0;
  virtual scalar tol() const = 0;

};

#endif
