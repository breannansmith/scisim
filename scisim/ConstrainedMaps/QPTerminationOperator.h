// QPTerminationOperator.h
//
// Breannan Smith
// Last updated: 09/08/2015

#ifndef QP_TERMINATION_OPERATOR_H
#define QP_TERMINATION_OPERATOR_H

#include "SCISim/Math/MathDefines.h"

class QPTerminationOperator
{

public:

  virtual ~QPTerminationOperator() = 0;
  virtual scalar operator()( const VectorXs& x, const VectorXs& y ) const = 0;
  virtual scalar tol() const = 0;

};

#endif
