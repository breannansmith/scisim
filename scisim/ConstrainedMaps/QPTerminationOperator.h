// QPTerminationOperator.h
//
// Breannan Smith
// Last updated: 09/08/2015

#ifndef QP_TERMINATION_OPERATOR_H
#define QP_TERMINATION_OPERATOR_H

#include "scisim/Math/MathDefines.h"

class QPTerminationOperator
{

public:

  QPTerminationOperator( const QPTerminationOperator& ) = delete;
  QPTerminationOperator( QPTerminationOperator&& ) = delete;
  QPTerminationOperator& operator=( const QPTerminationOperator& ) = delete;
  QPTerminationOperator& operator=( QPTerminationOperator&& ) = delete;

  virtual ~QPTerminationOperator() = 0;
  virtual scalar operator()( const VectorXs& x, const VectorXs& y ) const = 0;
  virtual scalar tol() const = 0;

protected:

  QPTerminationOperator() = default;

};

#endif
