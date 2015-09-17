// FischerBurmeisterBoundConstrained.h
//
// Breannan Smith
// Last updated: 09/08/2015

#ifndef FISCHER_BURMEISTER_BOUND_CONSTRAINED_H
#define FISCHER_BURMEISTER_BOUND_CONSTRAINED_H

#include "scisim/ConstrainedMaps/QPTerminationOperator.h"
#include "scisim/Math/MathDefines.h"

class FischerBurmeisterBoundConstrained final : public QPTerminationOperator
{

public:

  // c == \mu \alpha
  FischerBurmeisterBoundConstrained( const scalar& tol, const VectorXs& c );
  virtual ~FischerBurmeisterBoundConstrained() override = default;

  virtual scalar operator()( const VectorXs& beta, const VectorXs& vrel ) const override;

  virtual scalar tol() const override;

private:

  const scalar m_tol;
  const VectorXs& m_c;
  
};

#endif
