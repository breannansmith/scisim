// FischerBurmeisterImpact.h
//
// Breannan Smith
// Last updated: 09/08/2015

#ifndef FISCHER_BURMEISTER_IMPACT_H
#define FISCHER_BURMEISTER_IMPACT_H

#include "SCISim/ConstrainedMaps/QPTerminationOperator.h"
#include "SCISim/Math/MathDefines.h"

class FischerBurmeisterImpact final : public QPTerminationOperator
{

public:

  FischerBurmeisterImpact( const scalar& tol );
  virtual ~FischerBurmeisterImpact() override;

  virtual scalar operator()( const VectorXs& alpha, const VectorXs& grad_objective ) const override;

  virtual scalar tol() const override;

private:

  const scalar m_tol;
  
};

#endif
