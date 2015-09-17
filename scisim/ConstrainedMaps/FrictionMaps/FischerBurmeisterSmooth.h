// FischerBurmeisterSmooth.h
//
// Breannan Smith
// Last updated: 09/08/2015

#ifndef FISCHER_BURMEISTER_SMOOTH_H
#define FISCHER_BURMEISTER_SMOOTH_H

#include "scisim/ConstrainedMaps/QPTerminationOperator.h"
#include "scisim/Math/MathDefines.h"

class FischerBurmeisterSmooth final : public QPTerminationOperator
{

public:

  // c == \mu \alpha
  FischerBurmeisterSmooth( const scalar& tol, const VectorXs& c );

  virtual ~FischerBurmeisterSmooth() override;

  virtual scalar operator()( const VectorXs& beta, const VectorXs& vrel ) const override;

  virtual scalar tol() const override;

private:

  const scalar m_tol;
  const VectorXs& m_c;
  
};


#endif
