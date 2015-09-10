// FischerBurmeisterImpact.cpp
//
// Breannan Smith
// Last updated: 09/08/2015

#include "FischerBurmeisterImpact.h"

FischerBurmeisterImpact::FischerBurmeisterImpact( const scalar& tol )
: m_tol( tol )
{}

static scalar fischerBurmeisterInfinityNorm( const VectorXs& x, const VectorXs& y )
{
  assert( x.size() == y.size() );
  scalar inf_norm{ 0.0 };
  for( int i = 0; i < x.size(); ++i )
  {
    const scalar fb{ fabs( sqrt( x(i) * x(i) + y(i) * y(i) ) - x(i) - y(i) ) };
    if( fb > inf_norm )
    {
      inf_norm = fb;
    }
  }
  return inf_norm;
}

scalar FischerBurmeisterImpact::operator()( const VectorXs& alpha, const VectorXs& grad_objective ) const
{
  return fischerBurmeisterInfinityNorm( alpha, grad_objective );
}

FischerBurmeisterImpact::~FischerBurmeisterImpact()
{}

scalar FischerBurmeisterImpact::tol() const
{
  return m_tol;
}
