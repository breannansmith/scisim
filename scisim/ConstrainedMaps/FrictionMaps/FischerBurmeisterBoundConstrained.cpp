// FischerBurmeisterBoundConstrained.cpp
//
// Breannan Smith
// Last updated: 09/08/2015

#include "FischerBurmeisterBoundConstrained.h"

FischerBurmeisterBoundConstrained::FischerBurmeisterBoundConstrained( const scalar& tol, const VectorXs& c )
: m_tol( tol )
, m_c( c )
{
  assert( m_tol >= 0.0 );
  assert( ( m_c.array() >= 0.0 ).all() );
}

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

scalar FischerBurmeisterBoundConstrained::operator()( const VectorXs& beta, const VectorXs& vrel ) const
{
  assert( beta.size() == vrel.size() ); assert( beta.size() == m_c.size() );

  // Only project velocities (-vrel) pointing outward
  VectorXs lambda{ vrel };
  for( int i = 0; i < lambda.size(); ++i )
  {
    if( m_c( i ) == 0.0 )
    {
      lambda( i ) = 0.0;
    }
    else if( beta( i ) <= - m_c( i ) )
    {
      lambda( i ) = std::min<scalar>( lambda( i ), 0.0 );
    }
    else if( beta( i ) >= m_c( i ) )
    {
      lambda( i ) = std::max<scalar>( lambda( i ), 0.0 );
    }
  }

  // Compute the constraint violation
  const VectorXs constraint_violation{ m_c - beta.cwiseAbs() };
  return fischerBurmeisterInfinityNorm( lambda, constraint_violation );
}

scalar FischerBurmeisterBoundConstrained::tol() const
{
  return m_tol;
}
