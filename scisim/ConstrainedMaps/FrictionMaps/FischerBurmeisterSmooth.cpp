// FischerBurmeisterSmooth.cpp
//
// Breannan Smith
// Last updated: 09/08/2015

#include "FischerBurmeisterSmooth.h"

FischerBurmeisterSmooth::FischerBurmeisterSmooth( const scalar& tol, const VectorXs& c )
: m_tol( tol )
, m_c( c )
{
  assert( ( m_c.array() >= 0.0 ).all() );
}

FischerBurmeisterSmooth::~FischerBurmeisterSmooth()
{}

static scalar getSignedTangentMagnitude( const Vector2s& x, const scalar& c, const Vector2s& grad )
{
  if( x.norm() >= c )
  {
    Vector2s n;
    if( x.norm() > 0.0 )
    {
      n = x / x.norm();
    }
    else
    {
      if( c == 0.0 )
      {
        return 0.0;
      }
      else
      {
        n.setZero();
      }
    }

    const Vector2s t{ grad - grad.dot( n ) * n };
    return - t.norm();
  }
  else
  {
    return grad.norm();
  }
}

scalar FischerBurmeisterSmooth::operator()( const VectorXs& beta, const VectorXs& vrel ) const
{
  assert( beta.size() == vrel.size() );
  assert( beta.size() % 2 == 0 );
  assert( beta.size() / 2 == m_c.size() );

  const unsigned ncons{ static_cast<unsigned>( beta.size() / 2 ) };
  scalar inf_norm{ 0.0 };
  for( unsigned con_num = 0; con_num < ncons; ++con_num )
  {
    // Only project velocities (-vrel) pointing outward
    const scalar lambda{ getSignedTangentMagnitude( beta.segment<2>( 2 * con_num ), m_c( con_num ), vrel.segment<2>( 2 * con_num ) ) };
    const scalar disc_violation{ m_c( con_num ) - beta.segment<2>( 2 * con_num ).norm() };
    const scalar fb{ fabs( sqrt( lambda * lambda + disc_violation * disc_violation ) - lambda - disc_violation ) };
    if( fb > inf_norm )
    {
      inf_norm = fb;
    }
  }

  return inf_norm;
}

scalar FischerBurmeisterSmooth::tol() const
{
  return m_tol;
}
