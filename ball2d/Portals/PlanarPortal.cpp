// PlanarPortal.cpp
//
// Breannan Smith
// Last updated: 09/07/2015

#include "PlanarPortal.h"

#include "SCISim/Utilities.h"
#include "SCISim/Math/MathUtilities.h"

#include <iostream>

TeleportedBall::TeleportedBall( const unsigned body_index, const unsigned portal_index, const bool plane_index )
: m_body_index( body_index )
, m_portal_index( portal_index )
, m_plane_index( plane_index )
{}

unsigned TeleportedBall::bodyIndex() const
{
  return m_body_index;
}

unsigned TeleportedBall::portalIndex() const
{
  return m_portal_index;
}

bool TeleportedBall::planeIndex() const
{
  return m_plane_index;
}

TeleportedCollision::TeleportedCollision( const unsigned body_index_0, const unsigned body_index_1, const unsigned portal_index_0,  const unsigned portal_index_1, const bool plane_0, const bool plane_1 )
: m_body_index_0( body_index_0 )
, m_body_index_1( body_index_1 )
, m_portal_index_0( portal_index_0 )
, m_portal_index_1( portal_index_1 )
, m_plane_0( plane_0 )
, m_plane_1( plane_1 )
{
  assert( m_body_index_0 != m_body_index_1 );
  if( m_body_index_0 > m_body_index_1 )
  {
    std::swap( m_body_index_0, m_body_index_1 );
    std::swap( m_portal_index_0, m_portal_index_1 );
    std::swap( m_plane_0, m_plane_1 );
  }
}

bool TeleportedCollision::operator<( const TeleportedCollision& rhs ) const
{
  assert( m_body_index_0 < m_body_index_1 );
  return std::tie( m_body_index_0, m_body_index_1 ) < std::tie( rhs.m_body_index_0, rhs.m_body_index_1 );
}



PlanarPortal::PlanarPortal( const StaticPlane& plane_a, const StaticPlane& plane_b, const scalar& va, const scalar& vb, const Vector2s& boundsa, const Vector2s& boundsb )
: m_plane_a( plane_a )
, m_plane_b( plane_b )
, m_v_a( va )
, m_v_b( vb )
, m_bounds_a( boundsa )
, m_bounds_b( boundsb )
, m_dx_a( 0.0 )
, m_dx_b( 0.0 )
{
  assert( m_bounds_a(0) <= 0.0 ); assert( m_bounds_a(1) >= 0.0 );
  assert( m_bounds_b(0) <= 0.0 ); assert( m_bounds_b(1) >= 0.0 );
  assert( ( ( m_bounds_a(0) == -SCALAR_INFINITY ) && ( m_bounds_a(1) == SCALAR_INFINITY ) ) || ( std::isfinite( m_bounds_a(0) ) && std::isfinite( m_bounds_a(1) ) ) );
  assert( ( ( m_bounds_b(0) == -SCALAR_INFINITY ) && ( m_bounds_b(1) == SCALAR_INFINITY ) ) || ( std::isfinite( m_bounds_b(0) ) && std::isfinite( m_bounds_b(1) ) ) );
}

PlanarPortal::PlanarPortal( std::istream& input_stream )
: m_plane_a( input_stream )
, m_plane_b( input_stream )
, m_v_a( Utilities::deserialize<scalar>( input_stream ) )
, m_v_b( Utilities::deserialize<scalar>( input_stream ) )
, m_bounds_a( MathUtilities::deserialize<Vector2s>( input_stream ) )
, m_bounds_b( MathUtilities::deserialize<Vector2s>( input_stream ) )
, m_dx_a( Utilities::deserialize<scalar>( input_stream ) )
, m_dx_b( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( m_bounds_a(0) <= 0.0 ); assert( m_bounds_a(1) >= 0.0 );
  assert( m_bounds_b(0) <= 0.0 ); assert( m_bounds_b(1) >= 0.0 );
  assert( ( ( m_bounds_a(0) == -SCALAR_INFINITY ) && ( m_bounds_a(1) == SCALAR_INFINITY ) ) || ( std::isfinite( m_bounds_a(0) ) && std::isfinite( m_bounds_a(1) ) ) );
  assert( ( ( m_bounds_b(0) == -SCALAR_INFINITY ) && ( m_bounds_b(1) == SCALAR_INFINITY ) ) || ( std::isfinite( m_bounds_b(0) ) && std::isfinite( m_bounds_b(1) ) ) );
}

const StaticPlane& PlanarPortal::planeA() const
{
  return m_plane_a;
}

const StaticPlane& PlanarPortal::planeB() const
{
  return m_plane_b;
}

const Vector2s& PlanarPortal::boundsA() const
{
  return m_bounds_a;
}

const Vector2s& PlanarPortal::boundsB() const
{
  return m_bounds_b;
}

scalar PlanarPortal::planeAx() const
{
  return m_plane_a.x().x() + m_plane_a.t().x() * m_dx_a;
}

scalar PlanarPortal::planeAy() const
{
  return m_plane_a.x().y() + m_plane_a.t().y() * m_dx_a;
}

scalar PlanarPortal::planeBx() const
{
  return m_plane_b.x().x() + m_plane_b.t().x() * m_dx_b;
}

scalar PlanarPortal::planeBy() const
{
  return m_plane_b.x().y() + m_plane_b.t().y() * m_dx_b;
}

bool PlanarPortal::pointInsidePortal( const Vector2s& x ) const
{
  return planeA().distanceLessThanZero( m_dx_a * m_plane_a.t(), x ) || planeB().distanceLessThanZero( m_dx_b * m_plane_b.t(), x );
}

bool PlanarPortal::ballTouchesPortal( const Vector2s& x, const scalar& r, bool& intersecting_plane_idx ) const
{
  assert( r > 0.0 );
  const bool touches_plane_A{ ballInPlaneA( x, r ) };
  const bool touches_plane_B{ ballInPlaneB( x, r ) };
  if( touches_plane_A && touches_plane_B )
  {
    std::cerr << "Unhandled case in PlanarPortal::ballInsidePortal. Ball can't touch both planes of a portal at once." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  // TODO: simplify this
  if( touches_plane_A ) { intersecting_plane_idx = 0; }
  if( touches_plane_B ) { intersecting_plane_idx = 1; }
  return touches_plane_A || touches_plane_B;
}

void PlanarPortal::teleportPointInsidePortal( const Vector2s& xin, Vector2s& xout ) const
{
  assert( planeA().distanceLessThanZero( m_dx_a * m_plane_a.t(), xin ) != planeB().distanceLessThanZero( m_dx_b * m_plane_b.t(), xin ) );

  // Teleporting form A to B
  if( planeA().distanceLessThanZero( m_dx_a * m_plane_a.t(), xin ) )
  {
    teleportPointThroughPlaneA( xin, xout );
  }
  // Teleporting form B to A
  else
  {
    teleportPointThroughPlaneB( xin, xout );
  }
}

void PlanarPortal::teleportBall( const Vector2s& xin, const scalar& r, Vector2s& xout ) const
{
  assert( ballInPlaneA( xin, r ) != ballInPlaneB( xin, r ) );

  // Teleporting form A to B
  if( ballInPlaneA( xin, r ) )
  {
    teleportPointThroughPlaneA( xin, xout );
  }
  // Teleporting form B to A
  else
  {
    teleportPointThroughPlaneB( xin, xout );
  }
}

Vector2s PlanarPortal::getKinematicVelocityOfBall( const Vector2s& x, const scalar& r ) const
{
  assert( ballInPlaneA( x, r ) != ballInPlaneB( x, r ) );

  if( ballInPlaneA( x, r ) )
  {
    return m_v_b * m_plane_b.t() - m_v_a * m_plane_a.t();
  }
  else
  {
    return m_v_a * m_plane_a.t() - m_v_b * m_plane_b.t();
  }
}

bool PlanarPortal::ballInPlaneA( const Vector2s& x, const scalar& r ) const
{
  return m_plane_a.distanceLessThanOrEqualZero( m_dx_a * m_plane_a.t(), x, r );
}

bool PlanarPortal::ballInPlaneB( const Vector2s& x, const scalar& r ) const
{
  return m_plane_b.distanceLessThanOrEqualZero( m_dx_b * m_plane_b.t(), x, r );
}

void PlanarPortal::teleportPointThroughPlaneA( const Vector2s& xin, Vector2s& xout ) const
{
  // Compute coordinates of x in A and invert them
  const scalar nA{ planeA().n().dot( m_dx_a * m_plane_a.t() + planeA().x() - xin ) };
  scalar tA{ planeA().t().dot( m_dx_a * m_plane_a.t() + planeA().x() - xin ) };
  // Compute the inverted coordinates in B
  if( m_dx_b + tA < m_bounds_b(0) )
  {
    while( m_dx_b + tA < m_bounds_b(0) )
    {
      tA += m_bounds_b(1) - m_bounds_b(0);
    }
  }
  else if( m_dx_b + tA > m_bounds_b(1) )
  {
    while( m_dx_b + tA > m_bounds_b(1) )
    {
      tA += m_bounds_b(0) - m_bounds_b(1);
    }
  }
  assert( m_dx_b + tA >= m_bounds_b(0) ); assert( m_dx_b + tA <= m_bounds_b(1) );
  xout = m_dx_b * m_plane_b.t() + planeB().x() + nA * planeB().n() + tA * planeB().t();
}

void PlanarPortal::teleportPointThroughPlaneB( const Vector2s& xin, Vector2s& xout ) const
{
  // Compute coordinates of x in B and invert them
  const scalar nB{ planeB().n().dot( m_dx_b * m_plane_b.t() + planeB().x() - xin ) };
  scalar tB{ planeB().t().dot( m_dx_b * m_plane_b.t() + planeB().x() - xin ) };
  // Compute the inverted coordinates in B
  if( m_dx_a + tB < m_bounds_a(0) )
  {
    while( m_dx_a + tB < m_bounds_a(0) )
    {
      tB += m_bounds_a(1) - m_bounds_a(0);
    }
  }
  else if( m_dx_a + tB > m_bounds_a(1) )
  {
    while( m_dx_a + tB > m_bounds_a(1) )
    {
      tB += m_bounds_a(0) - m_bounds_a(1);
    }
  }
  assert( m_dx_a + tB >= m_bounds_a(0) ); assert( m_dx_a + tB <= m_bounds_a(1) );
  xout = m_dx_a * m_plane_a.t() + planeA().x() + nB * planeA().n() + tB * planeA().t();
}

void PlanarPortal::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  m_plane_a.serialize( output_stream );
  m_plane_b.serialize( output_stream );
  Utilities::serializeBuiltInType( m_v_a, output_stream );
  Utilities::serializeBuiltInType( m_v_b, output_stream );
  MathUtilities::serialize( m_bounds_a, output_stream );
  MathUtilities::serialize( m_bounds_b, output_stream );
  Utilities::serializeBuiltInType( m_dx_a, output_stream );
  Utilities::serializeBuiltInType( m_dx_b, output_stream );
}

void PlanarPortal::updateMovingPortals( const scalar& t )
{
  // First plane
  if( m_v_a > 0.0 )
  {
    const scalar dxa{ m_v_a * t };
    if( dxa <= m_bounds_a(1) )
    {
      m_dx_a = dxa;
    }
    else
    {
      m_dx_a = fmod( dxa - m_bounds_a(1), m_bounds_a(1) - m_bounds_a(0) ) + m_bounds_a(0);
    }
  }
  else if( m_v_a < 0.0 )
  {
    const scalar dxa{ m_v_a * t };
    if( dxa >= m_bounds_a(0) )
    {
      m_dx_a = dxa;
    }
    else
    {
      m_dx_a = fmod( dxa - m_bounds_a(0), m_bounds_a(0) - m_bounds_a(1) ) + m_bounds_a(1);
    }
  }
  assert( m_dx_a >= m_bounds_a(0) ); assert( m_dx_a <= m_bounds_a(1) );
  // Second plane
  if( m_v_b > 0.0 )
  {
    const scalar dxb{ m_v_b * t };
    if( dxb <= m_bounds_b(1) )
    {
      m_dx_b = dxb;
    }
    else
    {
      m_dx_b = fmod( dxb - m_bounds_b(1), m_bounds_b(1) - m_bounds_b(0) ) + m_bounds_b(0);
    }
  }
  else if( m_v_b < 0.0 )
  {
    const scalar dxb{ m_v_b * t };
    if( dxb >= m_bounds_b(0) )
    {
      m_dx_b = dxb;
    }
    else
    {
      m_dx_b = fmod( dxb - m_bounds_b(0), m_bounds_b(0) - m_bounds_b(1) ) + m_bounds_b(1);
    }
  }
  assert( m_dx_b >= m_bounds_b(0) ); assert( m_dx_b <= m_bounds_b(1) );
}

bool PlanarPortal::isLeesEdwards() const
{
  return m_v_a != 0.0 || m_v_b != 0.0;
}
