// PlanarPortal.cpp
//
// Breannan Smith
// Last updated: 09/10/2015

#include "PlanarPortal.h"

#include "SCISim/Utilities.h"
#include "SCISim/Math/MathUtilities.h"

#include <iostream>

TeleportedBody::TeleportedBody( const unsigned body_index, const unsigned portal_index, const bool plane_index )
: m_body_index( body_index )
, m_portal_index( portal_index )
, m_plane_index( plane_index )
{}

unsigned TeleportedBody::bodyIndex() const
{
  return m_body_index;
}

unsigned TeleportedBody::portalIndex() const
{
  return m_portal_index;
}

bool TeleportedBody::planeIndex() const
{
  return m_plane_index;
}

TeleportedCollision::TeleportedCollision( const unsigned body_index_0, const unsigned body_index_1, const unsigned portal_index_0,  const unsigned portal_index_1, const bool plane_0, const bool plane_1 )
: m_body_index_0( body_index_0 < body_index_1 ? body_index_0 : body_index_1 )
, m_body_index_1( body_index_0 < body_index_1 ? body_index_1 : body_index_0 )
, m_portal_index_0( body_index_0 < body_index_1 ? portal_index_0 : portal_index_1 )
, m_portal_index_1( body_index_0 < body_index_1 ? portal_index_1 : portal_index_0 )
, m_plane_0( body_index_0 < body_index_1 ? plane_0 : plane_1 )
, m_plane_1( body_index_0 < body_index_1 ? plane_1 : plane_0 )
{
  assert( m_body_index_0 != m_body_index_1 );
}

bool TeleportedCollision::operator<( const TeleportedCollision& rhs ) const
{
  assert( m_body_index_0 < m_body_index_1 ); assert( rhs.m_body_index_0 < rhs.m_body_index_1 );
  return std::tie( m_body_index_0, m_body_index_1 ) < std::tie( rhs.m_body_index_0, rhs.m_body_index_1 );
}

unsigned TeleportedCollision::bodyIndex0() const
{
  return m_body_index_0;
}

unsigned TeleportedCollision::bodyIndex1() const
{
  return m_body_index_1;
}

unsigned TeleportedCollision::portalIndex0() const
{
  return m_portal_index_0;
}

unsigned TeleportedCollision::portalIndex1() const
{
  return m_portal_index_1;
}

bool TeleportedCollision::plane0() const
{
  return m_plane_0;
}

bool TeleportedCollision::plane1() const
{
  return m_plane_1;
}


PlanarPortal::PlanarPortal( const RigidBody2DStaticPlane& plane_a, const RigidBody2DStaticPlane& plane_b )
: m_plane_a( plane_a )
, m_plane_b( plane_b )
, m_v_a( 0.0 )
, m_v_b( 0.0 )
, m_bounds_a( Vector2s::Zero() )
, m_bounds_b( Vector2s::Zero() )
, m_dx_a( 0.0 )
, m_dx_b( 0.0 )
{}

PlanarPortal::PlanarPortal( const RigidBody2DStaticPlane& plane_a, const RigidBody2DStaticPlane& plane_b, const scalar& va, const scalar& vb, const Vector2s& boundsa, const Vector2s& boundsb )
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

const RigidBody2DStaticPlane& PlanarPortal::planeA() const
{
  return m_plane_a;
}

const RigidBody2DStaticPlane& PlanarPortal::planeB() const
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

const scalar& PlanarPortal::vA() const
{
  return m_v_a;
}

const scalar& PlanarPortal::vB() const
{
  return m_v_b;
}

const scalar& PlanarPortal::deltaA() const
{
  return m_dx_a;
}

const scalar& PlanarPortal::deltaB() const
{
  return m_dx_b;
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

// TODO: Test displacement here
bool PlanarPortal::pointInsidePortal( const Vector2s& x ) const
{
  return planeA().distanceLessThanZero( m_dx_a * m_plane_a.t(), x ) || planeB().distanceLessThanZero( m_dx_b * m_plane_b.t(), x );
}

static bool aabbInHalfPlane( const Array2s& min, const Array2s& max, const Vector2s& plane_dx, const RigidBody2DStaticPlane& plane )
{
  // The AABB is in the plane if and only if at least one of its vertices is in the plane
  if( plane.distanceToPoint( plane_dx, Vector2s{ min.x(), min.y() } ) <= 0 ) { return true; }
  if( plane.distanceToPoint( plane_dx, Vector2s{ min.x(), max.y() } ) <= 0 ) { return true; }
  if( plane.distanceToPoint( plane_dx, Vector2s{ max.x(), min.y() } ) <= 0 ) { return true; }
  if( plane.distanceToPoint( plane_dx, Vector2s{ max.x(), max.y() } ) <= 0 ) { return true; }
  return false;
}

bool PlanarPortal::aabbTouchesPortal( const Array2s& min, const Array2s& max, bool& intersecting_plane_idx ) const
{
  #ifndef NDEBUG
  if( aabbInHalfPlane( min, max, m_dx_a * m_plane_a.t(), m_plane_a ) && aabbInHalfPlane( min, max, m_dx_b * m_plane_b.t(), m_plane_b ) )
  {
    std::cerr << "Unhandled case in PlanarPortal::aabbTouchesPortal, AABB is not allowed to touch both planes of a portal at once." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  #endif

  if( aabbInHalfPlane( min, max, m_dx_a * m_plane_a.t(), m_plane_a ) )
  {
    intersecting_plane_idx = 0;
    return true;
  }
  if( aabbInHalfPlane( min, max, m_dx_b * m_plane_b.t(), m_plane_b ) )
  {
    intersecting_plane_idx = 1;
    return true;
  }
  return false;
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

void PlanarPortal::teleportPoint( const Vector2s& xin, const bool intersecting_plane_idx, Vector2s& xout ) const
{
  // Plane A -> Plane B
  if( !intersecting_plane_idx )
  {
    teleportPointThroughPlaneA( xin, xout );
  }
  // Plane B -> Plane A
  else
  {
    teleportPointThroughPlaneB( xin, xout );
  }
}

Vector2s PlanarPortal::getKinematicVelocityOfAABB( const Array2s& min, const Array2s& max ) const
{
  assert( aabbInPlaneA( min, max ) != aabbInPlaneB( min, max ) );

  if( aabbInPlaneA( min, max ) )
  {
    return m_v_b * m_plane_b.t() - m_v_a * m_plane_a.t();
  }
  else
  {
    return m_v_a * m_plane_a.t() - m_v_b * m_plane_b.t();
  }
}

bool PlanarPortal::aabbInPlaneA( const Array2s& min, const Array2s& max ) const
{
  // The AABB is in the plane if and only if at least one of its vertices is in the plane
  if( m_plane_a.distanceToPoint( m_dx_a * m_plane_a.t(), Vector2s{ min.x(), min.y() } ) <= 0 ) { return true; }
  if( m_plane_a.distanceToPoint( m_dx_a * m_plane_a.t(), Vector2s{ min.x(), max.y() } ) <= 0 ) { return true; }
  if( m_plane_a.distanceToPoint( m_dx_a * m_plane_a.t(), Vector2s{ max.x(), min.y() } ) <= 0 ) { return true; }
  if( m_plane_a.distanceToPoint( m_dx_a * m_plane_a.t(), Vector2s{ max.x(), max.y() } ) <= 0 ) { return true; }
  return false;
}

bool PlanarPortal::aabbInPlaneB( const Array2s& min, const Array2s& max ) const
{
  // The AABB is in the plane if and only if at least one of its vertices is in the plane
  if( m_plane_b.distanceToPoint( m_dx_b * m_plane_b.t(), Vector2s{ min.x(), min.y() } ) <= 0 ) { return true; }
  if( m_plane_b.distanceToPoint( m_dx_b * m_plane_b.t(), Vector2s{ min.x(), max.y() } ) <= 0 ) { return true; }
  if( m_plane_b.distanceToPoint( m_dx_b * m_plane_b.t(), Vector2s{ max.x(), min.y() } ) <= 0 ) { return true; }
  if( m_plane_b.distanceToPoint( m_dx_b * m_plane_b.t(), Vector2s{ max.x(), max.y() } ) <= 0 ) { return true; }
  return false;
}

// TODO: If amount above/below portal is more than a single grid width, could be a bug
void PlanarPortal::teleportPointThroughPlaneA( const Vector2s& xin, Vector2s& xout ) const
{
  const Vector2s x_plane{ planeA().x() + m_dx_a * m_plane_a.t() };

  // Compute coordinates of x in A and invert them
  const scalar nA{ planeA().n().dot( x_plane - xin ) };
  scalar tA{ planeA().t().dot( x_plane - xin ) };
  // Compute the inverted coordinates in B
  // If below the portal, add the portal width to get in the correct position
  if( m_dx_b + tA < m_bounds_b(0) )
  {
    while( m_dx_b + tA < m_bounds_b(0) )
    {
      tA += m_bounds_b(1) - m_bounds_b(0);
    }
  }
  // If above the portal, subtract the portal width to get in the correct position
  else if( m_dx_b + tA > m_bounds_b(1) )
  {
    while( m_dx_b + tA > m_bounds_b(1) )
    {
      tA += m_bounds_b(0) - m_bounds_b(1);
    }
  }
  assert( m_dx_b + tA >= m_bounds_b(0) ); assert( m_dx_b + tA <= m_bounds_b(1) );
  xout = planeB().x() + m_dx_b * m_plane_b.t() + nA * planeB().n() + tA * planeB().t();
}

// TODO: If amount above/below portal is more than a single grid width, could be a bug
void PlanarPortal::teleportPointThroughPlaneB( const Vector2s& xin, Vector2s& xout ) const
{
  const Vector2s x_plane{ planeB().x() + m_dx_b * m_plane_b.t() };

  // Compute coordinates of x in B and invert them
  const scalar nB{ planeB().n().dot( x_plane - xin ) };
  scalar tB{ planeB().t().dot( x_plane - xin ) };
  // Compute the inverted coordinates in B
  // If below the portal, add the portal width to get in the correct position
  if( m_dx_a + tB < m_bounds_a(0) )
  {
    while( m_dx_a + tB < m_bounds_a(0) )
    {
      tB += m_bounds_a(1) - m_bounds_a(0);
    }
  }
  // If above the portal, subtract the portal width to get in the correct position
  else if( m_dx_a + tB > m_bounds_a(1) )
  {
    while( m_dx_a + tB > m_bounds_a(1) )
    {
      tB += m_bounds_a(0) - m_bounds_a(1);
    }
  }
  assert( m_dx_a + tB >= m_bounds_a(0) ); assert( m_dx_a + tB <= m_bounds_a(1) );
  xout = planeA().x() + m_dx_a * m_plane_a.t() + nB * planeA().n() + tB * planeA().t();
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
    const scalar dxa = m_v_a * t;
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
