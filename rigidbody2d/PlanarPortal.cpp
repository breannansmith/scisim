// PlanarPortal.cpp
//
// Breannan Smith
// Last updated: 09/10/2015

#include "PlanarPortal.h"

#include "scisim/Utilities.h"

#ifndef NDEBUG
#include <iostream>
#endif

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
, m_v( 0.0 )
, m_bounds( 0.0 )
, m_dx( 0.0 )
{}

PlanarPortal::PlanarPortal( const RigidBody2DStaticPlane& plane_a, const RigidBody2DStaticPlane& plane_b, const scalar& velocity, const scalar& bounds )
: m_plane_a( plane_a )
, m_plane_b( plane_b )
, m_v( velocity )
, m_bounds( bounds )
, m_dx( 0.0 )
{
  assert( m_bounds >= 0.0 );
}

PlanarPortal::PlanarPortal( std::istream& input_stream )
: m_plane_a( input_stream )
, m_plane_b( input_stream )
, m_v( Utilities::deserialize<scalar>( input_stream ) )
, m_bounds( Utilities::deserialize<scalar>( input_stream ) )
, m_dx( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( m_bounds >= 0.0 );
}

const RigidBody2DStaticPlane& PlanarPortal::planeA() const
{
  return m_plane_a;
}

const RigidBody2DStaticPlane& PlanarPortal::planeB() const
{
  return m_plane_b;
}

const scalar& PlanarPortal::bounds() const
{
  return m_bounds;
}

const scalar& PlanarPortal::v() const
{
  return m_v;
}

const scalar& PlanarPortal::delta() const
{
  return m_dx;
}

Vector2s PlanarPortal::transformedAx() const
{
  return m_plane_a.x() + m_plane_a.t() * m_dx;
}

Vector2s PlanarPortal::transformedBx() const
{
  return m_plane_b.x() + m_plane_b.t() * m_dx;
}

bool PlanarPortal::pointInsidePortal( const Vector2s& x ) const
{
  return planeA().distanceLessThanZero( x ) || planeB().distanceLessThanZero( x );
}

static bool aabbInHalfPlane( const Array2s& min, const Array2s& max, const RigidBody2DStaticPlane& plane )
{
  // The AABB is in the plane if and only if at least one of its vertices is in the plane
  if( plane.distanceToPoint( Vector2s{ min.x(), min.y() } ) <= 0 )
  {
    return true;
  }
  if( plane.distanceToPoint( Vector2s{ min.x(), max.y() } ) <= 0 )
  {
    return true;
  }
  if( plane.distanceToPoint( Vector2s{ max.x(), min.y() } ) <= 0 )
  {
    return true;
  }
  if( plane.distanceToPoint( Vector2s{ max.x(), max.y() } ) <= 0 )
  {
    return true;
  }
  return false;
}

bool PlanarPortal::aabbTouchesPortal( const Array2s& min, const Array2s& max, bool& intersecting_plane_idx ) const
{
  #ifndef NDEBUG
  if( aabbInHalfPlane( min, max, m_plane_a ) && aabbInHalfPlane( min, max, m_plane_b ) )
  {
    std::cerr << "Unhandled case in PlanarPortal::aabbTouchesPortal, AABB is not allowed to touch both planes of a portal at once." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  #endif

  if( aabbInHalfPlane( min, max, m_plane_a ) )
  {
    intersecting_plane_idx = 0;
    return true;
  }
  if( aabbInHalfPlane( min, max, m_plane_b ) )
  {
    intersecting_plane_idx = 1;
    return true;
  }
  return false;
}

void PlanarPortal::teleportPointInsidePortal( const Vector2s& xin, Vector2s& xout ) const
{
  assert( planeA().distanceLessThanZero( xin ) != planeB().distanceLessThanZero( xin ) );

  // Teleporting form A to B
  if( planeA().distanceLessThanZero( xin ) )
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
    return -m_v * m_plane_a.t();
  }
  else
  {
    return -m_v * m_plane_b.t();
  }
}

Vector2s PlanarPortal::getKinematicVelocityOfPoint( const Vector2s& x ) const
{
  assert( planeA().distanceLessThanZero( x ) != planeB().distanceLessThanZero( x ) );

  if( planeA().distanceLessThanZero( x ) )
  {
    return -m_v * m_plane_a.t();
  }
  else
  {
    return -m_v * m_plane_b.t();
  }
}

bool PlanarPortal::aabbInPlaneA( const Array2s& min, const Array2s& max ) const
{
  // The AABB is in the plane if and only if at least one of its vertices is in the plane
  if( m_plane_a.distanceToPoint( Vector2s{ min.x(), min.y() } ) <= 0 )
  {
    return true;
  }
  if( m_plane_a.distanceToPoint( Vector2s{ min.x(), max.y() } ) <= 0 )
  {
    return true;
  }
  if( m_plane_a.distanceToPoint( Vector2s{ max.x(), min.y() } ) <= 0 )
  {
    return true;
  }
  if( m_plane_a.distanceToPoint( Vector2s{ max.x(), max.y() } ) <= 0 )
  {
    return true;
  }
  return false;
}

bool PlanarPortal::aabbInPlaneB( const Array2s& min, const Array2s& max ) const
{
  // The AABB is in the plane if and only if at least one of its vertices is in the plane
  if( m_plane_b.distanceToPoint( Vector2s{ min.x(), min.y() } ) <= 0 )
  {
    return true;
  }
  if( m_plane_b.distanceToPoint( Vector2s{ min.x(), max.y() } ) <= 0 )
  {
    return true;
  }
  if( m_plane_b.distanceToPoint( Vector2s{ max.x(), min.y() } ) <= 0 )
  {
    return true;
  }
  if( m_plane_b.distanceToPoint( Vector2s{ max.x(), max.y() } ) <= 0 )
  {
    return true;
  }
  return false;
}

void PlanarPortal::teleportPointThroughPlaneA( const Vector2s& xin, Vector2s& xout ) const
{
  // Compute the translated coordinates of x in B...
  const scalar nA{ planeA().n().dot( planeA().x() - xin ) };
  scalar tA{ planeA().t().dot( m_dx * m_plane_a.t() + planeA().x() - xin ) };

  // ...accounting for periodic boundaries
  const int repeat_x{ int( floor( ( tA + m_bounds ) / ( 2.0 * m_bounds ) ) ) };
  tA -= 2.0 * repeat_x * m_bounds;
  assert( tA >= -m_bounds || !isLeesEdwards() );
  assert( tA <= m_bounds || !isLeesEdwards() );

  // Map the coordinates to the corresponding plane
  xout = planeB().x() + nA * planeB().n() + tA * planeB().t();
}

void PlanarPortal::teleportPointThroughPlaneB( const Vector2s& xin, Vector2s& xout ) const
{
  // Compute the translated coordinates of x in B...
  const scalar nB{ planeB().n().dot( planeB().x() - xin ) };
  scalar tB{ planeB().t().dot( m_dx * m_plane_b.t() + planeB().x() - xin ) };

  // ...accounting for periodic boundaries
  const int repeat_x{ int( floor( ( tB + m_bounds ) / ( 2.0 * m_bounds ) ) ) };
  tB -= 2.0 * repeat_x * m_bounds;
  assert( tB >= -m_bounds || !isLeesEdwards() );
  assert( tB <= m_bounds || !isLeesEdwards() );

  // Map the coordinates to the corresponding plane
  xout = planeA().x() + nB * planeA().n() + tB * planeA().t();
}

void PlanarPortal::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  m_plane_a.serialize( output_stream );
  m_plane_b.serialize( output_stream );
  Utilities::serializeBuiltInType( m_v, output_stream );
  Utilities::serializeBuiltInType( m_bounds, output_stream );
  Utilities::serializeBuiltInType( m_dx, output_stream );
}

void PlanarPortal::updateMovingPortals( const scalar& t )
{
  const int repeat_x{ int( floor( ( m_v * t + m_bounds ) / ( 2.0 * m_bounds ) ) ) };
  m_dx = m_v * t - 2.0 * repeat_x * m_bounds;
  assert( m_dx >= -m_bounds ); assert( m_dx <= m_bounds );
}

bool PlanarPortal::isLeesEdwards() const
{
  return m_v != 0.0;
}
