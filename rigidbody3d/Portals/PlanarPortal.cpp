// PlanarPortal.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "PlanarPortal.h"

#include "scisim/Math/MathUtilities.h"

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



PlanarPortal::PlanarPortal( const StaticPlane& plane_a, const StaticPlane& plane_b, const Array3i& portal_multiplier )
: m_plane_a( plane_a )
, m_plane_b( plane_b )
, m_portal_multiplier( portal_multiplier )
{}

PlanarPortal::PlanarPortal( std::istream& input_stream )
: m_plane_a( input_stream )
, m_plane_b( input_stream )
, m_portal_multiplier()
{
  assert( input_stream.good() );
  m_portal_multiplier = MathUtilities::deserialize<Array3i>( input_stream );
}

const StaticPlane& PlanarPortal::planeA() const
{
  return m_plane_a;
}

const StaticPlane& PlanarPortal::planeB() const
{
  return m_plane_b;
}

bool PlanarPortal::pointInsidePortal( const Vector3s& x ) const
{
  return ( planeA().distanceToPoint( x ) < 0 ) || ( planeB().distanceToPoint( x ) < 0 );
}

static bool aabbInHalfPlane( const Array3s& min, const Array3s& max, const StaticPlane& plane )
{
  // The AABB is in the plane if and only if at least one of its vertices is in the plane
  if( plane.distanceToPoint( Vector3s( min.x(), min.y(), min.z() ) ) <= 0 ) { return true; }
  if( plane.distanceToPoint( Vector3s( min.x(), min.y(), max.z() ) ) <= 0 ) { return true; }
  if( plane.distanceToPoint( Vector3s( min.x(), max.y(), min.z() ) ) <= 0 ) { return true; }
  if( plane.distanceToPoint( Vector3s( min.x(), max.y(), max.z() ) ) <= 0 ) { return true; }
  if( plane.distanceToPoint( Vector3s( max.x(), min.y(), min.z() ) ) <= 0 ) { return true; }
  if( plane.distanceToPoint( Vector3s( max.x(), min.y(), max.z() ) ) <= 0 ) { return true; }
  if( plane.distanceToPoint( Vector3s( max.x(), max.y(), min.z() ) ) <= 0 ) { return true; }
  if( plane.distanceToPoint( Vector3s( max.x(), max.y(), max.z() ) ) <= 0 ) { return true; }
  return false;
}

bool PlanarPortal::aabbTouchesPortal( const Array3s& min, const Array3s& max, bool& intersecting_plane_idx ) const
{
  #ifndef NDEBUG
  if( aabbInHalfPlane( min, max, m_plane_a ) && aabbInHalfPlane( min, max, m_plane_b ) )
  {
    std::cerr << "Unhandled case in PlanarPortal::aabbTouchesPortal. AABB is not allowed to touch both planes of a portal at once." << std::endl;
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

void PlanarPortal::teleportPointInsidePortal( const Vector3s& xin, Vector3s& xout ) const
{
  // Point must be in one and only one side of the portal
  assert( ( planeA().distanceToPoint( xin ) < 0 ) != ( planeB().distanceToPoint( xin ) < 0 ) );

  // Teleporting form A to B
  if( planeA().distanceToPoint( xin ) < 0 )
  {
    teleportPointThroughPlaneA( xin, xout );
  }
  // Teleporting form B to A
  else
  {
    teleportPointThroughPlaneB( xin, xout );
  }
}

void PlanarPortal::teleportPoint( const Vector3s& xin, const bool intersecting_plane_idx, Vector3s& xout ) const
{
  // Plane A -> Plane B
  if( intersecting_plane_idx == 0 )
  {
    teleportPointThroughPlaneA( xin, xout );
  }
  // Plane B -> Plane A
  else
  {
    teleportPointThroughPlaneB( xin, xout );
  }
}

void PlanarPortal::teleportPointThroughPlaneA( const Vector3s& xin, Vector3s& xout ) const
{
  // Compute coordinates of x in A and invert them
  const scalar nA{ m_portal_multiplier.x() * planeA().n().dot(  planeA().x() - xin ) };
  const scalar tA0{ m_portal_multiplier.y() * planeA().t0().dot( planeA().x() - xin ) };
  const scalar tA1{ m_portal_multiplier.z() * planeA().t1().dot( planeA().x() - xin ) };
  // Compute the inverted coordinates in B
  xout = planeB().x() + nA * planeB().n() + tA0 * planeB().t0() + tA1 * planeB().t1();
}

void PlanarPortal::teleportPointThroughPlaneB( const Vector3s& xin, Vector3s& xout ) const
{
  // Compute coordinates of x in B and invert them
  const scalar nB{ m_portal_multiplier.x() * planeB().n().dot(  planeB().x() - xin ) };
  const scalar tB0{ m_portal_multiplier.y() * planeB().t0().dot( planeB().x() - xin ) };
  const scalar tB1{ m_portal_multiplier.z() * planeB().t1().dot( planeB().x() - xin ) };
  // Compute the inverted coordinates in B
  xout = planeA().x() + nB * planeA().n() + tB0 * planeA().t0() + tB1 * planeA().t1();
}

void PlanarPortal::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  m_plane_a.serialize( output_stream );
  m_plane_b.serialize( output_stream );
  MathUtilities::serialize( m_portal_multiplier, output_stream );
}
