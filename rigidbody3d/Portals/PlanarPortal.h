// PlanarPortal.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef PLANAR_PORTAL_H
#define PLANAR_PORTAL_H

#include "scisim/Math/MathDefines.h"
#include "rigidbody3d/StaticGeometry/StaticPlane.h"

class TeleportedBody final
{

public:

  TeleportedBody( const unsigned body_index, const unsigned portal_index, const bool plane_index );

  unsigned bodyIndex() const;

  unsigned portalIndex() const;

  bool planeIndex() const;

private:

  const unsigned m_body_index;
  const unsigned m_portal_index;
  const bool m_plane_index;

};

class TeleportedCollision final
{

public:

  TeleportedCollision( const unsigned body_index_0, const unsigned body_index_1, const unsigned portal_index_0,  const unsigned portal_index_1, const bool plane_0, const bool plane_1 );

  bool operator<( const TeleportedCollision& rhs ) const;

  unsigned bodyIndex0() const;
  unsigned bodyIndex1() const;

  unsigned portalIndex0() const;
  unsigned portalIndex1() const;

  bool plane0() const;
  bool plane1() const;

private:

  const unsigned m_body_index_0;
  const unsigned m_body_index_1;
  const unsigned m_portal_index_0;
  const unsigned m_portal_index_1;
  const bool m_plane_0;
  const bool m_plane_1;

};

class PlanarPortal final
{

public:

  PlanarPortal( const StaticPlane& plane_a, const StaticPlane& plane_b, const Array3i& portal_multiplier );
  explicit PlanarPortal( std::istream& input_stream );

  const StaticPlane& planeA() const;
  const StaticPlane& planeB() const;

  // Returns true if the given point lies in the space covered by this portal
  bool pointInsidePortal( const Vector3s& x ) const;

  // Returns true if the given AABB touches the space covered by this portal
  bool aabbTouchesPortal( const Array3s& min, const Array3s& max, bool& intersecting_plane_idx ) const;

  // Teleports a point that is inside the portal
  void teleportPointInsidePortal( const Vector3s& xin, Vector3s& xout ) const;

  // Teleports a point through a given plane
  void teleportPoint( const Vector3s& xin, const bool intersecting_plane_idx, Vector3s& xout ) const;

  void teleportPointThroughPlaneA( const Vector3s& xin, Vector3s& xout ) const;
  void teleportPointThroughPlaneB( const Vector3s& xin, Vector3s& xout ) const;

  void serialize( std::ostream& output_stream ) const;

private:

  StaticPlane m_plane_a;
  StaticPlane m_plane_b;
  Array3i m_portal_multiplier;

};

#endif
