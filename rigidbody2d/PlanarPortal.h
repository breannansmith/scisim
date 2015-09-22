// PlanarPortal.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef PLANAR_PORTAL_H
#define PLANAR_PORTAL_H

#include "RigidBody2DStaticPlane.h"
#include "scisim/Math/MathDefines.h"

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

  PlanarPortal( const RigidBody2DStaticPlane& plane_a, const RigidBody2DStaticPlane& plane_b );
  PlanarPortal( const RigidBody2DStaticPlane& plane_a, const RigidBody2DStaticPlane& plane_b, const scalar& va, const scalar& vb, const Vector2s& boundsa, const Vector2s& boundsb );
  explicit PlanarPortal( std::istream& input_stream );

  const RigidBody2DStaticPlane& planeA() const;
  const RigidBody2DStaticPlane& planeB() const;

  const Vector2s& boundsA() const;
  const Vector2s& boundsB() const;

  const scalar& vA() const;
  const scalar& vB() const;

  const scalar& deltaA() const;
  const scalar& deltaB() const;

  // Current translated position
  scalar planeAx() const;
  scalar planeAy() const;
  scalar planeBx() const;
  scalar planeBy() const;

  // Returns true if the given point lies in the space covered by this portal
  bool pointInsidePortal( const Vector2s& x ) const;

  // Returns true if the given AABB touches the space covered by this portal
  bool aabbTouchesPortal( const Array2s& min, const Array2s& max, bool& intersecting_plane_idx ) const;

  // Teleports a point that is inside the portal
  void teleportPointInsidePortal( const Vector2s& xin, Vector2s& xout ) const;

  // Teleports a body through the given plane
  void teleportPoint( const Vector2s& xin, const bool intersecting_plane_idx, Vector2s& xout ) const;

  void teleportPointThroughPlaneA( const Vector2s& xin, Vector2s& xout ) const;
  void teleportPointThroughPlaneB( const Vector2s& xin, Vector2s& xout ) const;

  void serialize( std::ostream& output_stream ) const;

  void updateMovingPortals( const scalar& t );

  bool isLeesEdwards() const;

  Vector2s getKinematicVelocityOfAABB( const Array2s& min, const Array2s& max ) const;

private:

  bool aabbInPlaneA( const Array2s& min, const Array2s& max ) const;
  bool aabbInPlaneB( const Array2s& min, const Array2s& max ) const;

  RigidBody2DStaticPlane m_plane_a;
  RigidBody2DStaticPlane m_plane_b;

  // For Lees-Edwards boundary conditions
  scalar m_v_a; // Velocity of plane a in the tangent direction
  scalar m_v_b; // Velocity of plane b in the tangent direction
  Vector2s m_bounds_a; // Where to wrap plane a around
  Vector2s m_bounds_b; // Where to wrap plane b around

  scalar m_dx_a;
  scalar m_dx_b;

};

#endif
