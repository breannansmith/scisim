// PlanarPortal.h
//
// Breannan Smith
// Last updated: 09/05/2015

#ifndef PLANAR_PORTAL_H
#define PLANAR_PORTAL_H

#include "ball2d/StaticGeometry/StaticPlane.h"
#include "scisim/Math/MathDefines.h"

class TeleportedBall final
{

public:

  TeleportedBall( const unsigned body_index, const unsigned portal_index, const bool plane_index );

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

  inline unsigned bodyIndex0() const
  {
    return m_body_index_0;
  }

  inline unsigned bodyIndex1() const
  {
    return m_body_index_1;
  }

  inline unsigned portalIndex0() const
  {
    return m_portal_index_0;
  }

  inline unsigned portalIndex1() const
  {
    return m_portal_index_1;
  }

  inline bool plane0() const
  {
    return m_plane_0;
  }

  inline bool plane1() const
  {
    return m_plane_1;
  }

private:

  // TODO: Make these const
  unsigned m_body_index_0;
  unsigned m_body_index_1;
  unsigned m_portal_index_0;
  unsigned m_portal_index_1;
  bool m_plane_0;
  bool m_plane_1;

};

class PlanarPortal final
{

public:

  PlanarPortal( const StaticPlane& plane_a, const StaticPlane& plane_b, const scalar& va, const scalar& vb, const Vector2s& boundsa, const Vector2s& boundsb );
  PlanarPortal( std::istream& input_stream );

  const StaticPlane& planeA() const;
  const StaticPlane& planeB() const;

  const Vector2s& boundsA() const;
  const Vector2s& boundsB() const;

  // Current translated position
  scalar planeAx() const;
  scalar planeAy() const;
  scalar planeBx() const;
  scalar planeBy() const;

  // Returns true if the given point lies in the space covered by this portal
  bool pointInsidePortal( const Vector2s& x ) const;

  // Returns true if the given ball touches the space covered by this portal
  bool ballTouchesPortal( const Vector2s& x, const scalar& r, bool& intersecting_plane_idx ) const;

  // Teleports a point that is inside the portal
  void teleportPointInsidePortal( const Vector2s& xin, Vector2s& xout ) const;

  // Teleports a ball
  void teleportBall( const Vector2s& xin, const scalar& r, Vector2s& xout ) const;

  //void getTeleportedSamplePoints( const VectorXs& q, const TeleportedCollision& teleported_collision, Vector2s& x0, Vector2s& x1 ) const;

  void teleportPointThroughPlaneA( const Vector2s& xin, Vector2s& xout ) const;
  void teleportPointThroughPlaneB( const Vector2s& xin, Vector2s& xout ) const;

  void serialize( std::ostream& output_stream ) const;

  void updateMovingPortals( const scalar& t );

  bool isLeesEdwards() const;

  Vector2s getKinematicVelocityOfBall( const Vector2s& x, const scalar& r ) const;

private:

  bool ballInPlaneA( const Vector2s& x, const scalar& r ) const;
  bool ballInPlaneB( const Vector2s& x, const scalar& r ) const;

  StaticPlane m_plane_a;
  StaticPlane m_plane_b;

  // For Lees-Edwards boundary conditions
  scalar m_v_a; // Velocity of plane a in the tangent direction
  scalar m_v_b; // Velocity of plane b in the tangent direction
  Vector2s m_bounds_a; // Where to wrap plane a around
  Vector2s m_bounds_b; // Where to wrap plane b around

  scalar m_dx_a;
  scalar m_dx_b;

};

#endif
