// PlanarPortal.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef PLANAR_PORTAL_H
#define PLANAR_PORTAL_H

#include "ball2d/StaticGeometry/StaticPlane.h"
#include "scisim/Math/MathDefines.h"

// TODO: Move out of here
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

// TODO: Move out of here
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

  PlanarPortal( const StaticPlane& plane_a, const StaticPlane& plane_b, const scalar& velocity, const scalar& bounds );
  explicit PlanarPortal( std::istream& input_stream );

  const StaticPlane& planeA() const;
  const StaticPlane& planeB() const;

  const scalar& bounds() const;

  // Current translated position
  Vector2s transformedAx() const;
  Vector2s transformedBx() const;

  // Returns true if the given point lies in the space covered by this portal
  bool pointInsidePortal( const Vector2s& x ) const;

  // Returns true if the given ball touches the space covered by this portal
  bool ballTouchesPortal( const Vector2s& x, const scalar& r, bool& intersecting_plane_idx ) const;

  // Teleports a point that is inside the portal
  void teleportPointInsidePortal( const Vector2s& xin, Vector2s& xout ) const;

  // Teleports a ball
  void teleportBall( const Vector2s& xin, const scalar& r, Vector2s& xout ) const;

  void teleportPointThroughPlaneA( const Vector2s& xin, Vector2s& xout ) const;
  void teleportPointThroughPlaneB( const Vector2s& xin, Vector2s& xout ) const;

  void serialize( std::ostream& output_stream ) const;

  void updateMovingPortals( const scalar& t );

  bool isLeesEdwards() const;

  Vector2s getKinematicVelocityOfBall( const Vector2s& x, const scalar& r ) const;
  Vector2s getKinematicVelocityOfPoint( const Vector2s& x ) const;

private:

  StaticPlane m_plane_a;
  StaticPlane m_plane_b;

  // For Lees-Edwards boundary conditions
  scalar m_v;
  scalar m_bounds;

  scalar m_dx;

};

#endif
