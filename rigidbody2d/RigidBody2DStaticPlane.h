// RigidBody2DStaticPlane.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef RIGID_BODY_2D_STATIC_PLANE_H
#define RIGID_BODY_2D_STATIC_PLANE_H

#include "scisim/Math/MathDefines.h"
#include <iosfwd>

class RigidBody2DStaticPlane final
{

public:

  RigidBody2DStaticPlane( const Vector2s& x, const Vector2s& n );
  explicit RigidBody2DStaticPlane( std::istream& input_stream );

  const Vector2s& x() const;
  const Vector2s& n() const;
  const Vector2s& t() const;
  const Vector2s& v() const;
  const scalar& omega() const;

  void setX( const Vector2s& x );
  void setN( const Vector2s& n );
  void setV( const Vector2s& v );
  void setOmega( const scalar& omega );

  bool distanceLessThanZero( const Vector2s& x ) const;
  scalar distanceToPoint( const Vector2s& x ) const;

  void serialize( std::ostream& output_stream ) const;

private:

  // A point on the plane
  Vector2s m_x;
  // The plane's normal
  Vector2s m_n;
  // The plane's tangent
  Vector2s m_t;
  // The velocity of the point m_x
  Vector2s m_v;
  // The angular velocity about the point m_x
  scalar m_omega;

};

#endif
