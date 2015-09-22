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

  bool distanceLessThanZero( const Vector2s& plane_dx, const Vector2s& x ) const;
  scalar distanceToPoint( const Vector2s& plane_dx, const Vector2s& x ) const;

  void serialize( std::ostream& output_stream ) const;

private:

  Vector2s m_x;
  Vector2s m_n;
  Vector2s m_t;

};

#endif
