// StaticPlane.h
//
// Breannan Smith
// Last updated: 09/21/2015

// TODO: Store n directly instead of implicitly through a rotation. Using the rotation leads to subtle issues if a user expects a plane to align exactly with a cartesian axis.

#ifndef STATIC_PLANE_H
#define STATIC_PLANE_H

#include "scisim/Math/MathDefines.h"

class StaticPlane final
{

public:

  StaticPlane();
  StaticPlane( const Vector3s& x, const Vector3s& n );
  explicit StaticPlane( std::istream& input_stream );

  Vector3s& x();
  const Vector3s& x() const;

  Quaternions& R();
  const Quaternions& R() const;

  // t0 x n = t1
  const Vector3s n() const;
  const Vector3s t0() const;
  const Vector3s t1() const;

  Vector3s& v();
  const Vector3s& v() const;

  Vector3s& omega();
  const Vector3s& omega() const;

  scalar distanceToPoint( const Vector3s& x ) const;

  void serialize( std::ostream& output_stream ) const;

private:

  Vector3s m_x;
  Quaternions m_R; // Rotation about x from yhat to current normal
  Vector3s m_v; // Linear velocity of point x
  Vector3s m_omega; // Angular velocity about x

};

#endif
