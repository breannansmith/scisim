// StaticCylinder.h
//
// Breannan Smith
// Last updated: 09/15/2015

// TODO: Store n directly instead of implicitly through a rotation. Using the rotation leads to subtle issues if a user expects a plane to align exactly with a cartesian axis.

#ifndef STATIC_CYLINDER_H
#define STATIC_CYLINDER_H

#include "scisim/Math/MathDefines.h"

class StaticCylinder
{

public:

  StaticCylinder();
  StaticCylinder( const Vector3s& x, const Vector3s& axis, const scalar& radius );
  StaticCylinder( std::istream& input_stream );

  const Vector3s& x() const;

  Quaternions& R();
  const Quaternions& R() const;

  const Vector3s& v() const;

  Vector3s& omega();
  const Vector3s& omega() const;

  const scalar& r() const;

  const Vector3s axis() const;

  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

private:

  // Point on the cylinder's axis
  Vector3s m_x;
  // Orientation of this cylinder relative to one parallel to yhat
  Quaternions m_R;
  // Linear velocity of point x
  Vector3s m_v;
  // Angular velocity about x
  Vector3s m_omega;
  // Radius of the cylinder.
  scalar m_r;

};

#endif
