// StaticCylinder.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef STATIC_CYLINDER_H
#define STATIC_CYLINDER_H

#include "scisim/Math/MathDefines.h"

class StaticCylinder final
{

public:

  StaticCylinder( const Vector3s& x, const Vector3s& axis, const scalar& radius );
  explicit StaticCylinder( std::istream& input_stream );

  const Vector3s& x() const;

  void setOrientation( const Vector3s& axis, const scalar& theta );

  Quaternions R() const;

  const Vector3s& v() const;

  Vector3s& omega();
  const Vector3s& omega() const;

  const scalar& r() const;

  const Vector3s& axis() const;

  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

private:

  // Point on the cylinder's axis
  Vector3s m_x;
  // Cylinder's axis
  Vector3s m_n;
  // Rotation about the cylinder's axis
  scalar m_theta;
  // Linear velocity of point x
  Vector3s m_v;
  // Angular velocity about x
  Vector3s m_omega;
  // Radius of the cylinder.
  scalar m_r;

};

#endif
