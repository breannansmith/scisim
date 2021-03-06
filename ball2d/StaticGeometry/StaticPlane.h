// StaticPlane.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef STATIC_PLANE_H
#define STATIC_PLANE_H

#include "scisim/Math/MathDefines.h"

class StaticPlane final
{

public:

  StaticPlane( const Vector2s& x, const Vector2s& n );
  explicit StaticPlane( std::istream& input_stream );
  StaticPlane();

  Vector2s& x();
  const Vector2s& x() const;

  Vector2s& v();
  const Vector2s& v() const;

  const Vector2s& n() const;

  const Vector2s& t() const;

  bool distanceLessThanZero( const Vector2s& x ) const;

  bool distanceLessThanOrEqualZero( const Vector2s& x, const scalar& r ) const;

  void serialize( std::ostream& output_stream ) const;

private:

  Vector2s m_x;
  Vector2s m_v;
  Vector2s m_n;
  Vector2s m_t;

};

#endif
