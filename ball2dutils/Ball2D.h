// Ball2D.h
//
// Breannan Smith
// Last updated: 09/05/2015

#ifndef BALL_2D_H
#define BALL_2D_H

#include "scisim/Math/MathDefines.h"

class Ball2D
{

public:

  Ball2D( const Vector2s& x, const Vector2s& v, const scalar& m, const scalar& r, const bool fixed );

  const Vector2s& x() const;
  const Vector2s& v() const;
  const scalar& m() const;
  const scalar& r() const;
  bool fixed() const;

private:

  Vector2s m_x;
  Vector2s m_v;
  scalar m_m;
  scalar m_r;
  bool m_fixed;

};

#endif
