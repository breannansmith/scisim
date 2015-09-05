// Ball2D.h
//
// Breannan Smith
// Last updated: 09/05/2015

#ifndef BALL_2D_H
#define BALL_2D_H

#include "SCISim/Math/MathDefines.h"

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

  const Vector2s m_x;
  const Vector2s m_v;
  const scalar m_m;
  const scalar m_r;
  const bool m_fixed;

};

#endif
