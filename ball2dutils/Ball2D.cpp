// Ball2D.cpp
//
// Breannan Smith
// Last updated: 09/05/2015

#include "Ball2D.h"

Ball2D::Ball2D( const Vector2s& x, const Vector2s& v, const scalar& m, const scalar& r, const bool fixed )
: m_x(x)
, m_v(v)
, m_m(m)
, m_r(r)
, m_fixed(fixed)
{}

const Vector2s& Ball2D::x() const
{
  return m_x;
}

const Vector2s& Ball2D::v() const
{
  return m_v;
}

const scalar& Ball2D::m() const
{
  return m_m;
}

const scalar& Ball2D::r() const
{
  return m_r;
}

bool Ball2D::fixed() const
{
  return m_fixed;
}
