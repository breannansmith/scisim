// CircleBoxTools.cpp
//
// Breannan Smith
// Last updated: 04/06/2016

#include "CircleBoxTools.h"

bool CircleBoxTools::isActive( const Vector2s& x0, const scalar& r0, const Vector2s& x1, const scalar& theta1, const Vector2s& r1, Vector2s& n, Vector2s& p )
{
  const Matrix22sc R{ Eigen::Rotation2D<scalar>{ theta1 } };

  Vector2s x_circle{ R.transpose() * ( x0 - x1 ) };
  const bool invert_x{ x_circle.x() < 0.0 };
  const bool invert_y{ x_circle.y() < 0.0 };
  if( invert_x ) { x_circle.x() *= -1.0; }
  if( invert_y ) { x_circle.y() *= -1.0; }

  scalar pen_depth;

  // Right or corner region
  if( r1.x() * x_circle.y() < r1.y() * x_circle.x() )
  {
    // Right region
    if( x_circle.y() <= r1.y() )
    {
      pen_depth = x_circle.x() - r0 - r1.x();
      if( pen_depth > 0.0 )
      {
        return false;
      }
      n << 1.0, 0.0;
    }
    // Corner region
    else
    {
      n = x_circle - r1;
      pen_depth = n.squaredNorm();
      if( pen_depth > r0 * r0 )
      {
        return false;
      }
      pen_depth = sqrt( pen_depth );
      n /= pen_depth;
      pen_depth -= r0;
      #ifndef NDEBUG
      {
        const scalar x_depth{ x_circle.x() - r0 - r1.x() };
        assert( pen_depth > x_depth );
        const scalar y_depth{ x_circle.y() - r0 - r1.y() };
        assert( pen_depth > y_depth );
      }
      #endif
    }
  }
  // Top or corner region
  else
  {
    // Top region
    if( x_circle.x() <= r1.x() )
    {
      // Penetration test
      pen_depth = x_circle.y() - r0 - r1.y();
      if( pen_depth > 0.0 )
      {
        return false;
      }
      n << 0.0, 1.0;
    }
    else
    {
      n = x_circle - r1;
      pen_depth = n.squaredNorm();
      if( pen_depth > r0 * r0 )
      {
        return false;
      }
      pen_depth = sqrt( pen_depth );
      n /= pen_depth;
      pen_depth -= r0;
      #ifndef NDEBUG
      {
        const scalar x_depth{ x_circle.x() - r0 - r1.x() };
        assert( pen_depth > x_depth );
        const scalar y_depth{ x_circle.y() - r0 - r1.y() };
        assert( pen_depth > y_depth );
      }
      #endif
    }
  }

  assert( fabs( n.norm() - 1.0 ) <= 1.0e-9 );
  assert( pen_depth <= 0.0 );

  p = x_circle - ( r0 + 0.5 * pen_depth ) * n;

  if( invert_x )
  {
    n.x() *= -1.0;
    p.x() *= -1.0;
  }

  if( invert_y )
  {
    n.y() *= -1.0;
    p.y() *= -1.0;
  }

  n = R * n;
  p = R * p + x1;

  assert( fabs( n.norm() - 1.0 ) <= 1.0e-9 );

  return true;
}
