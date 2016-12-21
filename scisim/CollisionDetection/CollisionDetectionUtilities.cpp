#include "CollisionDetectionUtilities.h"

Vector3s CollisionDetectionUtilities::computeCCDQuadraticCoeffs( const Vector2s& q0a, const Vector2s& q1a, const scalar& ra, const Vector2s& q0b, const Vector2s& q1b, const scalar& rb )
{
  Vector3s coeffs;

  const Vector2s q0delta{ q0a - q0b };
  const Vector2s q1q0delta{ q1a - q1b - q0delta };

  // Constant coefficient
  using std::pow;
  coeffs(0) = q0delta.dot(q0delta) - pow( ra + rb, 2 );

  // Linear coefficient
  coeffs(1) = 2.0 * q0delta.dot(q1q0delta);

  // Quadratic coefficient
  coeffs(2) = (q1q0delta).dot(q1q0delta);
  assert( coeffs(2) >= 0.0 );

  return coeffs;
}

static scalar firstRootOfQuadratic( const scalar& a, const scalar& b, const scalar& c, const scalar& dscr_sqrt )
{
  scalar root;
  if( b >= 0.0 )
  {
    root = ( -b - dscr_sqrt ) / ( 2.0 * a );
  }
  else
  {
    root = ( 2.0 * c ) / ( -b + dscr_sqrt );
  }
  return root;
}

static scalar secondRootOfQuadratic( const scalar& a, const scalar& b, const scalar& c, const scalar& dscr_sqrt )
{
  scalar root;
  if( b >= 0.0 )
  {
    root = ( 2.0 * c ) / ( -b - dscr_sqrt );
  }
  else
  {
    root = ( -b + dscr_sqrt ) / ( 2.0 * a );
  }
  return root;
}

std::pair<bool,scalar> CollisionDetectionUtilities::ballBallCCDCollisionHappens( const Vector3s& c )
{
  std::pair<bool,scalar> collision;

  if( c(2) != 0.0 )
  {
    const scalar c1c1{ c(1) * c(1) };
    const scalar fc2c0{ 4 * c(2) * c(0) };

    // No roots
    if( c1c1 < fc2c0 )
    {
      collision.first = false;
    }
    else
    {
      using std::sqrt;
      const scalar dscr_sqrt = sqrt( c1c1 - fc2c0 );

      const scalar root1 = secondRootOfQuadratic( c(2), c(1), c(0), dscr_sqrt );

      if( root1 < 0.0 )
      {
        // Collision is backward in time
        collision.first = false;
      }
      else
      {
        const scalar root0 = firstRootOfQuadratic( c(2), c(1), c(0), dscr_sqrt );
        assert( root0 <= root1 );

        if( root0 > 1.0 )
        {
          // Collision is forward in time
          collision.first = false;
        }
        else
        {
          collision.first = true;
          using std::max;
          collision.second = max( 0.0, root0 );
        }
      }
    }
  }
  else
  {
    assert( c(1) == 0.0 );

    if( c(0) <= 0.0 )
    {
      collision.first = true;
      collision.second = 0.0;
    }
    else
    {
      collision.first = false;
    }
  }
  
  return collision;
}
