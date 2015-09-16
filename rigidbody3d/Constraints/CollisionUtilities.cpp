// CollisionUtilities.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "CollisionUtilities.h"

#include <iostream>

static scalar clamp( const scalar& val, const scalar& min, const scalar& max )
{
  if( val < min )
  {
    return min;
  }
  if( val > max )
  {
    return max;
  }
  return val;
}

// xb: Center of mass of box
// Rb: Orientation of box
// wb: Half-widths of box
// p: Point in space to compute closest point on rectangle to
// Given point p, return point q on (or in) OBB b, closest to p
static void computeClosestPointToBox( const Vector3s& xb, const Matrix33sc& Rb, const Vector3s& wb, const Vector3s& p, Vector3s& closest_point )
{
  // For each rect axis, project d onto that axis to get the distance
  // along the axis of d from the rect center. If distance farther than
  // the rect extents, clamp to the box. Step that distance along the axis
  // to get world coordinate

  closest_point = xb;

  for( int i = 0; i < 3; ++i )
  {
    closest_point += clamp( Rb.col( i ).dot( p - xb ), -wb( i ), wb( i ) ) * Rb.col( i );
  }
}

scalar CollisionUtilities::closestPointSegmentSegment( const Vector3s& p1, const Vector3s& q1, const Vector3s& p2, const Vector3s& q2, scalar& s, scalar& t, Vector3s& c1, Vector3s& c2 )
{
  const scalar DIST_EPS{ 1.0e-8 };

  const Vector3s d1{ q1 - p1 }; // Direction vector of segment S1
  const Vector3s d2{ q2 - p2}; // Direction vector of segment S2
  const Vector3s r{ p1 - p2 };
  const scalar a{ d1.dot( d1 ) }; // Squared length of segment S1, always nonnegative
  const scalar e{ d2.dot( d2 ) }; // Squared length of segment S2, always nonnegative
  const scalar f{ d2.dot( r ) };

  // Check if either or both segments degenerate into points
  if( a <= DIST_EPS && e <= DIST_EPS )
  {
    // Both segments degenerate into points
    s = t = 0.0;
    c1 = p1;
    c2 = p2;
    return ( c1 - c2 ).dot( c1 - c2 );
  }
  if( a <= DIST_EPS )
  {
    // First segment degenerates into a point
    s = 0.0;
    t = f / e; // s = 0 => t = (b*s + f) / e = f / e
    t = clamp( t, 0.0, 1.0 );
  }
  else
  {
    const scalar c{ d1.dot( r ) };
    if( e <= DIST_EPS )
    {
      // Second segment degenerates into a point
      t = 0.0;
      s = clamp( -c / a, 0.0, 1.0 ); // t = 0 => s = (b*t - c) / a = -c / a
    }
    else
    {
      // The general nondegenerate case starts here
      const scalar b{ d1.dot( d2 ) };
      const scalar denom{ a * e - b * b }; // Always nonnegative

      // If segments not parallel, compute closest point on L1 to L2, and
      // clamp to segment S1. Else pick arbitrary s (here 0)
      if( denom != 0.0 )
      {
        s = clamp( ( b * f - c * e ) / denom, 0.0, 1.0 );
      }
      else
      {
        s = 0.0;
      }

      // Compute point on L2 closest to S1(s) using
      // t = Dot((P1+D1*s)-P2,D2) / Dot(D2,D2) = (b*s + f) / e
      t = ( b * s + f ) / e;

      // If t in [0,1] done. Else clamp t, recompute s for the new value
      // of t using s = Dot((P2+D2*t)-P1,D1) / Dot(D1,D1)= (t*b - c) / a
      // and clamp s to [0, 1]
      if( t < 0.0 )
      {
        t = 0.0;
        s = clamp( -c / a, 0.0, 1.0 );
      }
      else if( t > 1.0 )
      {
        t = 1.0;
        s = clamp( (b - c) / a, 0.0, 1.0 );
      }
    }
  }

  c1 = p1 + d1 * s;
  c2 = p2 + d2 * t;
  return ( c1 - c2 ).dot( c1 - c2 );
}

scalar CollisionUtilities::closestPointPointSegment( const Vector3s& c, const Vector3s& a, const Vector3s& b, scalar& t )
{
  const Vector3s ab{ b - a };
  // Project c onto ab, computing parameterized position d(t) = a + t*(b – a)
  t = (c - a).dot( ab ) / ab.dot( ab );
  // If outside segment, clamp t (and therefore d) to the closest endpoint
  if( t < 0.0 )
  {
    t = 0.0;
  }
  if( t > 1.0 )
  {
    t = 1.0;
  }
  // Compute projected position from the clamped t
  const Vector3s& d{ a + t * ab };
  
  return ( c - d ).squaredNorm();
}

void CollisionUtilities::computeBoxSphereActiveSet( const Vector3s& xb, const Matrix33sc& Rb, const Vector3s& wb, const Vector3s& xs, const scalar& rs, std::vector<Vector3s>& p, std::vector<Vector3s>& n )
{
  // Rotation matrix should be orthonormal and orientation preserving
  assert( ( Rb * Rb.transpose() - Matrix33sc::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( Rb.determinant() - 1.0 ) <= 1.0e-6 );
  // All half-widths should be positive
  assert( ( wb.array() > 0.0 ).all() );

  Vector3s closest_point;
  computeClosestPointToBox( xb, Rb, wb, xs, closest_point );
  const scalar dist_squared{ ( closest_point - xs ).squaredNorm() };
  
  // If the closest point is outside the sphere's radius, there is no collision
  if( dist_squared > rs * rs )
  {
    return;
  }

  // If we are inside the box, throw an error
  // TODO: Handle this case
  if( dist_squared <= 1.0e-9 )
  {
    std::cerr << "Error, degenerate case in Box-Sphere collision detection. Implement degenerate CD code or decrease the time step! Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  
  // Compute the collision normal pointing from the sphere to the box
  const Vector3s normal{ ( closest_point - xs ) / sqrt( dist_squared ) };
  
  // Return the collision
  p.push_back( closest_point );
  n.push_back( normal );
}
