//  StapleStapleUtilities.cpp
//
// Breannan Smith
// Last updated: 09/16/2015

#include "StapleStapleUtilities.h"
#include "CollisionUtilities.h"
#include "RigidBody3D/Geometry/RigidBodyStaple.h"

#include <set>
#include <iostream>

//static bool edgesAreParallel( const Vector3s& v0, const Vector3s& v1, const Vector3s& v2, const Vector3s& v3, const scalar& eps )
//{
//  const Vector3s n0{ (v1-v0).normalized() };
//  const Vector3s n1{ (v3-v2).normalized() };
//
//  const scalar cnorm{ n0.cross( n1 ).norm() };
//
//  if( cnorm < eps )
//  {
//    return true;
//  }
//  return false;
//}

bool StapleStapleUtilities::isActive( const Vector3s& cm0, const Matrix33sr& R0, const RigidBodyStaple& staple0, const Vector3s& cm1, const Matrix33sr& R1, const RigidBodyStaple& staple1 )
{
  // Sum of the radii of each staple
  assert( staple0.r() > 0.0 ); assert( staple1.r() > 0.0 );
  const scalar rad_sum_squared{ ( staple0.r() + staple1.r() ) * ( staple0.r() + staple1.r() ) };

  // Dummy variables for the function call
  scalar s;
  scalar t;
  Vector3s c1;
  Vector3s c2;
  
  // For each edge of the first body
  for( int e0 = 0; e0 < 3; ++e0 )
  {
    const Vector3s v0{ R0 * staple0.points()[ e0 + 0 ] + cm0 };
    const Vector3s v1{ R0 * staple0.points()[ e0 + 1 ] + cm0 };
    // For each edge of the second body
    for( int e1 = 0; e1 < 3; ++e1 )
    {
      const Vector3s v2{ R1 * staple1.points()[ e1 + 0 ] + cm1 };
      const Vector3s v3{ R1 * staple1.points()[ e1 + 1 ] + cm1 };
      const scalar dd{ CollisionUtilities::closestPointSegmentSegment( v0, v1, v2, v3, s, t, c1, c2 ) };
      // Collision if squared distance is less than or equal to the sum of the radii squared
      if( dd <= rad_sum_squared )
      {
        return true;
      }
    }
  }

  return false;
}

void StapleStapleUtilities::computeConstraints( const Vector3s& cm0, const Matrix33sr& R0, const RigidBodyStaple& staple0,
                                                const Vector3s& cm1, const Matrix33sr& R1, const RigidBodyStaple& staple1,
                                                std::vector<Vector3s>& p, std::vector<Vector3s>& n )
{
  std::cerr << "Fix up StapleStapleUtilities::computeConstraints to get n in correct direction" << std::endl;
  std::exit( EXIT_FAILURE );
//  // Sum of the radii of each staple
//  assert( staple0.r() > 0.0 ); assert( staple1.r() > 0.0 );
//  const scalar rad_sum_squared = ( staple0.r() + staple1.r() ) * ( staple0.r() + staple1.r() );
//
//  std::set<StapleStapleCollision> staple_collisions;
//
//  // For each edge of the first body
//  for( int e0 = 0; e0 < 3; ++e0 )
//  {
//    const Vector3s v0 = R0 * staple0.points()[ e0 + 0 ] + cm0;
//    const Vector3s v1 = R0 * staple0.points()[ e0 + 1 ] + cm0;
//    // For each edge of the second body
//    for( int e1 = 0; e1 < 3; ++e1 )
//    {
//      const Vector3s v2 = R1 * staple1.points()[ e1 + 0 ] + cm1;
//      const Vector3s v3 = R1 * staple1.points()[ e1 + 1 ] + cm1;
//      
//      // Determine if the edges are parallel
//      if( edgesAreParallel( v0, v1, v2, v3, 1.0e-8 ) )
//      {
//        // Staple 0's edge vs. staple 1's first vertex
//        {
//          scalar u;
//          const scalar dd = CollisionUtilities::closestPointPointSegment( v2, v0, v1, u );
//          if( dd <= rad_sum_squared )
//          {
//            staple_collisions.insert( StapleStapleCollision( e0, e1, u, 0.0 ) );
//          }
//        }
//        // Staple 0's edge vs. staple 1's second vertex
//        {
//          scalar u;
//          const scalar dd = CollisionUtilities::closestPointPointSegment( v3, v0, v1, u );
//          if( dd <= rad_sum_squared )
//          {
//            staple_collisions.insert( StapleStapleCollision( e0, e1, u, 1.0 ) );
//          }
//        }
//        // Staple 1's edge vs. staple 0's first vertex
//        {
//          scalar u;
//          const scalar dd = CollisionUtilities::closestPointPointSegment( v0, v2, v3, u );
//          if( dd <= rad_sum_squared )
//          {
//            staple_collisions.insert( StapleStapleCollision( e0, e1, 0.0, u ) );
//          }
//        }
//        // Staple 1's edge vs. staple 0's second vertex
//        {
//          scalar u;
//          const scalar dd = CollisionUtilities::closestPointPointSegment( v1, v2, v3, u );
//          if( dd <= rad_sum_squared )
//          {
//            staple_collisions.insert( StapleStapleCollision( e0, e1, 1.0, u ) );
//          }
//        }
//        continue;
//      }
//
//      scalar s;
//      scalar t;
//      Vector3s c1;
//      Vector3s c2;
//      const scalar dd = CollisionUtilities::closestPointSegmentSegment( v0, v1, v2, v3, s, t, c1, c2 );
//      // Collision if squared distance is less than or equal to the sum of the radii squared
//      if( dd <= rad_sum_squared )
//      {
//        staple_collisions.insert( StapleStapleCollision( e0, e1, s, t ) );
//      }
//    }
//  }
//  
//  for( std::set<StapleStapleCollision>::iterator it= staple_collisions.begin(); it != staple_collisions.end(); ++it )
//  {
//    // Compute the closest points on each segment
//    const Vector3s v0 =  R0 * ( ( 1.0 - it->s ) * staple0.points()[ it->e0 ] + it->s * staple0.points()[ it->e0 + 1 ] ) + cm0;
//    const Vector3s v1 =  R1 * ( ( 1.0 - it->t ) * staple1.points()[ it->e1 ] + it->t * staple1.points()[ it->e1 + 1 ] ) + cm1;
//
//    // Compute the collision normal
//    const Vector3s normal = ( v1 - v0 ).normalized();
//
//    // Percentage of distance along v1 - v0 at which to compute the contact point
//    const scalar u = staple0.r() / ( staple0.r() + staple1.r() );
//
//    // Contact point
//    const Vector3s point = ( 1.0 - u )  * v0 + u * v1;
//
//    p.push_back( point );
//    n.push_back( normal );
//  }
}

void StapleStapleUtilities::computeStapleHalfPlaneActiveSet( const Vector3s& cm, const Matrix33sr& R, const RigidBodyStaple& staple,
                                                             const Vector3s& x0, const Vector3s& n, std::vector<int>& points )
{
  // For each vertex of the staple
  for( int i = 0; i < 4; ++i )
  {
    const Vector3s v{ R * staple.points()[ i ] + cm };
    // Compute the distance from the vertex to the halfplane
    const scalar d{ n.dot( v - x0 ) - staple.r() };
    // If the distance is not positive, we have a collision!
    if( d <= 0.0 )
    {
      points.emplace_back( i );
    }
  }
}
