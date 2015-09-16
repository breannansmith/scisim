// FrictionUtilities.cpp
//
// Breannan Smith
// Last updated: 09/16/2015

#include "FrictionUtilities.h"

#ifndef NDEBUG
// Unsigned angle
scalar angleBetweenVectors( const Vector3s& v0, const Vector3s& v1 )
{
  const scalar s = v0.cross( v1 ).norm();
  const scalar c = v0.dot( v1 );
  return atan2( s, c );
}
#endif

// TODO: This doesn't handle <0,0,0>
Vector3s FrictionUtilities::orthogonalVector( const Vector3s& n )
{
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 ); // TODO: Remove this
  // Chose the most orthogonal direction among x, y, z
  Vector3s orthog{ fabs(n.x()) <= fabs(n.y()) && fabs(n.x()) <= fabs(n.z()) ? Vector3s::UnitX() : fabs(n.y()) <= fabs(n.z()) ? Vector3s::UnitY() : Vector3s::UnitZ() };
  assert( orthog.cross(n).squaredNorm() != 0.0 ); // New vector shouldn't be parallel to the input
  // Project out any non-orthogonal component
  orthog -= n.dot( orthog ) * n;
  assert( orthog.norm() != 0.0 );
  return orthog.normalized();
}

void FrictionUtilities::generateOrthogonalVectors( const Vector3s& n, std::vector<Vector3s>& vectors, const Vector3s& suggested_tangent )
{
  if( vectors.empty() )
  {
    return;
  }
  assert( ( suggested_tangent.cross( n ) ).squaredNorm() != 0.0 );

  // Make sure the first vector is orthogonal to n and unit length
  vectors[0] = ( suggested_tangent - n.dot( suggested_tangent ) * n ).normalized();
  assert( fabs( vectors[0].norm() - 1.0 ) <= 1.0e-10 );
  assert( fabs( n.dot( vectors[0] ) ) <= 1.0e-10 );

  // Generate the remaining vectors by rotating the first vector about n
  const scalar dtheta{ 2.0 * MathDefines::PI<scalar>() / scalar( vectors.size() ) };
  for( std::vector<Vector3s>::size_type i = 1; i < vectors.size(); ++i )
  {
    vectors[i] = Eigen::AngleAxis<scalar>{ i * dtheta, n } * vectors[0];
    assert( fabs( vectors[i].norm() - 1.0 ) <= 1.0e-10 );
    assert( fabs( n.dot( vectors[i] ) ) <= 1.0e-10 );
  }

  #ifndef NDEBUG
  if( vectors.size() == 1 )
  {
    return;
  }
  // Check that the angle between each vector is the one we expect
  for( std::vector<Vector3s>::size_type i = 0; i < vectors.size(); ++i )
  {
    assert( fabs( angleBetweenVectors( vectors[i], vectors[( i + 1 ) % vectors.size()] ) - dtheta ) < 1.0e-6 );
  }
  #endif
}
