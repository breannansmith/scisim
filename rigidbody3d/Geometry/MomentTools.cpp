// MomentTools.cpp
//
// Breannan Smith
// Last updated: 09/14/2015

#include "MomentTools.h"

#include <iostream>

#include <Eigen/Eigenvalues>

#include "SCISim/Math/MathUtilities.h"

static void diagonalizeInertiaTensor( const Matrix3s& I, Matrix3s& R0, Vector3s& I0 )
{
  // Inertia tensor should by symmetric
  assert( ( I - I.transpose() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  // Inertia tensor should have positive determinant
  assert( I.determinant() > 0.0 );

  // Compute the eigenvectors and eigenvalues of the input matrix
  const Eigen::SelfAdjointEigenSolver<Matrix3s> es{ I };

  // Check for errors
  if( es.info() == Eigen::NumericalIssue )
  {
    std::cerr << "Warning, failed to compute eigenvalues of inertia tensor due to Eigen::NumericalIssue" << std::endl;
  }
  else if( es.info() == Eigen::NoConvergence )
  {
    std::cerr << "Warning, failed to compute eigenvalues of inertia tensor due to Eigen::NoConvergence" << std::endl;
  }
  else if( es.info() == Eigen::InvalidInput )
  {
    std::cerr << "Warning, failed to compute eigenvalues of inertia tensor due to Eigen::InvalidInput" << std::endl;
  }
  assert( es.info() == Eigen::Success );

  // Save the eigenvectors and eigenvalues
  I0 = es.eigenvalues();
  assert( ( I0.array() > 0.0 ).all() );
  assert( I0.x() <= I0.y() );
  assert( I0.y() <= I0.z() );
  R0 = es.eigenvectors();
  assert( fabs( fabs( R0.determinant() ) - 1.0 ) <= 1.0e-6 );

  // Ensure that we have an orientation preserving transform
  if( R0.determinant() < 0.0 )
  {
    R0.col( 0 ) *= -1.0;
  }
}

namespace MomentTools
{

// TODO: most of this function can be vectorized
void computeMoments( const Matrix3Xsc& vertices, const Matrix3Xuc& indices, scalar& mass, Vector3s& I, Vector3s& center, Matrix3s& R )
{
  assert( ( indices.array() < unsigned( vertices.cols() ) ).all() );

  const scalar oneDiv6{ 1.0 / 6.0 };
  const scalar oneDiv24{ 1.0 / 24.0 };
  const scalar oneDiv60{ 1.0 / 60.0 };
  const scalar oneDiv120{ 1.0 / 120.0 };

  // order:  1, x, y, z, x^2, y^2, z^2, xy, yz, zx
  VectorXs integral{ VectorXs::Zero( 10 ) };

  for( int i = 0; i < indices.cols(); ++i )
  {
    // Copy the vertices of triangle i
    const Vector3s v0{ vertices.col( indices( 0, i ) ) };
    const Vector3s v1{ vertices.col( indices( 1, i ) ) };
    const Vector3s v2{ vertices.col( indices( 2, i ) ) };
    
    // Compute a normal for the current triangle
    const Vector3s N{ ( v1 - v0 ).cross( v2 - v0 ) };

    // Compute the integral terms
    scalar tmp0{ v0.x() + v1.x() };
    scalar tmp1{ v0.x() * v0.x() };
    scalar tmp2{ tmp1 + v1.x() * tmp0 };
    const scalar f1x{ tmp0 + v2.x() };
    const scalar f2x{ tmp2 + v2.x() * f1x };
    const scalar f3x{ v0.x() * tmp1 + v1.x() * tmp2 + v2.x() * f2x };
    const scalar g0x{ f2x + v0.x() * ( f1x + v0.x() ) };
    const scalar g1x{ f2x + v1.x() * ( f1x + v1.x() ) };
    const scalar g2x{ f2x + v2.x() * ( f1x + v2.x() ) };
    
    tmp0 = v0.y() + v1.y();
    tmp1 = v0.y() * v0.y();
    tmp2 = tmp1 + v1.y() * tmp0;
    const scalar f1y{ tmp0 + v2.y() };
    const scalar f2y{ tmp2 + v2.y() * f1y };
    const scalar f3y{ v0.y() * tmp1 + v1.y() * tmp2 + v2.y() * f2y };
    const scalar g0y{ f2y + v0.y() * ( f1y + v0.y() ) };
    const scalar g1y{ f2y + v1.y() * ( f1y + v1.y() ) };
    const scalar g2y{ f2y + v2.y() * ( f1y + v2.y() ) };
    
    tmp0 = v0.z() + v1.z();
    tmp1 = v0.z()*v0.z();
    tmp2 = tmp1 + v1.z()*tmp0;
    const scalar f1z{ tmp0 + v2.z() };
    const scalar f2z{ tmp2 + v2.z() * f1z };
    const scalar f3z{ v0.z() * tmp1 + v1.z() * tmp2 + v2.z() * f2z };
    const scalar g0z{ f2z + v0.z() * ( f1z + v0.z() ) };
    const scalar g1z{ f2z + v1.z() * ( f1z + v1.z() ) };
    const scalar g2z{ f2z + v2.z() * ( f1z + v2.z() ) };
    
    // Update integrals
    integral(0) += N.x() * f1x;
    integral(1) += N.x() * f2x;
    integral(2) += N.y() * f2y;
    integral(3) += N.z() * f2z;
    integral(4) += N.x() * f3x;
    integral(5) += N.y() * f3y;
    integral(6) += N.z() * f3z;
    integral(7) += N.x() * ( v0.y() * g0x + v1.y() * g1x + v2.y() * g2x );
    integral(8) += N.y() * ( v0.z() * g0y + v1.z() * g1y + v2.z() * g2y );
    integral(9) += N.z() * ( v0.x() * g0z + v1.x() * g1z + v2.x() * g2z );
  }
  
  integral(0) *= oneDiv6;
  integral(1) *= oneDiv24;
  integral(2) *= oneDiv24;
  integral(3) *= oneDiv24;
  integral(4) *= oneDiv60;
  integral(5) *= oneDiv60;
  integral(6) *= oneDiv60;
  integral(7) *= oneDiv120;
  integral(8) *= oneDiv120;
  integral(9) *= oneDiv120;
  
  // Mass
  mass = integral(0);
  
  // Center of mass
  center = Vector3s( integral(1), integral(2), integral(3) )/mass;
  
  // Inertia relative to world origin
  R(0,0) = integral(5) + integral(6);
  R(0,1) = -integral(7);
  R(0,2) = -integral(9);
  R(1,0) = R(0,1);
  R(1,1) = integral(4) + integral(6);
  R(1,2) = -integral(8);
  R(2,0) = R(0,2);
  R(2,1) = R(1,2);
  R(2,2) = integral(4) + integral(5);
  
  // Comptue the inertia relative to the center of mass
  R(0,0) -= mass * ( center.y() * center.y() + center.z() * center.z() );
  R(0,1) += mass * center.x() * center.y();
  R(0,2) += mass * center.z() * center.x();
  R(1,0) = R(0,1);
  R(1,1) -= mass * ( center.z() * center.z() + center.x() * center.x() );
  R(1,2) += mass * center.y() * center.z();
  R(2,0) = R(0,2);
  R(2,1) = R(1,2);
  R(2,2) -= mass * ( center.x() * center.x() + center.y() * center.y() );

  // Diagonalize the inertia tensor
  Matrix3s R0;
  diagonalizeInertiaTensor( R, R0, I );

  // Check that we actually diagonalized the inertia tensor
  assert( ( R0 * Matrix3s( I.asDiagonal() ) * R0.transpose() - R ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( ( Matrix3s( I.asDiagonal() ) - R0.transpose() * R * R0 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  R = R0;

  // All inertias should be positive
  assert( ( I.array() > 0.0 ).all() );
  // Check that we have an orthonormal transformation
  assert( ( R * R.transpose() - Matrix3s::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( R.determinant() - 1.0 ) <= 1.0e-6 );
}

}
