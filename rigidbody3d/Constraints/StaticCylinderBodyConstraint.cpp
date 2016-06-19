// StaticCylinderBodyConstraint.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "StaticCylinderBodyConstraint.h"

#include "rigidbody3d/StaticGeometry/StaticCylinder.h"

#include <iostream>

#include "FrictionUtilities.h"

StaticCylinderBodyConstraint::StaticCylinderBodyConstraint( const unsigned body_index, const Vector3s& collision_point, const StaticCylinder& cyl, const unsigned cylinder_index, const VectorXs& q )
: m_idx_body( body_index )
, m_r( collision_point - q.segment<3>( 3 * body_index ) )
, m_cyl( cyl )
{}

scalar StaticCylinderBodyConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( 3 * ( m_idx_body + v.size() / 6 ) + 2 < v.size() );
  const Vector3s n{ computeN( q ) };
  return n.dot( computeRelativeVelocity( q, v ) );
}

int StaticCylinderBodyConstraint::impactStencilSize() const
{
  std::cerr << "StaticCylinderBodyConstraint::impactStencilSize" << std::endl;
  std::exit(EXIT_FAILURE);
}

void StaticCylinderBodyConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx_body;
  bodies.second = -1;
}

void StaticCylinderBodyConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
{
  assert( H0.rows() == 3 );
  assert( H0.cols() == 6 );
  assert( H1.rows() == 3 );
  assert( H1.cols() == 6 );

  // Grab the contact normal
  const Vector3s n{ basis.col( 0 ) };
  // Grab the tangent basis
  const Vector3s s{ basis.col( 1 ) };
  const Vector3s t{ basis.col( 2 ) };
  assert( MathUtilities::isRightHandedOrthoNormal( n, s, t, 1.0e-6 ) );

  H0.block<1,3>(0,0) = n;
  H0.block<1,3>(0,3) = m_r.cross( n );

  H0.block<1,3>(1,0) = s;
  H0.block<1,3>(1,3) = m_r.cross( s );

  H0.block<1,3>(2,0) = t;
  H0.block<1,3>(2,3) = m_r.cross( t );
}

bool StaticCylinderBodyConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool StaticCylinderBodyConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool StaticCylinderBodyConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string StaticCylinderBodyConstraint::name() const
{
  return "static_cylinder_body";
}

Vector3s StaticCylinderBodyConstraint::computeN( const VectorXs& q ) const
{
  const Vector3s x_body{ q.segment<3>( 3 * m_idx_body ) };
  const Vector3s axis{ m_cyl.axis() };
  Vector3s n{ - ( x_body - m_cyl.x() - axis.dot( x_body - m_cyl.x() ) * axis ) };
  assert( n.norm() > 0.0 );
  return n / n.norm();
}

VectorXs StaticCylinderBodyConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  // IMPORTANT NOTE: This code has not been updated to treat kinematic boundaries, yet. If this is important to you, please email smith@cs.columbia.edu
  // TODO: Fixing this will require mirroring the structure of StaticPlaneSphereConstraint
  assert( m_cyl.v().array() == 0.0 );
  assert( m_cyl.omega().array() == 0.0 );
  return VectorXs::Zero( 3 );
}

void StaticCylinderBodyConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  // Vector perpendicular to axis in direction of particle
  const Vector3s n{ computeN( q ) };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( n.dot( m_cyl.axis() ) ) <= 1.0e-6 );

  // Compute the relative velocity to use as a direction for the tangent sample
  Vector3s s{ computeRelativeVelocity( q, v ) };
  // If the relative velocity is zero, any vector will do
  if( n.cross( s ).squaredNorm() < 1.0e-9 )
  {
    s = FrictionUtilities::orthogonalVector( n );
  }
  // Otherwise project out the component along the normal and normalize the relative velocity
  else
  {
    s = ( s - s.dot( n ) * n ).normalized();
  }
  // Invert the tangent vector in order to oppose
  s *= -1.0;

  // Create a second orthogonal sample in the tangent plane
  const Vector3s t{ n.cross( s ).normalized() }; // Don't need to normalize but it won't hurt

  assert( MathUtilities::isRightHandedOrthoNormal( n, s, t, 1.0e-6 ) );
  basis.resize( 3, 3 );
  basis.col( 0 ) = n;
  basis.col( 1 ) = s;
  basis.col( 2 ) = t;
}

VectorXs StaticCylinderBodyConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( 3 * ( m_idx_body + v.size() / 6 ) + 2 < v.size() );

  const unsigned nbodies{ static_cast<unsigned>( v.size() / 6 ) };

  // v + omega x r
  return v.segment<3>( 3 * m_idx_body ) + v.segment<3>( 3 * ( nbodies + m_idx_body ) ).cross( m_r );
}

void StaticCylinderBodyConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx_body = idx;
}

