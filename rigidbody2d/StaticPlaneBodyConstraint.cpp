// StaticPlaneBodyConstraint.cpp
//
// Breannan Smith
// Last updated: 01/08/2016

#include "StaticPlaneBodyConstraint.h"

#include "scisim/Math/MathUtilities.h"

#include "RigidBody2DStaticPlane.h"

StaticPlaneBodyConstraint::StaticPlaneBodyConstraint( const unsigned body_idx, const Vector2s& body_space_arm, const RigidBody2DStaticPlane& plane, const unsigned plane_idx )
: m_idx_body( body_idx )
, m_body_r( body_space_arm )
, m_plane( plane )
, m_idx_plane( plane_idx )
{}

scalar StaticPlaneBodyConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  return m_plane.n().dot( computeRelativeVelocity( q, v ) );
}

int StaticPlaneBodyConstraint::impactStencilSize() const
{
  return 3;
}

void StaticPlaneBodyConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx_body;
  bodies.second = -1;
}

void StaticPlaneBodyConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx_body;
  bodies.second = -1;
}

unsigned StaticPlaneBodyConstraint::getStaticObjectIndex() const
{
  return m_idx_plane;
}

void StaticPlaneBodyConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
{
  assert( H0.rows() == 2 ); assert( H0.cols() == 3 );
  assert( H1.rows() == 2 ); assert( H1.cols() == 3 );
  assert( ( basis * basis.transpose() - MatrixXXsc::Identity( 2, 2 ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-6 );

  // Grab the contact normal
  const Vector2s n{ basis.col( 0 ) };
  // Grab the tangent basis
  const Vector2s t{ basis.col( 1 ) };

  const Vector2s r{ Eigen::Rotation2D<scalar>{ q( 3 * m_idx_body + 2 ) } * m_body_r };

  // Format for H:
  //   n^T  r x n
  //   t^T  r x t

  H0.block<1,2>(0,0) = n;
  H0(0,2) = MathUtilities::cross( r, n );

  H0.block<1,2>(1,0) = t;
  H0(1,2) = MathUtilities::cross( r, t );
}

bool StaticPlaneBodyConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool StaticPlaneBodyConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool StaticPlaneBodyConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string StaticPlaneBodyConstraint::name() const
{
  return "static_plane_body";
}

Vector2s StaticPlaneBodyConstraint::computePlaneCollisionPointVelocity( const VectorXs& q ) const
{
  // TODO: Add support for collisions with kinematically scripted moving planes
  assert( ( m_plane.v().array() == 0.0 ).all() );
  assert( m_plane.omega() == 0.0 );
  return Vector2s::Zero();
}

void StaticPlaneBodyConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  assert( fabs( m_plane.n().norm() - 1.0 ) <= 1.0e-9 );
  const Vector2s t{ -m_plane.n().y(), m_plane.n().x() };
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-9 ); assert( fabs( m_plane.n().dot( t ) ) <= 1.0e-9 );

  basis.resize( 2, 2 );
  basis.col( 0 ) = m_plane.n();
  basis.col( 1 ) = t;
}

VectorXs StaticPlaneBodyConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 );
  assert( 3 * m_idx_body + 2 < v.size() );

  const Vector2s r{ Eigen::Rotation2D<scalar>{ q( 3 * m_idx_body + 2 ) } * m_body_r };
  const Vector2s t{ -r.y(), r.x() };

  // v + omega x r - collision_point_vel
  return v.segment<2>( 3 * m_idx_body ) + v( 3 * m_idx_body + 2 ) * t - computePlaneCollisionPointVelocity( q );
}

void StaticPlaneBodyConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx_body = idx;
}

VectorXs StaticPlaneBodyConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  return computePlaneCollisionPointVelocity( q );
}

void StaticPlaneBodyConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<2>( 3 * m_idx_body ) + Eigen::Rotation2D<scalar>{ q( 3 * m_idx_body + 2 ) } * m_body_r;
}

void StaticPlaneBodyConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_plane.n();
}
