// StaticPlaneCircleConstraint.cpp
//
// Breannan Smith
// Last updated: 09/30/2015

#include "StaticPlaneCircleConstraint.h"

#include "scisim/Math/MathUtilities.h"

#include "RigidBody2DStaticPlane.h"

bool StaticPlaneCircleConstraint::isActive( const Vector2s& x_circle, const scalar& r_circle, const RigidBody2DStaticPlane& plane )
{
  assert( fabs( plane.n().norm() - 1.0 ) <= 1.0e-6 );
  return plane.n().dot( x_circle - plane.x() ) <= r_circle;
}

StaticPlaneCircleConstraint::StaticPlaneCircleConstraint( const unsigned body_idx, const unsigned plane_idx, const scalar& r, const RigidBody2DStaticPlane& plane )
: m_circle_idx( body_idx )
, m_r( r )
, m_plane( plane )
, m_plane_idx( plane_idx )
{
  assert( m_r >= 0.0 );
  assert( fabs( m_plane.n().norm() - 1.0 ) <= 1.0e-6 );
}

scalar StaticPlaneCircleConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 ); assert( q.size() == v.size() ); assert( 3 * m_circle_idx + 1 < v.size() );
  return m_plane.n().dot( v.segment<2>( 3 * m_circle_idx ) - computePlaneCollisionPointVelocity( q ) );
}

void StaticPlaneCircleConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 ); assert( col < G.cols() );
  assert( 3 * m_circle_idx + 1 < unsigned( G.rows() ) );

  assert( fabs( m_plane.n().norm() - 1.0 ) <= 1.0e-6 );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  G.insert( 3 * m_circle_idx + 0, col ) = m_plane.n().x();
  G.insert( 3 * m_circle_idx + 1, col ) = m_plane.n().y();
}

int StaticPlaneCircleConstraint::impactStencilSize() const
{
  return 2;
}

void StaticPlaneCircleConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_circle_idx;
  bodies.second = -1;
}

void StaticPlaneCircleConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_circle_idx;
  bodies.second = -1;
}

void StaticPlaneCircleConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 ); assert( strt_idx < gdotN.size() );
  gdotN( strt_idx ) = - m_plane.n().dot( computePlaneCollisionPointVelocity( q ) );
}

void StaticPlaneCircleConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
{
  assert( H0.rows() == 2 ); assert( H0.cols() == 3 );
  assert( H1.rows() == 2 ); assert( H1.cols() == 3 );
  assert( ( basis * basis.transpose() - MatrixXXsc::Identity( 2, 2 ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-6 );

  // Grab the contact normal
  const Vector2s n{ basis.col( 0 ) };
  // Grab the tangent basis
  const Vector2s t{ basis.col( 1 ) };

  // Compute the displacement from the center of mass to the point of contact
  assert( m_r >= 0.0 );
  const Vector2s r_world{ - m_r * n };

  H0.block<1,2>(0,0) = n;
  H0( 0, 2 ) = 0.0;

  H0.block<1,2>(1,0) = t;
  H0( 1, 2 ) = MathUtilities::cross( r_world, t );
}

bool StaticPlaneCircleConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool StaticPlaneCircleConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool StaticPlaneCircleConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string StaticPlaneCircleConstraint::name() const
{
  return "static_plane_circle";
}

Vector2s StaticPlaneCircleConstraint::computePlaneCollisionPointVelocity( const VectorXs& q ) const
{
  return Vector2s::Zero();
}

void StaticPlaneCircleConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  const Vector2s n{ m_plane.n() };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  const Vector2s t{ -n.y(), n.x() };
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( n.dot( t ) ) <= 1.0e-6 );

  basis.resize( 2, 2 );
  basis.col( 0 ) = n;
  basis.col( 1 ) = t;
}

VectorXs StaticPlaneCircleConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 );
  assert( 3 * m_circle_idx + 2 < v.size() );

  // Point of contact relative to first body's center of mass
  const Vector2s r0{ - m_r * m_plane.n() };
  assert( fabs( MathUtilities::cross( m_plane.n(), r0 ) ) <= 1.0e-6 );
  // Rotate 90 degrees counter clockwise for computing the torque
  const Vector2s t0{ -r0.y(), r0.x() };

  // v_point + omega_point x r_point - v_plane_collision_point
  return v.segment<2>( 3 * m_circle_idx ) + v( 3 * m_circle_idx + 2 ) * t0 - computePlaneCollisionPointVelocity( q );
}

void StaticPlaneCircleConstraint::setBodyIndex0( const unsigned idx )
{
  m_circle_idx = idx;
}

scalar StaticPlaneCircleConstraint::computePenetrationDepth( const VectorXs& q ) const
{
  assert( 3 * m_circle_idx + 2 < q.size() );
  return std::min( 0.0, m_plane.n().dot( q.segment<2>( 3 * m_circle_idx ) - m_plane.x() ) - m_r );
}

VectorXs StaticPlaneCircleConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  return computePlaneCollisionPointVelocity( q );
}

void StaticPlaneCircleConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<2>( 3 * m_circle_idx ) - m_r * m_plane.n();
}

void StaticPlaneCircleConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_plane.n();
}

unsigned StaticPlaneCircleConstraint::getStaticObjectIndex() const
{
  return m_plane_idx;
}

bool StaticPlaneCircleConstraint::operator==( const StaticPlaneCircleConstraint& other ) const
{
  return std::tie( other.m_circle_idx, other.m_plane_idx ) == std::tie( m_circle_idx, m_plane_idx );
}
