// TeleportedCircleCircleConstraint.cpp
//
// Breannan Smith
// Last updated: 09/10/2015

#include "TeleportedCircleCircleConstraint.h"

#include "scisim/Math/MathUtilities.h"

TeleportedCircleCircleConstraint::TeleportedCircleCircleConstraint( const unsigned idx0, const unsigned idx1, const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1, const Vector2s& delta0, const Vector2s& delta1, const scalar& radius0, const scalar& radius1 )
: m_idx0( idx0 )
, m_idx1( idx1 )
, m_n( ( x0 - x1 ).normalized() )
// Don't store the absolute collision point, as one body is effectively in two locations. Instead, store a relative displacement to the collision point.
// p = x0 + ( r0 / ( r0 + r1 ) ) * ( x1 - x0 )
// p - x0
, m_r0( ( r0 / ( r0 + r1 ) ) * ( x1 - x0 ) )
// p - x1
, m_r1( ( r0 / ( r0 + r1 ) - 1.0 ) * ( x1 - x0 ) )
, m_delta0( delta0 )
, m_delta1( delta1 )
, m_radius0( radius0 )
, m_radius1( radius1 )
{
  assert( idx0 != idx1 );
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
}

TeleportedCircleCircleConstraint::~TeleportedCircleCircleConstraint()
{}

scalar TeleportedCircleCircleConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 ); assert( 3 * m_idx0 + 1 < v.size() ); assert( 3 * m_idx1 + 1 < v.size() );
  // n || r => n dot ( omega cross r ) == 0
  return m_n.dot( v.segment<2>( 3 * m_idx0 ) - v.segment<2>( 3 * m_idx1 ) );
}

void TeleportedCircleCircleConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 ); assert( col < G.cols() );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  assert( m_idx0 < m_idx1 );
  assert( 3 * m_idx0 + 1 < unsigned( G.rows() ) );
  G.insert( 3 * m_idx0 + 0, col ) = m_n.x();
  G.insert( 3 * m_idx0 + 1, col ) = m_n.y();
  assert( 3 * m_idx1 + 1 < unsigned( G.rows() ) );
  G.insert( 3 * m_idx1 + 0, col ) = - m_n.x();
  G.insert( 3 * m_idx1 + 1, col ) = - m_n.y();
}

int TeleportedCircleCircleConstraint::impactStencilSize() const
{
  return 4;
}

void TeleportedCircleCircleConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx0;
  bodies.second = m_idx1;
}

void TeleportedCircleCircleConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx0;
  bodies.second = m_idx1;
}

void TeleportedCircleCircleConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  // No relative velocity contribution from kinematic scripting
  gdotN( strt_idx ) = 0.0;
}

bool TeleportedCircleCircleConstraint::conservesTranslationalMomentum() const
{
  return true;
}

bool TeleportedCircleCircleConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool TeleportedCircleCircleConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string TeleportedCircleCircleConstraint::name() const
{
  return "teleported_circle_circle";
}

void TeleportedCircleCircleConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
{
  assert( H0.rows() == 2 ); assert( H0.cols() == 3 );
  assert( H1.rows() == 2 ); assert( H1.cols() == 3 );
  assert( ( basis * basis.transpose() - MatrixXXsc::Identity( 2, 2 ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-6 );

  // Grab the contact normal
  const Vector2s n{ basis.col( 0 ) };
  // Grab the tangent basis
  const Vector2s t{ basis.col( 1 ) };

  // Format for H:
  //   n^T  r x n
  //   t^T  r x t

  H0.block<1,2>(0,0) = n;
  assert( fabs( MathUtilities::cross( m_r0, n ) ) <= 1.0e-6 );
  H0(0,2) = 0.0;

  H0.block<1,2>(1,0) = t;
  H0(1,2) = MathUtilities::cross( m_r0, t );

  H1.block<1,2>(0,0) = n;
  assert( fabs( MathUtilities::cross( m_r1, n ) ) <= 1.0e-6 );
  H1(0,2) = 0.0;

  H1.block<1,2>(1,0) = t;
  H1(1,2) = MathUtilities::cross( m_r1, t );
}

void TeleportedCircleCircleConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  const Vector2s t{ -m_n.y(), m_n.x() };
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );
  basis.resize( 2, 2 );
  basis.col( 0 ) = m_n;
  basis.col( 1 ) = t;
}

VectorXs TeleportedCircleCircleConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 );
  assert( 3 * m_idx0 + 2 < v.size() );
  assert( 3 * m_idx1 + 2 < v.size() );

  // Rotate 90 degrees counter clockwise for computing the torque
  assert( fabs( MathUtilities::cross( m_n, m_r0 ) ) <= 1.0e-6 );
  const Vector2s t0{ -m_r0.y(), m_r0.x() };

  // Rotate 90 degrees counter clockwise for computing the torque
  assert( fabs( MathUtilities::cross( m_n, m_r1 ) ) <= 1.0e-6 );
  const Vector2s t1{ -m_r1.y(), m_r1.x() };

  // v_0 + omega_0 x r_0 - ( v_1 + omega_1 x r_1 )
  return v.segment<2>( 3 * m_idx0 ) + v( 3 * m_idx0 + 2 ) * t0 - v.segment<2>( 3 * m_idx1 ) - v( 3 * m_idx1 + 2 ) * t1;
}

void TeleportedCircleCircleConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<2>( 3 * m_idx0 ) + m_r0;
}

bool TeleportedCircleCircleConstraint::operator==( const TeleportedCircleCircleConstraint& other ) const
{
  assert( other.m_idx0 < other.m_idx1 );
  assert( m_idx0 < m_idx1 );
  return std::tie( other.m_idx0, other.m_idx1 ) == std::tie( m_idx0, m_idx1 );
}

void TeleportedCircleCircleConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx0 = idx;
}

void TeleportedCircleCircleConstraint::setBodyIndex1( const unsigned idx )
{
  m_idx1 = idx;
}

scalar TeleportedCircleCircleConstraint::computePenetrationDepth( const VectorXs& q ) const
{
  const Vector2s q0{ q.segment<2>( 3 * m_idx0 ) + m_delta0 };
  const Vector2s q1{ q.segment<2>( 3 * m_idx1 ) + m_delta1 };
  return std::min( 0.0, ( q0 - q1 ).norm() - m_radius0 - m_radius1 );
}

VectorXs TeleportedCircleCircleConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  // No kinematic contribution
  return VectorXs::Zero( 2 );
}
