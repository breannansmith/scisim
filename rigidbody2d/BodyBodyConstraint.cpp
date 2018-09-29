#include "BodyBodyConstraint.h"

#include "scisim/Math/MathUtilities.h"

BodyBodyConstraint::BodyBodyConstraint( const unsigned idx0, const unsigned idx1, const Vector2s& p, const Vector2s& n, const VectorXs& q )
: m_idx0( idx0 )
, m_idx1( idx1 )
, m_n( n )
, m_r0( p - q.segment<2>( 3 * m_idx0 ) )
, m_r1( p - q.segment<2>( 3 * m_idx1 ) )
{
  assert( m_idx0 != m_idx1 );
  assert( m_idx0 < m_idx1 );
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
}

scalar BodyBodyConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  return m_n.dot( computeRelativeVelocity( q, v ) );
}

int BodyBodyConstraint::impactStencilSize() const
{
  return 6;
}

void BodyBodyConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx0;
  bodies.second = m_idx1;
}

void BodyBodyConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx0;
  bodies.second = m_idx1;
}

void BodyBodyConstraint::evalH( const VectorXs& /*q*/, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
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
  H0(0,2) = MathUtilities::cross( m_r0, n );

  H0.block<1,2>(1,0) = t;
  H0(1,2) = MathUtilities::cross( m_r0, t );

  H1.block<1,2>(0,0) = n;
  H1(0,2) = MathUtilities::cross( m_r1, n );

  H1.block<1,2>(1,0) = t;
  H1(1,2) = MathUtilities::cross( m_r1, t );
}

void BodyBodyConstraint::computeContactBasis( const VectorXs& /*q*/, const VectorXs& /*v*/, MatrixXXsc& basis ) const
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  const Vector2s t{ -m_n.y(), m_n.x() };
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );
  basis.resize( 2, 2 );
  basis.col( 0 ) = m_n;
  basis.col( 1 ) = t;
}

bool BodyBodyConstraint::conservesTranslationalMomentum() const
{
  return true;
}

bool BodyBodyConstraint::conservesAngularMomentumUnderImpact() const
{
  return true;
}

bool BodyBodyConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return true;
}

std::string BodyBodyConstraint::name() const
{
  return "body_body";
}

VectorXs BodyBodyConstraint::computeRelativeVelocity( const VectorXs& /*q*/, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 );
  assert( 3 * m_idx0 + 2 < v.size() );
  assert( 3 * m_idx1 + 2 < v.size() );

  const Vector2s t0{ -m_r0.y(), m_r0.x() };
  const Vector2s t1{ -m_r1.y(), m_r1.x() };

  // v_0 + omega_0 x r_0 - ( v_1 + omega_1 x r_1 )
  return v.segment<2>( 3 * m_idx0 ) + v( 3 * m_idx0 + 2 ) * t0 - v.segment<2>( 3 * m_idx1 ) - v( 3 * m_idx1 + 2 ) * t1;
}

void BodyBodyConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx0 = idx;
}

void BodyBodyConstraint::setBodyIndex1( const unsigned idx )
{
  m_idx1 = idx;
}

VectorXs BodyBodyConstraint::computeKinematicRelativeVelocity( const VectorXs& /*q*/, const VectorXs& /*v*/ ) const
{
  // No kinematic contribution
  return VectorXs::Zero( 2 );
}

void BodyBodyConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = m_r0 + q.segment<2>( 3 * m_idx0 );
  #ifndef NDEBUG
  const VectorXs p{ m_r1 + q.segment<2>( 3 * m_idx1 ) };
  assert( ( p - contact_point ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  #endif
}

void BodyBodyConstraint::getWorldSpaceContactNormal( const VectorXs& /*q*/, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}
