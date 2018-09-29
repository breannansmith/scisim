#include "CircleCircleConstraint.h"

#include "scisim/Math/MathUtilities.h"

bool CircleCircleConstraint::isActive( const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1 )
{
  return ( x0 - x1 ).squaredNorm() <= ( r0 + r1 ) * ( r0 + r1 );
}

CircleCircleConstraint::CircleCircleConstraint( const unsigned sphere_idx0, const unsigned sphere_idx1, const Vector2s& n, const Vector2s& p, const scalar& r0, const scalar& r1 )
: m_idx0( sphere_idx0 )
, m_idx1( sphere_idx1 )
, m_n( n )
, m_p( p )
, m_r0( r0 )
, m_r1( r1 )
{
  assert( m_idx0 != m_idx1 );
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  assert( m_r0 >= 0.0 );
  assert( m_r1 >= 0.0 );
}

scalar CircleCircleConstraint::evalNdotV( const VectorXs& /*q*/, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 ); assert( 3 * m_idx0 + 1 < v.size() ); assert( 3 * m_idx1 + 1 < v.size() );
  // n || r => n dot ( omega cross r ) == 0, so computeRelativeVelocity not used
  return m_n.dot( v.segment<2>( 3 * m_idx0 ) - v.segment<2>( 3 * m_idx1 ) );
}

void CircleCircleConstraint::evalgradg( const VectorXs& /*q*/, const int col, SparseMatrixsc& G, const FlowableSystem& /*fsys*/ ) const
{
  assert( col >= 0 ); assert( col < G.cols() );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  assert( m_idx0 < m_idx1 );
  assert( 3 * m_idx0 + 1 < unsigned( G.rows() ) );
  G.insert( 3 * m_idx0 + 0, col ) =  m_n.x();
  G.insert( 3 * m_idx0 + 1, col ) =  m_n.y();
  assert( 3 * m_idx1 + 1 < unsigned( G.rows() ) );
  G.insert( 3 * m_idx1 + 0, col ) = -m_n.x();
  G.insert( 3 * m_idx1 + 1, col ) = -m_n.y();
}

void CircleCircleConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( column < unsigned( D.cols() ) ); assert( q.size() % 3 == 0 );
  assert( t.size() == 2 ); assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );
  assert( m_idx0 < m_idx1 );

  // Effect on center of mass of body i
  D.insert( 3 * m_idx0 + 0, column ) = t.x();
  D.insert( 3 * m_idx0 + 1, column ) = t.y();
  // Effect on orientation of body i
  {
    const Vector2s ri{ m_p - q.segment<2>( 3 * m_idx0 ) };
    assert( fabs( MathUtilities::cross( m_n, ri ) ) <= 1.0e-6 );
    const scalar ntilde{ MathUtilities::cross( ri, t ) };
    D.insert( 3 * m_idx0 + 2, column ) = ntilde;
  }

  // Effect on center of mass of body j
  D.insert( 3 * m_idx1 + 0, column ) = -t.x();
  D.insert( 3 * m_idx1 + 1, column ) = -t.y();
  // Effect on orientation of body j
  {
    const Vector2s rj{ m_p - q.segment<2>( 3 * m_idx1 ) };
    assert( fabs( MathUtilities::cross( m_n, rj ) ) <= 1.0e-6 );
    const scalar ntilde{ MathUtilities::cross( rj, t ) };
    D.insert( 3 * m_idx1 + 2, column ) = -ntilde;
  }
}

int CircleCircleConstraint::impactStencilSize() const
{
  return 4;
}

int CircleCircleConstraint::frictionStencilSize() const
{
  return 6;
}

void CircleCircleConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx0;
  bodies.second = m_idx1;
}

void CircleCircleConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx0;
  bodies.second = m_idx1;
}

void CircleCircleConstraint::evalKinematicNormalRelVel( const VectorXs& /*q*/, const int strt_idx, VectorXs& gdotN ) const
{
  // No relative velocity contribution from kinematic scripting
  gdotN( strt_idx ) = 0.0;
}

void CircleCircleConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
{
  assert( H0.rows() == 2 ); assert( H0.cols() == 3 );
  assert( H1.rows() == 2 ); assert( H1.cols() == 3 );
  assert( ( basis * basis.transpose() - MatrixXXsc::Identity( 2, 2 ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-6 );

  // Grab the contact normal
  const Vector2s n{ basis.col( 0 ) };
  // Grab the tangent basis
  const Vector2s t{ basis.col( 1 ) };

  // Generate arms
  const Vector2s ri{ m_p - q.segment<2>( 3 * m_idx0 ) };
  const Vector2s rj{ m_p - q.segment<2>( 3 * m_idx1 ) };

  // Format for H:
  //   n^T  r x n
  //   t^T  r x t

  H0.block<1,2>(0,0) = n;
  assert( fabs( MathUtilities::cross( ri, n ) ) <= 1.0e-6 );
  H0(0,2) = 0.0;

  H0.block<1,2>(1,0) = t;
  H0(1,2) = MathUtilities::cross( ri, t );

  H1.block<1,2>(0,0) = n;
  assert( fabs( MathUtilities::cross( rj, n ) ) <= 1.0e-6 );
  H1(0,2) = 0.0;

  H1.block<1,2>(1,0) = t;
  H1(1,2) = MathUtilities::cross( rj, t );
}

void CircleCircleConstraint::computeContactBasis( const VectorXs& /*q*/, const VectorXs& /*v*/, MatrixXXsc& basis ) const
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  const Vector2s t{ -m_n.y(), m_n.x() };
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );
  basis.resize( 2, 2 );
  basis.col( 0 ) = m_n;
  basis.col( 1 ) = t;
}

bool CircleCircleConstraint::conservesTranslationalMomentum() const
{
  return true;
}

bool CircleCircleConstraint::conservesAngularMomentumUnderImpact() const
{
  return true;
}

bool CircleCircleConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return true;
}

std::string CircleCircleConstraint::name() const
{
  return "circle_circle";
}

VectorXs CircleCircleConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 );
  assert( 3 * m_idx0 + 2 < v.size() );
  assert( 3 * m_idx1 + 2 < v.size() );

  // Point of contact relative to first body's center of mass
  const Vector2s r0{ m_p - q.segment<2>( 3 * m_idx0 ) };
  assert( fabs( MathUtilities::cross( m_n, r0 ) ) <= 1.0e-6 );
  // Rotate 90 degrees counter clockwise for computing the torque
  const Vector2s t0{ -r0.y(), r0.x() };

  // Point of contact relative to second body's center of mass
  const Vector2s r1{ m_p - q.segment<2>( 3 * m_idx1 ) };
  assert( fabs( MathUtilities::cross( m_n, r1 ) ) <= 1.0e-6 );
  // Rotate 90 degrees counter clockwise for computing the torque
  const Vector2s t1{ -r1.y(), r1.x() };

  // v_0 + omega_0 x r_0 - ( v_1 + omega_1 x r_1 )
  return v.segment<2>( 3 * m_idx0 ) + v( 3 * m_idx0 + 2 ) * t0 - v.segment<2>( 3 * m_idx1 ) - v( 3 * m_idx1 + 2 ) * t1;
}

void CircleCircleConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx0 = idx;
}

void CircleCircleConstraint::setBodyIndex1( const unsigned idx )
{
  m_idx1 = idx;
}

scalar CircleCircleConstraint::computePenetrationDepth( const VectorXs& q ) const
{
  return std::min( 0.0, ( q.segment<2>( 3 * m_idx0 ) - q.segment<2>( 3 * m_idx1 ) ).norm() - m_r0 - m_r1 );
}

VectorXs CircleCircleConstraint::computeKinematicRelativeVelocity( const VectorXs& /*q*/, const VectorXs& /*v*/ ) const
{
  // No kinematic contribution
  return VectorXs::Zero( 2 );
}

void CircleCircleConstraint::getWorldSpaceContactPoint( const VectorXs& /*q*/, VectorXs& contact_point ) const
{
  contact_point = m_p;
}

void CircleCircleConstraint::getWorldSpaceContactNormal( const VectorXs& /*q*/, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}

bool CircleCircleConstraint::operator==( const CircleCircleConstraint& other ) const
{
  assert( other.m_idx0 < other.m_idx1 );
  assert( m_idx0 < m_idx1 );
  return std::tie( other.m_idx0, other.m_idx1 ) == std::tie( m_idx0, m_idx1 );
}
