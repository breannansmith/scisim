#include "BallStaticDrumConstraint.h"

bool StaticDrumConstraint::isActive( const unsigned ball_idx, const VectorXs& q, const VectorXs& r, const Vector2s& X, const scalar& R )
{
  assert( 2 * r.size() == q.size() );
  assert( ball_idx < r.size() );
  assert( R >= 0.0 ); assert( r( ball_idx ) >= 0.0 );
  assert( R >= r( ball_idx ) );

  return ( X - q.segment<2>( 2 * ball_idx ) ).squaredNorm() >= ( R - r( ball_idx ) ) * ( R - r( ball_idx ) );
}

StaticDrumConstraint::StaticDrumConstraint( const unsigned ball_idx, const VectorXs& q, const scalar& r, const Vector2s& X, const unsigned drum_idx )
: m_idx_ball( ball_idx )
, m_n( ( X - q.segment<2>( 2 * ball_idx ) ).normalized() )
, m_idx_drum( drum_idx )
, m_r_ball( r )
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  assert( m_r_ball > 0.0 );
}

StaticDrumConstraint::~StaticDrumConstraint()
{}

scalar StaticDrumConstraint::evalNdotV( const VectorXs& /*q*/, const VectorXs& v ) const
{
  assert( v.size() % 2 == 0 ); assert( 2 * m_idx_ball + 1 < v.size() );

  return m_n.dot( v.segment<2>( 2 * m_idx_ball ) );
}

//void StaticDrumConstraint::resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const VectorXs& vin, VectorXs& vout ) const
//{
//  // TODO
//  std::cerr << "StaticDrumConstraint::resolveImpact temporarily disabled" << std::endl;
//  std::exit( EXIT_FAILURE );
////  assert( vin.size() == vout.size() ); assert( vin.size() % 2 == 0 ); assert( 2*m_idx_ball+1 < vin.size() );
////
////  // Apply impulse along constraint gradient
////  assert( evalNdotV( vin ) < 0.0 );
////  vout.segment<2>( 2 * m_idx_ball ) += -( 1. + CoR ) * evalNdotV( vin ) * m_n;
//}

//void StaticDrumConstraint::exertImpulse( const VectorXs& q, const scalar& lambda, const SparseMatrixsc& Minv, VectorXs& vout ) const
//{
//  assert( vout.size() % 2 == 0 ); assert( 2 * m_idx_ball + 1 < vout.size() );
//  assert( Minv.rows() == Minv.cols() ); assert( Minv.nonZeros() == Minv.rows() ); assert( Minv.rows() == vout.size() );
//  assert( Minv.valuePtr()[ 2 * m_idx_ball ] == Minv.valuePtr()[ 2 * m_idx_ball + 1 ] );
//
//  vout.segment<2>( 2 * m_idx_ball ) += Minv.valuePtr()[ 2 * m_idx_ball ] * lambda * m_n;
//}

void StaticDrumConstraint::evalgradg( const VectorXs& /*q*/, const int col, SparseMatrixsc& G, const FlowableSystem& /*fsys*/ ) const
{
  assert( col >= 0 ); assert( col < G.cols() ); assert( 2 * m_idx_ball + 1 < unsigned( G.rows() ) );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  G.insert( 2 * m_idx_ball,     col ) = m_n.x();
  G.insert( 2 * m_idx_ball + 1, col ) = m_n.y();
}

void StaticDrumConstraint::computeGeneralizedFrictionDisk( const VectorXs& /*q*/, const VectorXs& /*v*/, const int start_column, const int /*num_samples*/, SparseMatrixsc& D, VectorXs& gdotD ) const
{
  assert( start_column >= 0 );
  assert( start_column < D.cols() );
  assert( D.cols() == gdotD.size() );

  // Rotate the normal by 90 degrees
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  const Vector2s t{ -m_n.y(), m_n.x() };
  assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );

  // Effect on center of mass
  assert( 2 * m_idx_ball + 1 < unsigned( D.rows() ) );
  D.insert( 2 * m_idx_ball + 0, start_column ) = t.x();
  D.insert( 2 * m_idx_ball + 1, start_column ) = t.y();

  // Relative velocity contribution from kinematic scripting, zero for now
  gdotD( start_column ) = 0.0;
}

void StaticDrumConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& /*q*/, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( t.size() == 2 ); assert( column < unsigned( D.cols() ) );
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );

  // Effect on center of mass
  assert( 2 * m_idx_ball + 1 < unsigned( D.rows() ) );
  D.insert( 2 * m_idx_ball + 0, column ) = t.x();
  D.insert( 2 * m_idx_ball + 1, column ) = t.y();
}

int StaticDrumConstraint::impactStencilSize() const
{
  return 2;
}

int StaticDrumConstraint::frictionStencilSize() const
{
  return 2;
}

void StaticDrumConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx_ball;
  bodies.second = -1;
}

void StaticDrumConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  this->getSimulatedBodyIndices( bodies );
}

void StaticDrumConstraint::evalKinematicNormalRelVel( const VectorXs& /*q*/, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 ); assert( strt_idx < gdotN.size() );

  // No kinematic scripting here, yet
  gdotN( strt_idx ) = 0.0;
}

void StaticDrumConstraint::evalH( const VectorXs& /*q*/, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& /*H1*/ ) const
{
  assert( H0.rows() == 2 ); assert( H0.cols() == 2 );
  assert( ( basis * basis.transpose() - MatrixXXsc::Identity( 2, 2 ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-6 );

  // Grab the contact normal
  const Vector2s n{ basis.col( 0 ) };
  // Grab the tangent basis
  const Vector2s t{ basis.col( 1 ) };

  // Format for H:
  //   n^T
  //   t^T
  H0.block<1,2>(0,0) = n;
  H0.block<1,2>(1,0) = t;
}

bool StaticDrumConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool StaticDrumConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool StaticDrumConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string StaticDrumConstraint::name() const
{
  return "static_drum_constraint";
}

void StaticDrumConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<2>( 2 * m_idx_ball ) - m_r_ball * m_n;
}

void StaticDrumConstraint::getWorldSpaceContactNormal( const VectorXs& /*q*/, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}

unsigned StaticDrumConstraint::getStaticObjectIndex() const
{
  return m_idx_drum;
}

unsigned StaticDrumConstraint::drumIdx() const
{
  return m_idx_drum;
}

unsigned StaticDrumConstraint::ballIdx() const
{
  return m_idx_ball;
}

void StaticDrumConstraint::computeContactBasis( const VectorXs& /*q*/, const VectorXs& /*v*/, MatrixXXsc& basis ) const
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  const Vector2s t{ -m_n.y(), m_n.x() };
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );

  basis.resize( 2, 2 );
  basis.col( 0 ) = m_n;
  basis.col( 1 ) = t;
}

VectorXs StaticDrumConstraint::computeRelativeVelocity( const VectorXs& /*q*/, const VectorXs& v ) const
{
  assert( v.size() % 2 == 0 );
  assert( 2 * m_idx_ball + 1 < v.size() );
  return v.segment<2>( 2 * m_idx_ball );
}

void StaticDrumConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx_ball = idx;
}

VectorXs StaticDrumConstraint::computeKinematicRelativeVelocity( const VectorXs& /*q*/, const VectorXs& /*v*/ ) const
{
  // No kinematic contribution, for now
  return VectorXs::Zero( 2 );
}
