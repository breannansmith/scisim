#include "BallStaticPlaneConstraint.h"

#include "ball2d/StaticGeometry/StaticPlane.h"

bool StaticPlaneConstraint::isActive( const unsigned ball_idx, const VectorXs& q, const VectorXs& r, const Vector2s& x, const Vector2s& n )
{
  assert( q.size() % 2 == 0 ); assert( r.size() == q.size() / 2 ); assert( ball_idx < r.size() ); assert( fabs( n.norm() - 1.0 ) < 1.0e-6 );

  return n.dot( q.segment<2>( 2 * ball_idx ) - x ) <= r( ball_idx );
}

StaticPlaneConstraint::StaticPlaneConstraint( const unsigned ball_idx, const scalar& r, const StaticPlane& static_plane, const unsigned plane_idx )
: m_ball_idx( ball_idx )
, m_r( r )
, m_static_plane( static_plane )
, m_idx_plane( plane_idx )
{
  assert( fabs( m_static_plane.n().norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_static_plane.t().norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_static_plane.n().dot( m_static_plane.t() ) ) <= 1.0e-6 );
  assert( m_r >= 0.0 );
}

StaticPlaneConstraint::~StaticPlaneConstraint()
{}

scalar StaticPlaneConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 2 == 0 ); assert( q.size() == v.size() ); assert( 2 * m_ball_idx + 1 < v.size() );
  return m_static_plane.n().dot( computeRelativeVelocity( q, v ) );
}

//void StaticPlaneConstraint::resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const VectorXs& vin, VectorXs& vout ) const
//{
//  std::cerr << "StaticPlaneConstraint::resolveImpact temporarily disabled" << std::endl;
//  std::exit( EXIT_FAILURE );
////  assert( vin.size() == vout.size() ); assert( vin.size() % 2 == 0 ); assert( 2 * m_ball_idx + 1 < vin.size() );
////
////  // Apply impulse along constraint gradient
////  assert( evalNdotV( vin ) < 0.0 );
////  vout.segment<2>( 2 * m_ball_idx ) += -( 1. + CoR ) * evalNdotV( vin ) * m_n;
//}

//void StaticPlaneConstraint::exertImpulse( const VectorXs& q, const scalar& lambda, const SparseMatrixsc& Minv, VectorXs& vout ) const
//{
//  assert( vout.size() % 2 == 0 ); assert( 2 * m_ball_idx + 1 < vout.size() );
//  assert( Minv.rows() == Minv.cols() ); assert( Minv.nonZeros() == Minv.rows() ); assert( Minv.rows() == vout.size() );
//  assert( Minv.valuePtr()[ 2 * m_ball_idx ] == Minv.valuePtr()[ 2 * m_ball_idx + 1 ] );
//
//  vout.segment<2>( 2 * m_ball_idx ) += Minv.valuePtr()[ 2 * m_ball_idx ] * lambda * m_static_plane.n();
//}

scalar StaticPlaneConstraint::evaluateGapFunction( const VectorXs& q ) const
{
  assert( q.size() % 2 == 0 );
  assert( 2 * m_ball_idx + 1 < q.size() );
  return m_static_plane.n().dot( q.segment<2>( 2 * m_ball_idx ) - m_static_plane.x() ) - m_r;
}

void StaticPlaneConstraint::evalgradg( const VectorXs& /*q*/, const int col, SparseMatrixsc& G, const FlowableSystem& /*fsys*/ ) const
{
  assert( col >= 0 ); assert( col < G.cols() ); assert( fabs( m_static_plane.n().norm() - 1.0 ) <= 1.0e-6 );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  assert( 2 * m_ball_idx + 1 < unsigned( G.rows() ) );
  G.insert( 2 * m_ball_idx,     col ) = m_static_plane.n().x();
  G.insert( 2 * m_ball_idx + 1, col ) = m_static_plane.n().y();
}

void StaticPlaneConstraint::computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& /*v*/, const int start_column, const int /*num_samples*/, SparseMatrixsc& D, VectorXs& gdotD ) const
{
  assert( start_column >= 0 );
  assert( start_column < D.cols() );
  assert( D.cols() == gdotD.size() );

  // Effect on center of mass
  assert( 2 * m_ball_idx + 1 < unsigned( D.rows() ) );
  D.insert( 2 * m_ball_idx + 0, start_column ) = m_static_plane.t().x();
  D.insert( 2 * m_ball_idx + 1, start_column ) = m_static_plane.t().y();

  // Relative velocity contribution from kinematic scripting
  gdotD( start_column ) = -m_static_plane.t().dot( computePlaneCollisionPointVelocity( q ) );
}

void StaticPlaneConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& /*q*/, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( t.size() == 2 ); assert( column < unsigned( D.cols() ) );
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( m_static_plane.n().dot( t ) ) <= 1.0e-6 );

  // Effect on center of mass
  D.insert( 2 * m_ball_idx + 0, column ) = t.x();
  D.insert( 2 * m_ball_idx + 1, column ) = t.y();
}

int StaticPlaneConstraint::impactStencilSize() const
{
  return 2;
}

int StaticPlaneConstraint::frictionStencilSize() const
{
  return 2;
}

void StaticPlaneConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_ball_idx;
  bodies.second = -1;
}

void StaticPlaneConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  this->getSimulatedBodyIndices( bodies );
}

void StaticPlaneConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 ); assert( strt_idx < gdotN.size() );
  gdotN( strt_idx ) = - m_static_plane.n().dot( computePlaneCollisionPointVelocity( q ) );
}

void StaticPlaneConstraint::evalH( const VectorXs& /*q*/, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& /*H1*/ ) const
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

Vector2s StaticPlaneConstraint::computePlaneCollisionPointVelocity( const VectorXs& /*q*/ ) const
{
  // No rotational component allowed in 2D static planes, yet
  return m_static_plane.v();
}

bool StaticPlaneConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool StaticPlaneConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool StaticPlaneConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string StaticPlaneConstraint::name() const
{
  return "static_plane_constraint";
}

void StaticPlaneConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<2>( 2 * m_ball_idx ) - m_r * m_static_plane.n();
}

void StaticPlaneConstraint::getWorldSpaceContactNormal( const VectorXs& /*q*/, VectorXs& contact_normal ) const
{
  contact_normal = m_static_plane.n();
}

unsigned StaticPlaneConstraint::getStaticObjectIndex() const
{
  return m_idx_plane;
}

unsigned StaticPlaneConstraint::planeIdx() const
{
  return m_idx_plane;
}

unsigned StaticPlaneConstraint::ballIdx() const
{
  return m_ball_idx;
}

void StaticPlaneConstraint::computeContactBasis( const VectorXs& /*q*/, const VectorXs& /*v*/, MatrixXXsc& basis ) const
{
  const Vector2s n{ m_static_plane.n() };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  const Vector2s t{ m_static_plane.t() };
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( n.dot( t ) ) <= 1.0e-6 );

  basis.resize( 2, 2 );
  basis.col( 0 ) = n;
  basis.col( 1 ) = t;
}

VectorXs StaticPlaneConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 2 == 0 ); assert( q.size() == v.size() ); assert( 2 * m_ball_idx + 1 < v.size() );
  // v_point - v_plane_collision_point
  return v.segment<2>( 2 * m_ball_idx ) - computePlaneCollisionPointVelocity( q );
}

void StaticPlaneConstraint::setBodyIndex0( const unsigned idx )
{
  m_ball_idx = idx;
}

scalar StaticPlaneConstraint::computePenetrationDepth( const VectorXs& q ) const
{
  return std::min( 0.0, m_static_plane.n().dot( q.segment<2>( 2 * m_ball_idx ) - m_static_plane.x() ) - m_r );
}

VectorXs StaticPlaneConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& /*v*/ ) const
{
  return computePlaneCollisionPointVelocity( q );
}
