// BallBallConstraint.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "BallBallConstraint.h"

bool BallBallConstraint::isActive( const unsigned idx0, const unsigned idx1, const VectorXs& q, const VectorXs& r )
{
  assert( q.size() % 2 == 0 ); assert( r.size() == q.size() / 2 );
  assert( idx0 < r.size() ); assert( idx1 < r.size() );

  return isActive( q.segment<2>( 2 * idx0 ), q.segment<2>( 2 * idx1 ), r( idx0 ), r( idx1 ) );
}

bool BallBallConstraint::isActive( const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1 )
{
  assert( r0 > 0.0 ); assert( r1 > 0.0 );
  return ( x0 - x1 ).squaredNorm() <= ( r0 + r1 ) * ( r0 + r1 );
}

BallBallConstraint::BallBallConstraint( const unsigned idx0, const unsigned idx1, const VectorXs& q, const scalar& r0, const scalar& r1, const bool teleported )
: BallBallConstraint( idx0, idx1, q.segment<2>( 2 * idx0 ), q.segment<2>( 2 * idx1 ), r0, r1, teleported )
{}

BallBallConstraint::BallBallConstraint( const unsigned idx0, const unsigned idx1, const Vector2s& x0, const Vector2s& x1, const scalar& r0, const scalar& r1, const bool teleported )
: m_sphere_idx0( idx0 )
, m_sphere_idx1( idx1 )
, m_n( ( x0 - x1 ).normalized() )
, m_r0( r0 )
, m_r1( r1 )
, m_teleported( teleported )
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  assert( m_r0 >= 0.0 );
  assert( m_r1 >= 0.0 );
}

scalar BallBallConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 2 == 0 ); assert( 2 * m_sphere_idx0 + 1 < v.size() ); assert( 2 * m_sphere_idx1 + 1 < v.size() );
  return m_n.dot( v.segment<2>( 2 * m_sphere_idx0 ) - v.segment<2>( 2 * m_sphere_idx1 ) );
}

void BallBallConstraint::resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const scalar& ndotv, VectorXs& vout, scalar& alpha ) const
{
  assert( vout.size() % 2 == 0 );
  assert( 2 * m_sphere_idx0 + 1 < vout.size() );
  assert( 2 * m_sphere_idx1 + 1 < vout.size() );
  assert( M.rows() == M.cols() );
  assert( M.nonZeros() == M.rows() );
  assert( M.rows() == vout.size() );
  assert( M.valuePtr()[ 2 * m_sphere_idx0 ] == M.valuePtr()[ 2 * m_sphere_idx0 + 1 ] );
  assert( M.valuePtr()[ 2 * m_sphere_idx1 ] == M.valuePtr()[ 2 * m_sphere_idx1 + 1 ] );
  assert( ndotv < 0.0 );

  const scalar& m0{ M.valuePtr()[ 2 * m_sphere_idx0 ] };
  const scalar& m1{ M.valuePtr()[ 2 * m_sphere_idx1 ] };

  alpha = - ( 1.0 + CoR ) * ndotv / ( ( 1.0 / m0 ) + ( 1.0 / m1 ) );
  vout.segment<2>( 2 * m_sphere_idx0 ) += m_n * ( alpha / m0 );
  vout.segment<2>( 2 * m_sphere_idx1 ) -= m_n * ( alpha / m1 );
}

//void BallBallConstraint::exertImpulse( const VectorXs& q, const scalar& lambda, const SparseMatrixsc& Minv, VectorXs& vout ) const
//{
//  assert( vout.size() % 2 == 0 ); assert( 2 * m_sphere_idx0 + 1 < vout.size() ); assert( 2 * m_sphere_idx1 + 1 < vout.size() );
//  assert( Minv.rows() == Minv.cols() ); assert( Minv.nonZeros() == Minv.rows() ); assert( Minv.rows() == vout.size() );
//  assert( Minv.valuePtr()[ 2 * m_sphere_idx1 ] == Minv.valuePtr()[ 2 * m_sphere_idx1 + 1 ] );
//  assert( Minv.valuePtr()[ 2 * m_sphere_idx0 ] == Minv.valuePtr()[ 2 * m_sphere_idx0 + 1 ] );
//
//  vout.segment<2>( 2 * m_sphere_idx1 ) += Minv.valuePtr()[ 2 * m_sphere_idx1 ] * lambda * m_n;
//  vout.segment<2>( 2 * m_sphere_idx0 ) -= Minv.valuePtr()[ 2 * m_sphere_idx0 ] * lambda * m_n;
//}

scalar BallBallConstraint::evaluateGapFunction( const VectorXs& q ) const
{
  if( m_teleported )
  {
    return SCALAR_NAN;
  }
  else
  {
    return ( q.segment<2>( 2 * m_sphere_idx0 ) - q.segment<2>( 2 * m_sphere_idx1 ) ).norm() - ( m_r0 + m_r1 );
  }
}

void BallBallConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 ); assert( col < G.cols() );
  assert( m_sphere_idx0 < m_sphere_idx1 );
  assert( 2 * m_sphere_idx0 + 1 < unsigned( G.rows() ) );
  assert( 2 * m_sphere_idx1 + 1 < unsigned( G.rows() ) );

  // Effect on center of mass
  G.insert( 2 * m_sphere_idx0,     col ) =   m_n.x();
  G.insert( 2 * m_sphere_idx0 + 1, col ) =   m_n.y();
  G.insert( 2 * m_sphere_idx1,     col ) = - m_n.x();
  G.insert( 2 * m_sphere_idx1 + 1, col ) = - m_n.y();
}

//void BallBallConstraint::computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& gdotD ) const
//{
//  assert( start_column >= 0 ); assert( start_column < D.cols() );
//  assert( D.cols() == gdotD.size() );
//  assert( num_samples == 1 );
//  assert( m_sphere_idx0 < m_sphere_idx1 );
//  assert( 2 * m_sphere_idx1 + 1 < unsigned( D.rows() ) );
//
//  // Rotate the normal by 90 degrees
//  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
//  const Vector2s t{ -m_n.y(), m_n.x() };
//  assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );
//  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );
//
//  // Effect on center of mass
//  D.insert( 2 * m_sphere_idx0,     start_column ) =   t.x();
//  D.insert( 2 * m_sphere_idx0 + 1, start_column ) =   t.y();
//  D.insert( 2 * m_sphere_idx1,     start_column ) = - t.x();
//  D.insert( 2 * m_sphere_idx1 + 1, start_column ) = - t.y();
//
//  // Relative velocity contribution from kinematic scripting, zero for now
//  gdotD( start_column ) = 0.0;
//}

void BallBallConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( column < unsigned( D.cols() ) ); assert( q.size() % 2 == 0 );
  assert( t.size() == 2 ); assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );
  assert( m_sphere_idx0 < m_sphere_idx1 );
  assert( 2 * m_sphere_idx1 + 1 < unsigned( D.rows() ) );

  // Effect on center of mass
  D.insert( 2 * m_sphere_idx0 + 0, column ) =   t.x();
  D.insert( 2 * m_sphere_idx0 + 1, column ) =   t.y();
  D.insert( 2 * m_sphere_idx1 + 0, column ) = - t.x();
  D.insert( 2 * m_sphere_idx1 + 1, column ) = - t.y();
}

int BallBallConstraint::impactStencilSize() const
{
  return 4;
}

int BallBallConstraint::frictionStencilSize() const
{
  return 4;
}

void BallBallConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_sphere_idx0;
  bodies.second = m_sphere_idx1;
}

void BallBallConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  this->getSimulatedBodyIndices( bodies );
}

void BallBallConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 ); assert( strt_idx < gdotN.size() );
  // No kinematic scripting here, yet
  gdotN( strt_idx ) = 0.0;
}

void BallBallConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
{
  assert( H0.rows() == 2 ); assert( H0.cols() == 2 );
  assert( H1.rows() == 2 ); assert( H1.cols() == 2 );
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
  H1.block<1,2>(0,0) = n;
  H1.block<1,2>(1,0) = t;
}

bool BallBallConstraint::conservesTranslationalMomentum() const
{
  // Teleported collisions do conserve momentum
  return true;
}

bool BallBallConstraint::conservesAngularMomentumUnderImpact() const
{
  // Teleported collisions do not necessarily conserve angular momentum
  return !m_teleported;
}

bool BallBallConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string BallBallConstraint::name() const
{
  if( m_teleported )
  {
    return "teleported_ball_ball";
  }
  else
  {
    return "ball_ball";
  }
}

void BallBallConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<2>( 2 * m_sphere_idx0 ) - m_r0 * m_n;
}

void BallBallConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}

unsigned BallBallConstraint::idx0() const
{
  return m_sphere_idx0;
}

unsigned BallBallConstraint::idx1() const
{
  return m_sphere_idx1;
}

bool BallBallConstraint::teleported() const
{
  return m_teleported;
}

void BallBallConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  const Vector2s t{ -m_n.y(), m_n.x() };
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );
  basis.resize( 2, 2 );
  basis.col( 0 ) = m_n;
  basis.col( 1 ) = t;
}

VectorXs BallBallConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 2 == 0 ); assert( 2 * m_sphere_idx0 + 1 < v.size() ); assert( 2 * m_sphere_idx1 + 1 < v.size() );
  return v.segment<2>( 2 * m_sphere_idx0 ) - v.segment<2>( 2 * m_sphere_idx1 );
}

void BallBallConstraint::setBodyIndex0( const unsigned idx )
{
  m_sphere_idx0 = idx;
}

void BallBallConstraint::setBodyIndex1( const unsigned idx )
{
  m_sphere_idx1 = idx;
}

scalar BallBallConstraint::computePenetrationDepth( const VectorXs& q ) const
{
  if( m_teleported )
  {
    return SCALAR_NAN;
  }
  else
  {
    return std::min( 0.0, ( q.segment<2>( 2 * m_sphere_idx0 ) - q.segment<2>( 2 * m_sphere_idx1 ) ).norm() - ( m_r0 + m_r1 ) );
  }
}

VectorXs BallBallConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  // No kinematic contribution
  return VectorXs::Zero( 2 );
}
