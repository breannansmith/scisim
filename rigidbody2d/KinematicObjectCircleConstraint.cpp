// KinematicObjectCircleConstraint.cpp
//
// Breannan Smith
// Last updated: 12/07/2015

#include "KinematicObjectCircleConstraint.h"

#include "scisim/Math/MathUtilities.h"

KinematicObjectCircleConstraint::KinematicObjectCircleConstraint( const unsigned sim_bdy_idx, const scalar& sim_bdy_r, const Vector2s& n, const unsigned knmtc_bdy_idx )
: m_sim_idx( sim_bdy_idx )
, m_r( sim_bdy_r )
, m_n( n )
, m_kinematic_index( knmtc_bdy_idx )
{
  assert( m_r > 0.0 );
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
}

scalar KinematicObjectCircleConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 ); assert( 3 * m_sim_idx + 1 < v.size() );
  return m_n.dot( v.segment<2>( 3 * m_sim_idx ) - computeKinematicRelativeVelocity( q, v ) );
}

void KinematicObjectCircleConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 );
  assert( col < G.cols() );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  assert( 3 * m_sim_idx + 1 < unsigned( G.rows() ) );
  G.insert( 3 * m_sim_idx + 0, col ) =  m_n.x();
  G.insert( 3 * m_sim_idx + 1, col ) =  m_n.y();
}

int KinematicObjectCircleConstraint::impactStencilSize() const
{
  return 2;
}

void KinematicObjectCircleConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_sim_idx;
  bodies.second = -1;
}

void KinematicObjectCircleConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_sim_idx;
  bodies.second = m_kinematic_index;
}

void KinematicObjectCircleConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  // No relative velocity contribution from kinematic scripting, for now
  gdotN( strt_idx ) = 0.0;
}

void KinematicObjectCircleConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
{
  assert( H0.rows() == 2 );
  assert( H0.cols() == 3 );
  assert( H1.rows() == 2 );
  assert( H1.cols() == 3 );
  assert( ( basis * basis.transpose() - MatrixXXsc::Identity( 2, 2 ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-6 );

  // Grab the contact normal
  const Vector2s n{ basis.col( 0 ) };
  // Grab the tangent basis
  const Vector2s t{ basis.col( 1 ) };

  // Generate arms
  const Vector2s r{ - m_r * n };

  // Format for H:
  //   n^T  r x n
  //   t^T  r x t

  H0.block<1,2>(0,0) = n;
  assert( fabs( MathUtilities::cross( r, n ) ) <= 1.0e-6 );
  H0(0,2) = 0.0;

  H0.block<1,2>(1,0) = t;
  H0(1,2) = MathUtilities::cross( r, t );
}

void KinematicObjectCircleConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  const Vector2s t{ -m_n.y(), m_n.x() };
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 ); assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );
  basis.resize( 2, 2 );
  basis.col( 0 ) = m_n;
  basis.col( 1 ) = t;
}

bool KinematicObjectCircleConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool KinematicObjectCircleConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool KinematicObjectCircleConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string KinematicObjectCircleConstraint::name() const
{
  return "kinematic_object_circle";
}

VectorXs KinematicObjectCircleConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 );
  assert( 3 * m_sim_idx + 2 < v.size() );

  // Point of contact relative to the simulated body's center of mass
  const Vector2s r{ - m_r * m_n };
  assert( fabs( MathUtilities::cross( m_n, r ) ) <= 1.0e-6 );

  // Rotate 90 degrees counter clockwise for computing the torque
  const Vector2s t{ -r.y(), r.x() };

  // v + omega x r - kinematic_vel
  return v.segment<2>( 3 * m_sim_idx ) + v( 3 * m_sim_idx + 2 ) * t - computeKinematicRelativeVelocity( q, v );
}

void KinematicObjectCircleConstraint::setBodyIndex0( const unsigned idx )
{
  m_sim_idx = idx;
}

VectorXs KinematicObjectCircleConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  // The kinematic object must be stationary, for now
  return VectorXs::Zero( 2 );
}

void KinematicObjectCircleConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<2>( 3 * m_sim_idx ) - m_r * m_n;
}

void KinematicObjectCircleConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}
