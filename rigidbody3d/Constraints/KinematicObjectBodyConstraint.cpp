// KinematicObjectBodyConstraint.cpp
//
// Breannan Smith
// Last updated: 09/16/2015

#include "KinematicObjectBodyConstraint.h"

#include "FrictionUtilities.h"

#ifndef NDEBUG
#include "scisim/Math/MathUtilities.h"
#endif

KinematicObjectBodyConstraint::KinematicObjectBodyConstraint( const unsigned bdy_idx, const unsigned knmtc_idx, const Vector3s& p, const Vector3s& n, const VectorXs& q )
: m_bdy_idx( bdy_idx )
, m_knmtc_idx( knmtc_idx )
, m_n( n )
, m_r( p - q.segment<3>( 3 * bdy_idx ) )
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
}

KinematicObjectBodyConstraint::~KinematicObjectBodyConstraint()
{}

scalar KinematicObjectBodyConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  return m_n.dot( computeRelativeVelocity( q, v ) );
}

void KinematicObjectBodyConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( q.size() % 12 == 0 );
  assert( col >= 0 );
  assert( col < G.cols() );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  assert( 3 * nbodies + 3 * m_bdy_idx + 2 < unsigned( G.rows() ) );
  G.insert( 3 * m_bdy_idx + 0, col ) = m_n.x();
  G.insert( 3 * m_bdy_idx + 1, col ) = m_n.y();
  G.insert( 3 * m_bdy_idx + 2, col ) = m_n.z();
  const Vector3s ntilde{ m_r.cross( m_n ) };
  G.insert( 3 * ( m_bdy_idx + nbodies ) + 0, col ) = ntilde.x();
  G.insert( 3 * ( m_bdy_idx + nbodies ) + 1, col ) = ntilde.y();
  G.insert( 3 * ( m_bdy_idx + nbodies ) + 2, col ) = ntilde.z();
}

void KinematicObjectBodyConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( column < unsigned( D.cols() ) );
  assert( q.size() % 12 == 0 );
  assert( t.size() == 3 );
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  // Effect on center of mass
  D.insert( 3 * m_bdy_idx + 0, column ) = t.x();
  D.insert( 3 * m_bdy_idx + 1, column ) = t.y();
  D.insert( 3 * m_bdy_idx + 2, column ) = t.z();
  // Effect on orientation
  {
    const Vector3s ttilde{ m_r.cross( Eigen::Map<const Vector3s>{ t.data() } ) };
    D.insert( 3 * ( m_bdy_idx + nbodies ) + 0, column ) = ttilde.x();
    D.insert( 3 * ( m_bdy_idx + nbodies ) + 1, column ) = ttilde.y();
    D.insert( 3 * ( m_bdy_idx + nbodies ) + 2, column ) = ttilde.z();
  }
}

int KinematicObjectBodyConstraint::impactStencilSize() const
{
  return 6;
}

int KinematicObjectBodyConstraint::frictionStencilSize() const
{
  return 6;
}

void KinematicObjectBodyConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_bdy_idx;
  bodies.second = -1;
}

void KinematicObjectBodyConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 );
  assert( strt_idx < gdotN.size() );

  // No kinematic scripting here, yet
  gdotN( strt_idx ) = 0.0;
}

void KinematicObjectBodyConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
{
  assert( H0.rows() == 3 );
  assert( H0.cols() == 6 );
  assert( H1.rows() == 3 );
  assert( H1.cols() == 6 );
  assert( ( m_n - basis.col( 0 ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );

  // Grab the contact normal
  const Vector3s n{ basis.col( 0 ) };
  // Grab the tangent basis
  const Vector3s s{ basis.col( 1 ) };
  const Vector3s t{ basis.col( 2 ) };
  assert( MathUtilities::isRightHandedOrthoNormal( n, s, t, 1.0e-6 ) );

  // Format for H:
  //   n^T  \tilde{n}^T
  //   s^T  \tilde{s}^T
  //   t^T  \tilde{t}^T

  H0.block<1,3>(0,0) = n;
  H0.block<1,3>(0,3) = m_r.cross( n );

  H0.block<1,3>(1,0) = s;
  H0.block<1,3>(1,3) = m_r.cross( s );

  H0.block<1,3>(2,0) = t;
  H0.block<1,3>(2,3) = m_r.cross( t );
}

void KinematicObjectBodyConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_bdy_idx;
  bodies.second = m_knmtc_idx;
}

bool KinematicObjectBodyConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool KinematicObjectBodyConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool KinematicObjectBodyConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string KinematicObjectBodyConstraint::name() const
{
  return "kinematic_object_body";
}

void KinematicObjectBodyConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );

  // Compute the relative velocity to use as a direction for the tangent sample
  Vector3s s{ computeRelativeVelocity( q, v ) };
  // If the relative velocity is zero, any vector will do
  if( m_n.cross( s ).squaredNorm() < 1.0e-9 )
  {
    s = FrictionUtilities::orthogonalVector( m_n );
  }
  // Otherwise project out the component along the normal and normalize the relative velocity
  else
  {
    s = ( s - s.dot( m_n ) * m_n ).normalized();
  }
  // Invert the tangent vector in order to oppose
  s *= -1.0;

  // Create a second orthogonal sample in the tangent plane
  const Vector3s t{ m_n.cross( s ).normalized() }; // Don't need to normalize but it won't hurt

  assert( MathUtilities::isRightHandedOrthoNormal( m_n, s, t, 1.0e-6 ) );
  basis.resize( 3, 3 );
  basis.col( 0 ) = m_n;
  basis.col( 1 ) = s;
  basis.col( 2 ) = t;
}

VectorXs KinematicObjectBodyConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( v.size() / 2 + 3 * m_bdy_idx + 2 < v.size() );

  const unsigned nbodies{ static_cast<unsigned>( v.size() / 6 ) };

  // v_j + omega_j x r_j
  return v.segment<3>( 3 * m_bdy_idx ) + v.segment<3>( 3 * ( nbodies + m_bdy_idx ) ).cross( m_r );
}

void KinematicObjectBodyConstraint::setBodyIndex0( const unsigned idx )
{
  m_bdy_idx = idx;
}

VectorXs KinematicObjectBodyConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  // No kinematic contribution
  return VectorXs::Zero( 3 );
}

void KinematicObjectBodyConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<3>( 3 * m_bdy_idx ) + m_r;
}

void KinematicObjectBodyConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}
