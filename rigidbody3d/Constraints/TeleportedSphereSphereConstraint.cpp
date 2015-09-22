// TeleportedSphereSphereConstraint.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "TeleportedSphereSphereConstraint.h"

#include "FrictionUtilities.h"
#include "scisim/Math/MathUtilities.h"

TeleportedSphereSphereConstraint::TeleportedSphereSphereConstraint( const unsigned idx0, const unsigned idx1, const Vector3s& x0, const Vector3s& x1, const scalar& r0, const scalar& r1 )
: m_idx0( idx0 )
, m_idx1( idx1 )
, m_n( ( x0 - x1 ).normalized() )
// Don't store the absolute collision point, as one ball is effectively in two locations. Instead, store a relative displacement to the collision point.
// p = x0 + ( r0 / ( r0 + r1 ) ) * ( x1 - x0 )
// p - x0
, m_r0( ( r0 / ( r0 + r1 ) ) * ( x1 - x0 ) )
// p - x1
, m_r1( ( r0 / ( r0 + r1 ) - 1.0 ) * ( x1 - x0 ) )
{
  assert( idx0 != idx1 );
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
}

TeleportedSphereSphereConstraint::~TeleportedSphereSphereConstraint()
{}

unsigned TeleportedSphereSphereConstraint::idx0() const
{
  return m_idx0;
}

unsigned TeleportedSphereSphereConstraint::idx1() const
{
  return m_idx1;
}

scalar TeleportedSphereSphereConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( 3 * m_idx0 + 2 < v.size() );
  assert( 3 * m_idx1 + 2 < v.size() );
  // n || r => n dot ( omega cross r ) == 0
  return m_n.dot( v.segment<3>( 3 * m_idx0 ) - v.segment<3>( 3 * m_idx1 ) );
}

void TeleportedSphereSphereConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 );
  assert( col < G.cols() );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  assert( m_idx0 < m_idx1 );
  assert( 3 * m_idx0 + 2 < unsigned( G.rows() ) );
  G.insert( 3 * m_idx0 + 0, col ) = m_n.x();
  G.insert( 3 * m_idx0 + 1, col ) = m_n.y();
  G.insert( 3 * m_idx0 + 2, col ) = m_n.z();
  assert( 3 * m_idx1 + 2 < unsigned( G.rows() ) );
  G.insert( 3 * m_idx1 + 0, col ) = - m_n.x();
  G.insert( 3 * m_idx1 + 1, col ) = - m_n.y();
  G.insert( 3 * m_idx1 + 2, col ) = - m_n.z();
}

void TeleportedSphereSphereConstraint::computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const
{
  assert( start_column >= 0 );
  assert( start_column < D.cols() );
  assert( num_samples > 0 );
  assert( start_column + num_samples - 1 < D.cols() );
  assert( q.size() % 12 == 0 );
  assert( q.size() == 2 * v.size() );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  std::vector<Vector3s> friction_disk;
  {
    // Compute the relative velocity
    Vector3s tangent_suggestion{ computeRelativeVelocity( q, v ) };
    if( tangent_suggestion.cross( m_n ).squaredNorm() < 1.0e-9 )
    {
      tangent_suggestion = FrictionUtilities::orthogonalVector( m_n );
    }
    tangent_suggestion *= -1.0;

    // Sample the friction disk
    friction_disk.resize( num_samples );
    FrictionUtilities::generateOrthogonalVectors( m_n, friction_disk, tangent_suggestion );
  }

  // For each sample of the friction disk
  assert( m_idx0 < m_idx1 );
  for( int i = 0; i < num_samples; ++i )
  {
    const int cur_col{ start_column + i };
    assert( cur_col >= 0 );
    assert( cur_col < D.cols() );

    // Effect on center of mass of body i
    assert( 3 * m_idx0 + 2 < unsigned( D.rows() ) );
    D.insert( 3 * m_idx0 + 0, cur_col ) = friction_disk[i].x();
    D.insert( 3 * m_idx0 + 1, cur_col ) = friction_disk[i].y();
    D.insert( 3 * m_idx0 + 2, cur_col ) = friction_disk[i].z();
    // Effect on orientation of body i
    {
      assert( m_n.cross( m_r0 ).norm() <= 1.0e-6 );
      const Vector3s ntilde{ m_r0.cross( friction_disk[i] ) };
      assert( 3 * ( m_idx0 + nbodies ) + 2 < unsigned( D.rows() ) );
      D.insert( 3 * ( m_idx0 + nbodies ) + 0, cur_col ) = ntilde.x();
      D.insert( 3 * ( m_idx0 + nbodies ) + 1, cur_col ) = ntilde.y();
      D.insert( 3 * ( m_idx0 + nbodies ) + 2, cur_col ) = ntilde.z();
    }

    // Effect on center of mass of body j
    assert( 3 * m_idx1 + 2 < unsigned( D.rows() ) );
    D.insert( 3 * m_idx1 + 0, cur_col ) = - friction_disk[i].x();
    D.insert( 3 * m_idx1 + 1, cur_col ) = - friction_disk[i].y();
    D.insert( 3 * m_idx1 + 2, cur_col ) = - friction_disk[i].z();
    // Effect on orientation of body j
    {
      assert( m_n.cross( m_r1 ).norm() <= 1.0e-6 );
      const Vector3s ntilde{ m_r1.cross( friction_disk[i] ) };
      assert( 3 * ( m_idx1 + nbodies ) + 2 < unsigned( D.rows() ) );
      D.insert( 3 * ( m_idx1 + nbodies ) + 0, cur_col ) = - ntilde.x();
      D.insert( 3 * ( m_idx1 + nbodies ) + 1, cur_col ) = - ntilde.y();
      D.insert( 3 * ( m_idx1 + nbodies ) + 2, cur_col ) = - ntilde.z();
    }

    // No relative velocity contribution from kinematic scripting
    assert( cur_col < drel.size() );
    drel( cur_col ) = 0.0;
  }
}

void TeleportedSphereSphereConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( column < unsigned( D.cols() ) );
  assert( q.size() % 12 == 0 );
  assert( t.size() == 3 );
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );
  assert( m_idx0 < m_idx1 );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  // Effect on center of mass of body i
  D.insert( 3 * m_idx0 + 0, column ) = t.x();
  D.insert( 3 * m_idx0 + 1, column ) = t.y();
  D.insert( 3 * m_idx0 + 2, column ) = t.z();
  // Effect on orientation of body i
  {
    assert( m_n.cross( m_r0 ).norm() <= 1.0e-6 );
    const Vector3s ntilde{ m_r0.cross( Eigen::Map<const Vector3s>{ t.data() } ) };
    assert( 3 * ( m_idx0 + nbodies ) + 2 < unsigned( D.rows() ) );
    D.insert( 3 * ( m_idx0 + nbodies ) + 0, column ) = ntilde.x();
    D.insert( 3 * ( m_idx0 + nbodies ) + 1, column ) = ntilde.y();
    D.insert( 3 * ( m_idx0 + nbodies ) + 2, column ) = ntilde.z();
  }

  // Effect on center of mass of body j
  D.insert( 3 * m_idx1 + 0, column ) = - t.x();
  D.insert( 3 * m_idx1 + 1, column ) = - t.y();
  D.insert( 3 * m_idx1 + 2, column ) = - t.z();
  // Effect on orientation of body j
  {
    assert( m_n.cross( m_r1 ).norm() <= 1.0e-6 );
    const Vector3s ntilde{ m_r1.cross( Eigen::Map<const Vector3s>{ t.data() } ) };
    assert( 3 * ( m_idx1 + nbodies ) + 2 < unsigned( D.rows() ) );
    D.insert( 3 * ( m_idx1 + nbodies ) + 0, column ) = - ntilde.x();
    D.insert( 3 * ( m_idx1 + nbodies ) + 1, column ) = - ntilde.y();
    D.insert( 3 * ( m_idx1 + nbodies ) + 2, column ) = - ntilde.z();
  }
}

int TeleportedSphereSphereConstraint::impactStencilSize() const
{
  return 6;
}

int TeleportedSphereSphereConstraint::frictionStencilSize() const
{
  return 12;
}

void TeleportedSphereSphereConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx0;
  bodies.second = m_idx1;
}

void TeleportedSphereSphereConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx0;
  bodies.second = m_idx1;
}

void TeleportedSphereSphereConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  // No relative velocity contribution from kinematic scripting
  gdotN( strt_idx ) = 0.0;
}

bool TeleportedSphereSphereConstraint::conservesTranslationalMomentum() const
{
  return true;
}

bool TeleportedSphereSphereConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool TeleportedSphereSphereConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string TeleportedSphereSphereConstraint::name() const
{
  return "teleported_sphere_sphere";
}

void TeleportedSphereSphereConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
{
  assert( H0.rows() == 3 );
  assert( H0.cols() == 6 );
  assert( H1.rows() == 3 );
  assert( H1.cols() == 6 );

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
  assert( m_r0.cross( n ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  H0.block<1,3>(0,3).setZero();

  H0.block<1,3>(1,0) = s;
  H0.block<1,3>(1,3) = m_r0.cross( s );

  H0.block<1,3>(2,0) = t;
  H0.block<1,3>(2,3) = m_r0.cross( t );

  H1.block<1,3>(0,0) = n;
  assert( m_r1.cross( n ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  H1.block<1,3>(0,3).setZero();

  H1.block<1,3>(1,0) = s;
  H1.block<1,3>(1,3) = m_r1.cross( s );

  H1.block<1,3>(2,0) = t;
  H1.block<1,3>(2,3) = m_r1.cross( t );
}

void TeleportedSphereSphereConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
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

VectorXs TeleportedSphereSphereConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( v.size() / 2 + 3 * m_idx0 + 2 < v.size() );
  assert( v.size() / 2 + 3 * m_idx1 + 2 < v.size() );

  const unsigned nbodies{ static_cast<unsigned>( v.size() / 6 ) };

  // v_1 + omega_1 x r_1 - ( v_0 + omega_0 x r_0 )
  return v.segment<3>( 3 * m_idx0 ) + v.segment<3>( 3 * ( nbodies + m_idx0 ) ).cross( m_r0 ) - v.segment<3>( 3 * m_idx1 ) - v.segment<3>( 3 * ( nbodies + m_idx1 ) ).cross( m_r1 );
}

void TeleportedSphereSphereConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx0 = idx;
}

void TeleportedSphereSphereConstraint::setBodyIndex1( const unsigned idx )
{
  m_idx1 = idx;
}

void TeleportedSphereSphereConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<3>( 3 * m_idx0 ) + m_r0;
}

void TeleportedSphereSphereConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}

VectorXs TeleportedSphereSphereConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  // No kinematic contribution
  return VectorXs::Zero( 3 );
}
