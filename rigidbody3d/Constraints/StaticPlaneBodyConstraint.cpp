// StaticPlaneBodyConstraint.cpp
//
// Breannan Smith
// Last updated: 09/16/2015

#include "StaticPlaneBodyConstraint.h"

#include "FrictionUtilities.h"
#include "SCISim/Math/MathUtilities.h"

StaticPlaneBodyConstraint::StaticPlaneBodyConstraint( const unsigned body_idx, const Vector3s& collision_point, const Vector3s& n, const VectorXs& q, const unsigned plane_idx )
: m_idx_body( body_idx )
, m_n( n )
, m_r( collision_point - q.segment<3>( 3 * m_idx_body ) )
, m_idx_plane( plane_idx )
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
}

StaticPlaneBodyConstraint::~StaticPlaneBodyConstraint()
{}

scalar StaticPlaneBodyConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( 3 * ( m_idx_body + v.size() / 6 ) + 2 < v.size() );
  return m_n.dot( computeRelativeVelocity( q, v ) );
}

//void StaticPlaneBodyConstraint::exertImpulse( const VectorXs& q, const scalar& lambda, const SparseMatrixsc& Minv, VectorXs& vout ) const
//{
//  assert( vout.size() % 6 == 0 ); assert( 3 * ( m_idx_body + vout.size() / 6 ) + 2 < vout.size() );
//  assert( Minv.rows() == Minv.cols() ); assert( Minv.nonZeros() == 2 * vout.size() );
//  assert( Minv.valuePtr()[ 3 * m_idx_body ] > 0.0 );
//  assert( Minv.valuePtr()[ 3 * m_idx_body ] == Minv.valuePtr()[ 3 * m_idx_body + 1 ] );
//  assert( Minv.valuePtr()[ 3 * m_idx_body ] == Minv.valuePtr()[ 3 * m_idx_body + 2 ] );
//
//  const unsigned nbodies = vout.size() / 6;
//
//  vout.segment<3>( 3 * m_idx_body ) += Minv.valuePtr()[ 3 * m_idx_body ] * lambda * m_n;
//
//  const Vector3s ntilde = m_r.cross( m_n );
//
//  // Retrieve the inverse mass matrix for this body
//  const Eigen::Map<const Matrix33sr> Iinv = Eigen::Map<const Matrix33sr>( &Minv.valuePtr()[3 * nbodies + 9 * m_idx_body] );
//  assert( ( Iinv - Iinv.transpose() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
//
//  vout.segment<3>( 3 * ( m_idx_body + nbodies ) ) += lambda * Iinv * ntilde;
//}

void StaticPlaneBodyConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 );
  assert( col < G.cols() );
  assert( q.size() % 12 == 0 );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  G.insert( 3 * m_idx_body + 0, col ) = m_n.x();
  G.insert( 3 * m_idx_body + 1, col ) = m_n.y();
  G.insert( 3 * m_idx_body + 2, col ) = m_n.z();

  {
    const Vector3s ntilde{ m_r.cross( m_n ) };
    G.insert( 3 * ( m_idx_body + nbodies ) + 0, col ) = ntilde.x();
    G.insert( 3 * ( m_idx_body + nbodies ) + 1, col ) = ntilde.y();
    G.insert( 3 * ( m_idx_body + nbodies ) + 2, col ) = ntilde.z();
  }
}

void StaticPlaneBodyConstraint::computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const
{
  assert( start_column >= 0 );
  assert( start_column < D.cols() );
  assert( num_samples > 0 );
  assert( start_column + num_samples - 1 < D.cols() );
  assert( q.size() % 12 == 0 );
  assert( q.size() == 2 * v.size() );

  std::vector<Vector3s> friction_disk{ static_cast<std::vector<Vector3s>::size_type>( num_samples ) };
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
  assert( unsigned( num_samples ) == friction_disk.size() );

  // For each sample of the friction disk
  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };
  for( unsigned friction_sample = 0; friction_sample < unsigned( num_samples ); ++friction_sample )
  {
    const unsigned cur_col{ start_column + friction_sample };
    assert( cur_col < unsigned( D.cols() ) );

    // Effect on center of mass
    assert( fabs( friction_disk[friction_sample].norm() - 1.0 ) <= 1.0e-6 );
    D.insert( 3 * m_idx_body + 0, cur_col ) = friction_disk[friction_sample].x();
    D.insert( 3 * m_idx_body + 1, cur_col ) = friction_disk[friction_sample].y();
    D.insert( 3 * m_idx_body + 2, cur_col ) = friction_disk[friction_sample].z();

    // Effect on orientation
    {
      const Vector3s ntilde{ m_r.cross( friction_disk[friction_sample] ) };
      D.insert( 3 * ( nbodies + m_idx_body ) + 0, cur_col ) = ntilde.x();
      D.insert( 3 * ( nbodies + m_idx_body ) + 1, cur_col ) = ntilde.y();
      D.insert( 3 * ( nbodies + m_idx_body ) + 2, cur_col ) = ntilde.z();
    }

    // Relative velocity contribution from kinematic scripting
    assert( cur_col < drel.size() );
    // Zero for now
    drel( cur_col ) = 0.0;
  }
}

void StaticPlaneBodyConstraint::computeSmoothGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, SparseMatrixsc& D ) const
{
  assert( start_column >= 0 );
  assert( start_column < D.cols() );
  assert( start_column + 1 < D.cols() );
  assert( q.size() % 12 == 0 );
  assert( q.size() == 2 * v.size() );

  std::vector<Vector3s> friction_disk{ 2 };

  // Compute the relative velocity to use as a direction for the tangent sample
  friction_disk[0] = computeRelativeVelocity( q, v );
  // If the relative velocity is zero, any vector will do
  if( friction_disk[0].cross( m_n ).squaredNorm() < 1.0e-9 )
  {
    friction_disk[0] = FrictionUtilities::orthogonalVector( m_n );
  }
  // Otherwise project out the component along the normal and normalize the relative velocity
  else
  {
    friction_disk[0] = ( friction_disk[0] - friction_disk[0].dot( m_n ) * m_n ).normalized();
  }
  // Invert the tangent vector in order to oppose
  friction_disk[0] *= -1.0;

  // Create a second orthogonal sample in the tangent plane
  friction_disk[1] = m_n.cross( friction_disk[0] ).normalized(); // Don't need to normalize but it won't hurt
  assert( MathUtilities::isRightHandedOrthoNormal( m_n, friction_disk[0], friction_disk[1], 1.0e-6 ) );

  // For each sample of the friction disk
  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };
  for( unsigned friction_sample = 0; friction_sample < 2; ++friction_sample )
  {
    const unsigned cur_col{ start_column + friction_sample };
    assert( cur_col < unsigned( D.cols() ) );

    // Effect on center of mass
    D.insert( 3 * m_idx_body + 0, cur_col ) = friction_disk[friction_sample].x();
    D.insert( 3 * m_idx_body + 1, cur_col ) = friction_disk[friction_sample].y();
    D.insert( 3 * m_idx_body + 2, cur_col ) = friction_disk[friction_sample].z();

    // Effect on orientation
    {
      const Vector3s ntilde{ m_r.cross( friction_disk[friction_sample] ) };
      D.insert( 3 * ( nbodies + m_idx_body ) + 0, cur_col ) = ntilde.x();
      D.insert( 3 * ( nbodies + m_idx_body ) + 1, cur_col ) = ntilde.y();
      D.insert( 3 * ( nbodies + m_idx_body ) + 2, cur_col ) = ntilde.z();
    }
  }
}

void StaticPlaneBodyConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( column < unsigned( D.cols() ) );
  assert( q.size() % 12 == 0 );
  assert( t.size() == 3 );
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  // Effect on center of mass of body i
  D.insert( 3 * m_idx_body + 0, column ) = t.x();
  D.insert( 3 * m_idx_body + 1, column ) = t.y();
  D.insert( 3 * m_idx_body + 2, column ) = t.z();
  // Effect on orientation of body i
  {
    const Vector3s ntilde{ m_r.cross( Eigen::Map<const Vector3s>{ t.data() } ) };
    D.insert( 3 * ( m_idx_body + nbodies ) + 0, column ) = ntilde.x();
    D.insert( 3 * ( m_idx_body + nbodies ) + 1, column ) = ntilde.y();
    D.insert( 3 * ( m_idx_body + nbodies ) + 2, column ) = ntilde.z();
  }
}

int StaticPlaneBodyConstraint::impactStencilSize() const
{
  return 6;
}

int StaticPlaneBodyConstraint::frictionStencilSize() const
{
  return 6;
}

void StaticPlaneBodyConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx_body;
  bodies.second = -1;
}

void StaticPlaneBodyConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  this->getSimulatedBodyIndices( bodies );
}

void StaticPlaneBodyConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 );
  assert( strt_idx < gdotN.size() );

  // No kinematic scripting here, yet
  gdotN( strt_idx ) = 0.0;
}

void StaticPlaneBodyConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
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

  H0.block<1,3>(0,0) = n;
  H0.block<1,3>(0,3) = m_r.cross( n );

  H0.block<1,3>(1,0) = s;
  H0.block<1,3>(1,3) = m_r.cross( s );

  H0.block<1,3>(2,0) = t;
  H0.block<1,3>(2,3) = m_r.cross( t );
}

bool StaticPlaneBodyConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool StaticPlaneBodyConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool StaticPlaneBodyConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string StaticPlaneBodyConstraint::name() const
{
  return "static_plane_body";
}

void StaticPlaneBodyConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<3>( 3 * m_idx_body ) + m_r;
}

void StaticPlaneBodyConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}

unsigned StaticPlaneBodyConstraint::getStaticObjectIndex() const
{
  return m_idx_plane;
}

void StaticPlaneBodyConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
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

VectorXs StaticPlaneBodyConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( 3 * ( m_idx_body + v.size() / 6 ) + 2 < v.size() );

  const unsigned nbodies{ static_cast<unsigned>( v.size() / 6 ) };

  // v + omega x r
  return v.segment<3>( 3 * m_idx_body ) + v.segment<3>( 3 * ( nbodies + m_idx_body ) ).cross( m_r );
}

void StaticPlaneBodyConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx_body = idx;
}

VectorXs StaticPlaneBodyConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  // IMPORTANT NOTE: This code has not been updated to treat kinematic boundaries, yet. If this is important to you, please email smith@cs.columbia.edu
  // TODO: Fixing this will require mirroring the structure of StaticPlaneSphereConstraint
  return VectorXs::Zero( 3 );
}
