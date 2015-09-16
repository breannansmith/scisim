// SphereSphereConstraint.cpp
//
// Breannan Smith
// Last updated: 09/16/2015

#include "SphereSphereConstraint.h"

#include "FrictionUtilities.h"
#include "SCISim/Math/MathUtilities.h"

bool SphereSphereConstraint::isActive( const Vector3s& x0, const Vector3s& x1, const scalar& r0, const scalar& r1 )
{
  return ( x0 - x1 ).squaredNorm() <= ( r0 + r1 ) * ( r0 + r1 );
}

SphereSphereConstraint::SphereSphereConstraint( const unsigned sphere_idx0, const unsigned sphere_idx1, const Vector3s& n, const Vector3s& p, const scalar& r0, const scalar& r1 )
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

SphereSphereConstraint::~SphereSphereConstraint()
{}

scalar SphereSphereConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( 3 * m_idx0 + 2 < v.size() );
  assert( 3 * m_idx1 + 2 < v.size() );
  // n || r => n dot ( omega cross r ) == 0, so computeRelativeVelocity not used
  return m_n.dot( v.segment<3>( 3 * m_idx0 ) - v.segment<3>( 3 * m_idx1 ) );
}

void SphereSphereConstraint::resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const VectorXs& vin, const scalar& ndotv, VectorXs& vout, scalar& alpha ) const
{
  assert( CoR >= 0.0 );
  assert( CoR <= 1.0 );
  assert( ndotv < 0.0 );
  assert( vin.size() == vout.size() );
  assert( vin.size() % 3 == 0 );
  assert( 3 * m_idx0 + 2 < vin.size() );
  assert( 3 * m_idx1 + 2 < vin.size() );
  assert( M.rows() == M.cols() );
  assert( M.nonZeros() == 2 * vin.size() );

  const Eigen::Map<const VectorXs> m{ M.valuePtr(), vin.size() };
  assert( m( 3 * m_idx1 ) == m( 3 * m_idx1 + 1 ) );
  assert( m( 3 * m_idx1 ) == m( 3 * m_idx1 + 2 ) );
  assert( m( 3 * m_idx0 ) == m( 3 * m_idx0 + 1 ) );
  assert( m( 3 * m_idx0 ) == m( 3 * m_idx0 + 2 ) );

  const scalar& m0{ m( 3 * m_idx0 ) };
  assert( m0 > 0.0 );
  const scalar& m1{ m( 3 * m_idx1 ) };
  assert( m1 > 0.0 );

  // Compute the impulse
  alpha = - ( 1 + CoR ) * ndotv * m0 * m1 / ( m0 + m1 );
  assert( alpha >= 0.0 );
  vout.segment<3>( 3 * m_idx0 ) += m_n * alpha / m0;
  vout.segment<3>( 3 * m_idx1 ) -= m_n * alpha / m1;
}

void SphereSphereConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 );
  assert( col < G.cols() );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  assert( m_idx0 < m_idx1 );
  assert( 3 * m_idx0 + 2 < unsigned( G.rows() ) );
  G.insert( 3 * m_idx0 + 0, col ) =  m_n.x();
  G.insert( 3 * m_idx0 + 1, col ) =  m_n.y();
  G.insert( 3 * m_idx0 + 2, col ) =  m_n.z();
  assert( 3 * m_idx1 + 2 < unsigned( G.rows() ) );
  G.insert( 3 * m_idx1 + 0, col ) = -m_n.x();
  G.insert( 3 * m_idx1 + 1, col ) = -m_n.y();
  G.insert( 3 * m_idx1 + 2, col ) = -m_n.z();
}

void SphereSphereConstraint::computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const
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
    Vector3s tangent_suggestion = computeRelativeVelocity( q, v );
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
      const Vector3s ri{ m_p - q.segment<3>( 3 * m_idx0 ) };
      assert( m_n.cross( ri ).norm() <= 1.0e-6 );
      const Vector3s ntilde{ ri.cross( friction_disk[i] ) };
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
      const Vector3s rj{ m_p - q.segment<3>( 3 * m_idx1 ) };
      assert( m_n.cross( rj ).norm() <= 1.0e-6 );
      const Vector3s ntilde{ rj.cross( friction_disk[i] ) };
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

void SphereSphereConstraint::computeSmoothGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, SparseMatrixsc& D ) const
{
  assert( start_column >= 0 );
  assert( start_column < D.cols() );
  assert( start_column+1 < D.cols() );
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
  assert( m_idx0 < m_idx1 );
  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };
  for( int i = 0; i < 2; ++i )
  {
    const int cur_col{ start_column + i };
    assert( cur_col >= 0 );
    assert( cur_col < D.cols() );

    // Effect on center of mass of body i
    D.insert( 3 * m_idx0 + 0, cur_col ) = friction_disk[i].x();
    D.insert( 3 * m_idx0 + 1, cur_col ) = friction_disk[i].y();
    D.insert( 3 * m_idx0 + 2, cur_col ) = friction_disk[i].z();
    // Effect on orientation of body i
    {
      const Vector3s ri{ m_p - q.segment<3>( 3 * m_idx0 ) };
      assert( m_n.cross( ri ).norm() <= 1.0e-6 );
      const Vector3s ntilde{ ri.cross( friction_disk[i] ) };
      D.insert( 3 * ( m_idx0 + nbodies ) + 0, cur_col ) = ntilde.x();
      D.insert( 3 * ( m_idx0 + nbodies ) + 1, cur_col ) = ntilde.y();
      D.insert( 3 * ( m_idx0 + nbodies ) + 2, cur_col ) = ntilde.z();
    }

    // Effect on center of mass of body j
    D.insert( 3 * m_idx1 + 0, cur_col ) = -friction_disk[i].x();
    D.insert( 3 * m_idx1 + 1, cur_col ) = -friction_disk[i].y();
    D.insert( 3 * m_idx1 + 2, cur_col ) = -friction_disk[i].z();
    // Effect on orientation of body j
    {
      const Vector3s rj{ m_p - q.segment<3>( 3 * m_idx1 ) };
      assert( m_n.cross( rj ).norm() <= 1.0e-6 );
      const Vector3s ntilde{ rj.cross( friction_disk[i] ) };
      D.insert( 3 * ( m_idx1 + nbodies ) + 0, cur_col ) = -ntilde.x();
      D.insert( 3 * ( m_idx1 + nbodies ) + 1, cur_col ) = -ntilde.y();
      D.insert( 3 * ( m_idx1 + nbodies ) + 2, cur_col ) = -ntilde.z();
    }
  }
}

void SphereSphereConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
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
    const Vector3s ri{ m_p - q.segment<3>( 3 * m_idx0 ) };
    assert( m_n.cross( ri ).norm() <= 1.0e-6 );
    const Vector3s ttilde{ ri.cross( Eigen::Map<const Vector3s>{ t.data() } ) };
    D.insert( 3 * ( m_idx0 + nbodies ) + 0, column ) = ttilde.x();
    D.insert( 3 * ( m_idx0 + nbodies ) + 1, column ) = ttilde.y();
    D.insert( 3 * ( m_idx0 + nbodies ) + 2, column ) = ttilde.z();
  }

  // Effect on center of mass of body j
  D.insert( 3 * m_idx1 + 0, column ) = - t.x();
  D.insert( 3 * m_idx1 + 1, column ) = - t.y();
  D.insert( 3 * m_idx1 + 2, column ) = - t.z();
  // Effect on orientation of body j
  {
    const Vector3s rj{ m_p - q.segment<3>( 3 * m_idx1 ) };
    assert( m_n.cross( rj ).norm() <= 1.0e-6 );
    const Vector3s ttilde{ rj.cross( Eigen::Map<const Vector3s>{ t.data() } ) };
    D.insert( 3 * ( m_idx1 + nbodies ) + 0, column ) = - ttilde.x();
    D.insert( 3 * ( m_idx1 + nbodies ) + 1, column ) = - ttilde.y();
    D.insert( 3 * ( m_idx1 + nbodies ) + 2, column ) = - ttilde.z();
  }
}

int SphereSphereConstraint::impactStencilSize() const
{
  return 6;
}

int SphereSphereConstraint::frictionStencilSize() const
{
  return 12;
}

void SphereSphereConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx0;
  bodies.second = m_idx1;
}

void SphereSphereConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  this->getSimulatedBodyIndices( bodies );
}

void SphereSphereConstraint::computeFrictionMask( const int nbodies, VectorXs& friction_mask ) const
{
  assert( 3 * m_idx0 + 2 < friction_mask.size() );
  friction_mask.segment<3>( 3 * m_idx0 ).setConstant( 1.0 );

  assert( 3 * ( m_idx0 + nbodies ) + 2 < friction_mask.size() );
  friction_mask.segment<3>( 3 * ( m_idx0 + nbodies ) ).setConstant( 1.0 );

  assert( 3 * m_idx1 + 2 < friction_mask.size() );
  friction_mask.segment<3>( 3 * m_idx1 ).setConstant( 1.0 );

  assert( 3 * ( m_idx1 + nbodies ) + 2 < friction_mask.size() );
  friction_mask.segment<3>( 3 * ( m_idx1 + nbodies ) ).setConstant( 1.0 );
}

void SphereSphereConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  // No relative velocity contribution from kinematic scripting
  gdotN( strt_idx ) = 0.0;
}

void SphereSphereConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
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

  // Generate arms
  const Vector3s ri{ m_p - q.segment<3>( 3 * m_idx0 ) };
  const Vector3s rj{ m_p - q.segment<3>( 3 * m_idx1 ) };

  // Format for H:
  //   n^T  \tilde{n}^T
  //   s^T  \tilde{s}^T
  //   t^T  \tilde{t}^T

  H0.block<1,3>(0,0) = n;
  assert( ri.cross( n ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  H0.block<1,3>(0,3).setZero();

  H0.block<1,3>(1,0) = s;
  H0.block<1,3>(1,3) = ri.cross( s );

  H0.block<1,3>(2,0) = t;
  H0.block<1,3>(2,3) = ri.cross( t );

  H1.block<1,3>(0,0) = n;
  assert( rj.cross( n ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  H1.block<1,3>(0,3).setZero();

  H1.block<1,3>(1,0) = s;
  H1.block<1,3>(1,3) = rj.cross( s );

  H1.block<1,3>(2,0) = t;
  H1.block<1,3>(2,3) = rj.cross( t );
}

void SphereSphereConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );

  // Compute the relative velocity to use as a direction for the tangent sample
  Vector3s s = computeRelativeVelocity( q, v );
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


bool SphereSphereConstraint::conservesTranslationalMomentum() const
{
  return true;
}

bool SphereSphereConstraint::conservesAngularMomentumUnderImpact() const
{
  return true;
}

bool SphereSphereConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return true;
}

std::string SphereSphereConstraint::name() const
{
  return "sphere_sphere";
}

VectorXs SphereSphereConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( v.size() / 2 + 3 * m_idx0 + 2 < v.size() );
  assert( v.size() / 2 + 3 * m_idx1 + 2 < v.size() );

  const unsigned nbodies{ static_cast<unsigned>( v.size() / 6 ) };

  // Point of contact relative to each body's center of mass
  const Vector3s r0 = m_p - q.segment<3>( 3 * m_idx0 );
  assert( m_n.cross( r0 ).norm() <= 1.0e-6 );
  const Vector3s r1 = m_p - q.segment<3>( 3 * m_idx1 );
  assert( m_n.cross( r1 ).norm() <= 1.0e-6 );

  // v_0 + omega_0 x r_0 - ( v_1 + omega_1 x r_1 )
  return v.segment<3>( 3 * m_idx0 ) + v.segment<3>( 3 * ( nbodies + m_idx0 ) ).cross( r0 ) - v.segment<3>( 3 * m_idx1 ) - v.segment<3>( 3 * ( nbodies + m_idx1 ) ).cross( r1 );
}

void SphereSphereConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx0 = idx;
}

void SphereSphereConstraint::setBodyIndex1( const unsigned idx )
{
  m_idx1 = idx;
}

scalar SphereSphereConstraint::computePenetrationDepth( const VectorXs& q ) const
{
  return std::min( 0.0, ( q.segment<3>( 3 * m_idx0 ) - q.segment<3>( 3 * m_idx1 ) ).norm() - m_r0 - m_r1 );
}

static scalar overlapVolumeGivenDistanceAndRadii( const scalar& d, const scalar& r, const scalar& R )
{
  assert( d != 0.0 );
  return MathDefines::PI<scalar>() * ( R + r - d ) * ( R + r - d ) * ( d * d + 2.0 * d * r - 3.0 * r * r + 2.0 * d * R + 6.0 * r * R - 3.0 * R * R ) / ( 12.0 * d );
}

scalar SphereSphereConstraint::computeOverlapVolume( const VectorXs& q ) const
{
  // Compute the distance between the spheres
  const scalar d{ ( q.segment<3>( 3 * m_idx0 ) - q.segment<3>( 3 * m_idx1 ) ).norm() };

  // If there spheres are not touching, no overlap volume
  if( d > m_r0 + m_r1 )
  {
    return 0.0;
  }

  const scalar r{ std::min( m_r0, m_r1 ) };
  const scalar R{ std::max( m_r0, m_r1 ) };

  // If one sphere is totally within another, return the smaller sphere's volume
  if( d + r <= R )
  {
    return 4.0 * MathDefines::PI<scalar>() * r * r * r / 3.0;
  }

  // Otherwise, compute the volume of intersection
  return overlapVolumeGivenDistanceAndRadii( d, r, R );
}

VectorXs SphereSphereConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  // No kinematic contribution
  return VectorXs::Zero( 3 );
}

void SphereSphereConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = m_p;
}

void SphereSphereConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}
