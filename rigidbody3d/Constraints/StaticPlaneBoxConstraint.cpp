// StaticPlaneBoxConstraint.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "StaticPlaneBoxConstraint.h"

#include "FrictionUtilities.h"
#include "SCISim/Math/MathUtilities.h"

static Vector3s getBodySpaceCorner( const Vector3s& half_width, const short i )
{
  assert( i >= 0 );
  assert( i < 8 );
  // TODO: This is unreadable, fix
  return ( half_width.array() * Vector3s( 2 * ( i % 2 ) - 1, 2 * ( ( i >> 1 ) % 2 ) - 1, 2 * ( ( i >> 2 ) % 2 ) - 1 ).array() ).matrix();
}

bool StaticPlaneBoxConstraint::isActive( const Vector3s& x_plane, const Vector3s& n, const Vector3s& x_box, const Matrix33sr& R, const Vector3s& half_width, std::vector<short>& active_corners )
{
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );

  for( short corner_num = 0; corner_num < 8; ++corner_num )
  {
    const Vector3s world_space_point{ R * getBodySpaceCorner( half_width, corner_num ) + x_box };
    if( n.dot( world_space_point - x_plane ) <= 0.0 )
    {
      active_corners.emplace_back( corner_num );
    }
  }

  return !active_corners.empty();
}

StaticPlaneBoxConstraint::StaticPlaneBoxConstraint( const unsigned box_idx, const short corner_num, const Vector3s& n, const Vector3s& half_width, const VectorXs& q, const unsigned plane_idx )
: m_idx_box( box_idx )
, m_n( n )
, m_r( Eigen::Map<const Matrix33sr>{ q.segment<9>( 3 * ( q.size() / 12 ) + 9 * m_idx_box ).data() } * getBodySpaceCorner( half_width, corner_num ) )
, m_idx_plane( plane_idx )
{
  assert( q.size() % 12 == 0 );
  assert( m_idx_box < q.size() / 12 );
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  assert( ( half_width.array() > 0.0 ).all() );
  assert( corner_num >= 0 ); assert( corner_num < 8 );
}

StaticPlaneBoxConstraint::~StaticPlaneBoxConstraint()
{}

scalar StaticPlaneBoxConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  return m_n.dot( computeRelativeVelocity( q, v ) );
}

//void StaticPlaneBoxConstraint::exertImpulse( const VectorXs& q, const scalar& lambda, const SparseMatrixsc& Minv, VectorXs& vout ) const
//{
//  assert( vout.size() % 6 == 0 ); assert( 3 * m_idx_box + 2 < vout.size() );
//  assert( Minv.rows() == Minv.cols() ); assert( Minv.nonZeros() == 2 * vout.size() );
//  assert( Minv.valuePtr()[ 3 * m_idx_box ] == Minv.valuePtr()[ 3 * m_idx_box + 1 ] );
//  assert( Minv.valuePtr()[ 3 * m_idx_box ] == Minv.valuePtr()[ 3 * m_idx_box + 2 ] );
//
//  const unsigned nbodies = vout.size() / 6;
//  
//  // Retrieve the inverse mass matrix for this body
//  const Eigen::Map<const Matrix33sr> Iinv = Eigen::Map<const Matrix33sr>( &Minv.valuePtr()[ 3 * nbodies + 9 * m_idx_box ] );
//  assert( ( Iinv - Iinv.transpose() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
//
//  vout.segment<3>( 3 * m_idx_box ) += Minv.valuePtr()[ 3 * m_idx_box ] * lambda * m_n;
//
//  const Vector3s ntilde = m_r.cross( m_n );
//  vout.segment<3>( 3 * ( m_idx_box + nbodies ) ) += lambda * Iinv * ntilde;
//}

void StaticPlaneBoxConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 );
  assert( col < G.cols() );
  assert( q.size() % 12 == 0 );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  G.insert( 3 * m_idx_box + 0, col ) = m_n.x();
  G.insert( 3 * m_idx_box + 1, col ) = m_n.y();
  G.insert( 3 * m_idx_box + 2, col ) = m_n.z();

  {
    const Vector3s ntilde{ m_r.cross( m_n ) };
    G.insert( 3 * ( m_idx_box + nbodies ) + 0, col ) = ntilde.x();
    G.insert( 3 * ( m_idx_box + nbodies ) + 1, col ) = ntilde.y();
    G.insert( 3 * ( m_idx_box + nbodies ) + 2, col ) = ntilde.z();
  }
}

void StaticPlaneBoxConstraint::computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const
{
  assert( start_column >= 0 );
  assert( start_column < D.cols() );
  assert( num_samples > 0 );
  assert( start_column + num_samples - 1 < D.cols() );
  assert( q.size() % 12 == 0 );
  assert( q.size() == 2 * v.size() );

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
  assert( unsigned( num_samples ) == friction_disk.size() );

  // For each sample of the friction disk
  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };
  for( unsigned friction_sample = 0; friction_sample < unsigned( num_samples ); ++friction_sample )
  {
    const unsigned cur_col{ start_column + friction_sample };
    assert( cur_col < unsigned( D.cols() ) );

    // Effect on center of mass
    assert( fabs( friction_disk[friction_sample].norm() - 1.0 ) <= 1.0e-6 );
    D.insert( 3 * m_idx_box + 0, cur_col ) = friction_disk[friction_sample].x();
    D.insert( 3 * m_idx_box + 1, cur_col ) = friction_disk[friction_sample].y();
    D.insert( 3 * m_idx_box + 2, cur_col ) = friction_disk[friction_sample].z();

    // Effect on orientation
    {
      const Vector3s ntilde{ m_r.cross( friction_disk[friction_sample] ) };
      D.insert( 3 * ( nbodies + m_idx_box ) + 0, cur_col ) = ntilde.x();
      D.insert( 3 * ( nbodies + m_idx_box ) + 1, cur_col ) = ntilde.y();
      D.insert( 3 * ( nbodies + m_idx_box ) + 2, cur_col ) = ntilde.z();
    }

    // Relative velocity contribution from kinematic scripting
    assert( cur_col < drel.size() );
    // Zero for now
    drel( cur_col ) = 0.0;
  }
}

void StaticPlaneBoxConstraint::computeSmoothGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, SparseMatrixsc& D ) const
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
    D.insert( 3 * m_idx_box + 0, cur_col ) = friction_disk[friction_sample].x();
    D.insert( 3 * m_idx_box + 1, cur_col ) = friction_disk[friction_sample].y();
    D.insert( 3 * m_idx_box + 2, cur_col ) = friction_disk[friction_sample].z();

    // Effect on orientation
    {
      const Vector3s ntilde{ m_r.cross( friction_disk[friction_sample] ) };
      D.insert( 3 * ( nbodies + m_idx_box ) + 0, cur_col ) = ntilde.x();
      D.insert( 3 * ( nbodies + m_idx_box ) + 1, cur_col ) = ntilde.y();
      D.insert( 3 * ( nbodies + m_idx_box ) + 2, cur_col ) = ntilde.z();
    }
  }
}

void StaticPlaneBoxConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( column < unsigned( D.cols() ) );
  assert( q.size() % 12 == 0 );
  assert( t.size() == 3 );
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  // Effect on center of mass of body i
  D.insert( 3 * m_idx_box + 0, column ) = t.x();
  D.insert( 3 * m_idx_box + 1, column ) = t.y();
  D.insert( 3 * m_idx_box + 2, column ) = t.z();
  // Effect on orientation of body i
  {
    const Vector3s ntilde0{ m_r.cross( Eigen::Map<const Vector3s>{ t.data() } ) };
    D.insert( 3 * ( m_idx_box + nbodies ) + 0, column ) = ntilde0.x();
    D.insert( 3 * ( m_idx_box + nbodies ) + 1, column ) = ntilde0.y();
    D.insert( 3 * ( m_idx_box + nbodies ) + 2, column ) = ntilde0.z();
  }
}

int StaticPlaneBoxConstraint::impactStencilSize() const
{
  return 6;
}

int StaticPlaneBoxConstraint::frictionStencilSize() const
{
  return 6;
}

void StaticPlaneBoxConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx_box;
  bodies.second = -1;
}

void StaticPlaneBoxConstraint::computeFrictionMask( const int nbodies, VectorXs& friction_mask ) const
{
  assert( 3 * m_idx_box + 2 < friction_mask.size() );
  friction_mask.segment<3>( 3 * m_idx_box ).setConstant( 1.0 );
  assert( 3 * ( m_idx_box + nbodies ) + 2 < friction_mask.size() );
  friction_mask.segment<3>( 3 * ( m_idx_box + nbodies ) ).setConstant( 1.0 );
}

void StaticPlaneBoxConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 );
  assert( strt_idx < gdotN.size() );

  // No kinematic scripting here, yet
  gdotN(strt_idx) = 0.0;
}

void StaticPlaneBoxConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
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

void StaticPlaneBoxConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  this->getSimulatedBodyIndices( bodies );
}

bool StaticPlaneBoxConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool StaticPlaneBoxConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool StaticPlaneBoxConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string StaticPlaneBoxConstraint::name() const
{
  return "static_plane_box";
}

void StaticPlaneBoxConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
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

// TODO: Update to handle kinematic planes
VectorXs StaticPlaneBoxConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( v.size() / 2 + 3 * m_idx_box + 2 < v.size() );

  const unsigned nbodies{ static_cast<unsigned>( v.size() / 6 ) };

  // v + omega x r
  return v.segment<3>( 3 * m_idx_box ) + v.segment<3>( 3 * ( nbodies + m_idx_box ) ).cross( m_r );
}

void StaticPlaneBoxConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx_box = idx;
}

VectorXs StaticPlaneBoxConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  // Zero, for now
  return VectorXs::Zero( 3 );
}

void StaticPlaneBoxConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<3>( 3 * m_idx_box ) + m_r;
}

void StaticPlaneBoxConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}

unsigned StaticPlaneBoxConstraint::getStaticObjectIndex() const
{
  return m_idx_plane;
}
