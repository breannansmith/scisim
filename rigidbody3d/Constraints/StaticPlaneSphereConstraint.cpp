// StaticPlaneSphereConstraint.cpp
//
// Breannan Smith
// Last updated: 09/24/2015

#include "StaticPlaneSphereConstraint.h"

#include "FrictionUtilities.h"

#include "rigidbody3d/StaticGeometry/StaticPlane.h"
#include "scisim/Math/MathUtilities.h"

#include <iostream>

bool StaticPlaneSphereConstraint::isActive( const Vector3s& x_plane, const Vector3s& n_plane, const Vector3s& x_sphere, const scalar& r )
{
  assert( fabs( n_plane.norm() - 1.0 ) <= 1.0e-6 );
  return n_plane.dot( x_sphere - x_plane ) <= r;
}

StaticPlaneSphereConstraint::StaticPlaneSphereConstraint( const unsigned sphere_idx, const scalar& r, const StaticPlane& plane, const unsigned plane_idx )
: m_sphere_idx( sphere_idx )
, m_r( r )
, m_plane( plane )
, m_plane_idx( plane_idx )
{
  assert( m_r >= 0.0 );
  assert( fabs( m_plane.n().norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_plane.R().norm() - 1.0 ) <= 1.0e-6 );
}

StaticPlaneSphereConstraint::~StaticPlaneSphereConstraint()
{}

scalar StaticPlaneSphereConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 ); assert( 3 * m_sphere_idx + 2 < v.size() );
  return m_plane.n().dot( v.segment<3>( 3 * m_sphere_idx ) - computePlaneCollisionPointVelocity( q ) );
}

void StaticPlaneSphereConstraint::resolveImpact( const scalar& CoR, const SparseMatrixsc& M, const VectorXs& vin, const scalar& ndotv, VectorXs& vout, scalar& alpha ) const
{
  assert( CoR >= 0.0 );
  assert( CoR <= 1.0 );
  assert( ndotv < 0.0 );
  assert( vin.size() == vout.size() );
  assert( vin.size() % 3 == 0 );
  assert( M.rows() == M.cols() );
  assert( M.nonZeros() == 2 * vin.size() );
  assert( 3 * m_sphere_idx + 2 < vin.size() );

  const Eigen::Map<const VectorXs> m{ M.valuePtr(), vin.size() };
  assert( m( 3 * m_sphere_idx ) == m( 3 * m_sphere_idx + 1 ) );
  assert( m( 3 * m_sphere_idx ) == m( 3 * m_sphere_idx + 2 ) );

  const scalar msphere{ m( 3 * m_sphere_idx ) };

  // Compute the impulse
  alpha = - ( 1.0 + CoR ) * ndotv * msphere;
  assert( alpha >= 0.0 );
  vout.segment<3>( 3 * m_sphere_idx ) += - ( 1.0 + CoR ) * ndotv * m_plane.n();
}

void StaticPlaneSphereConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 );
  assert( col < G.cols() );
  assert( 3 * m_sphere_idx + 2 < unsigned( G.rows() ) );

  const Vector3s n{ m_plane.n() };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  G.insert( 3 * m_sphere_idx + 0, col ) = n.x();
  G.insert( 3 * m_sphere_idx + 1, col ) = n.y();
  G.insert( 3 * m_sphere_idx + 2, col ) = n.z();
}

// This method and the smooth version share the second half of code. Abstract that out.
void StaticPlaneSphereConstraint::computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const
{
  assert( start_column >= 0 );
  assert( start_column < D.cols() );
  assert( num_samples > 0 );
  assert( start_column + num_samples - 1 < D.cols() );
  assert( q.size() % 12 == 0 );
  assert( q.size() == 2 * v.size() );

  const Vector3s n{ m_plane.n() };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );

  std::vector<Vector3s> friction_disk( static_cast<std::vector<Vector3s>::size_type>( num_samples ) );
  assert( friction_disk.size() == std::vector<Vector3s>::size_type( num_samples ) );
  {
    // Compute the relative velocity
    Vector3s tangent_suggestion{ computeRelativeVelocity( q, v ) };
    if( tangent_suggestion.cross( n ).squaredNorm() < 1.0e-9 )
    {
      tangent_suggestion = FrictionUtilities::orthogonalVector( n );
    }
    tangent_suggestion *= -1.0;

    // Sample the friction disk
    FrictionUtilities::generateOrthogonalVectors( n, friction_disk, tangent_suggestion );
  }
  assert( unsigned( num_samples ) == friction_disk.size() );

  // Compute the displacement from the center of mass to the point of contact
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-10 ); assert( m_r >= 0.0 );
  const Vector3s r_world{ - m_r * n };

  // Cache the velocity of the collision point on the plane
  const Vector3s plane_collision_point_vel{ computePlaneCollisionPointVelocity( q ) };

  // For each sample of the friction disk
  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };
  for( unsigned friction_sample = 0; friction_sample < unsigned( num_samples ); ++friction_sample )
  {
    const unsigned cur_col{ start_column + friction_sample };
    assert( cur_col < unsigned( D.cols() ) );

    // Effect on center of mass
    D.insert( 3 * m_sphere_idx + 0, cur_col ) = friction_disk[friction_sample].x();
    D.insert( 3 * m_sphere_idx + 1, cur_col ) = friction_disk[friction_sample].y();
    D.insert( 3 * m_sphere_idx + 2, cur_col ) = friction_disk[friction_sample].z();

    // Effect on orientation
    {
      const Vector3s ntilde{ r_world.cross( friction_disk[friction_sample] ) };
      D.insert( 3 * ( nbodies + m_sphere_idx ) + 0, cur_col ) = ntilde.x();
      D.insert( 3 * ( nbodies + m_sphere_idx ) + 1, cur_col ) = ntilde.y();
      D.insert( 3 * ( nbodies + m_sphere_idx ) + 2, cur_col ) = ntilde.z();
    }

    // Relative velocity contribution from kinematic scripting
    assert( cur_col < drel.size() );
    drel( cur_col ) = - friction_disk[friction_sample].dot( plane_collision_point_vel );
  }
}

void StaticPlaneSphereConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( t.size() == 3 );
  assert( column < unsigned( D.cols() ) );
  assert( q.size() % 12 == 0 );
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_plane.n().dot( t ) ) <= 1.0e-6 );

  // Effect on center of mass
  D.insert( 3 * m_sphere_idx + 0, column ) = t.x();
  D.insert( 3 * m_sphere_idx + 1, column ) = t.y();
  D.insert( 3 * m_sphere_idx + 2, column ) = t.z();

  // Effect on orientation
  {
    const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };
    // Compute the displacement from the center of mass to the point of contact
    assert( fabs( m_plane.n().norm() - 1.0 ) <= 1.0e-10 );
    assert( m_r >= 0.0 );
    const Vector3s r_world{ - m_r * m_plane.n() };
    const Vector3s ntilde{ r_world.cross( Eigen::Map<const Vector3s>( t.data() ) ) };
    D.insert( 3 * ( nbodies + m_sphere_idx ) + 0, column ) = ntilde.x();
    D.insert( 3 * ( nbodies + m_sphere_idx ) + 1, column ) = ntilde.y();
    D.insert( 3 * ( nbodies + m_sphere_idx ) + 2, column ) = ntilde.z();
  }
}

int StaticPlaneSphereConstraint::impactStencilSize() const
{
  return 3;
}

int StaticPlaneSphereConstraint::frictionStencilSize() const
{
  return 6;
}

void StaticPlaneSphereConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_sphere_idx;
  bodies.second = -1;
}

void StaticPlaneSphereConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  this->getSimulatedBodyIndices( bodies );
}

void StaticPlaneSphereConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 );
  assert( strt_idx < gdotN.size() );
  gdotN( strt_idx ) = - m_plane.n().dot( computePlaneCollisionPointVelocity( q ) );
}

void StaticPlaneSphereConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
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

  // Compute the displacement from the center of mass to the point of contact
  assert( m_r >= 0.0 );
  const Vector3s r_world{ - m_r * n };

  H0.block<1,3>(0,0) = n;
  H0.block<1,3>(0,3).setZero();

  H0.block<1,3>(1,0) = s;
  H0.block<1,3>(1,3) = r_world.cross( s );

  H0.block<1,3>(2,0) = t;
  H0.block<1,3>(2,3) = r_world.cross( t );
}

bool StaticPlaneSphereConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool StaticPlaneSphereConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool StaticPlaneSphereConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string StaticPlaneSphereConstraint::name() const
{
  return "static_plane_sphere";
}

Vector3s StaticPlaneSphereConstraint::computePlaneCollisionPointVelocity( const VectorXs& q ) const
{
  const Vector3s n{ m_plane.n() };

  // Compute the collision point on the plane relative to x
  const Vector3s plane_point{ ( q.segment<3>( 3 * m_sphere_idx ) - m_plane.x() ) - n.dot( q.segment<3>( 3 * m_sphere_idx ) - m_plane.x() ) * n };

  return m_plane.v() + m_plane.omega().cross( plane_point );
}

void StaticPlaneSphereConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  const Vector3s n{ m_plane.n() };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );

  // Compute the relative velocity to use as a direction for the tangent sample
  Vector3s s{ computeRelativeVelocity( q, v ) };
  // If the relative velocity is zero, any vector will do
  if( n.cross( s ).squaredNorm() < 1.0e-9 )
  {
    s = FrictionUtilities::orthogonalVector( n );
  }
  // Otherwise project out the component along the normal and normalize the relative velocity
  else
  {
    s = ( s - s.dot( n ) * n ).normalized();
  }
  // Invert the tangent vector in order to oppose
  s *= -1.0;

  // Create a second orthogonal sample in the tangent plane
  const Vector3s t{ n.cross( s ).normalized() }; // Don't need to normalize but it won't hurt

  assert( MathUtilities::isRightHandedOrthoNormal( n, s, t, 1.0e-6 ) );
  basis.resize( 3, 3 );
  basis.col( 0 ) = n;
  basis.col( 1 ) = s;
  basis.col( 2 ) = t;
}

VectorXs StaticPlaneSphereConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( v.size() / 2 + 3 * m_sphere_idx + 2 < v.size() );

  const unsigned nbodies{ static_cast<unsigned>( v.size() / 6 ) };

  const Vector3s r_sphere{ - m_r * m_plane.n() };

  // v_point + omega_point x r_point - v_plane_collision_point
  return v.segment<3>( 3 * m_sphere_idx ) + v.segment<3>( 3 * ( nbodies + m_sphere_idx ) ).cross( r_sphere ) - computePlaneCollisionPointVelocity( q );
}

void StaticPlaneSphereConstraint::setBodyIndex0( const unsigned idx )
{
  m_sphere_idx = idx;
}

scalar StaticPlaneSphereConstraint::computePenetrationDepth( const VectorXs& q ) const
{
  assert( 3 * m_sphere_idx + 2 < q.size() );
  return std::min( 0.0, m_plane.n().dot( q.segment<3>( 3 * m_sphere_idx ) - m_plane.x() ) - m_r );
}

VectorXs StaticPlaneSphereConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  return computePlaneCollisionPointVelocity( q );
}

void StaticPlaneSphereConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<3>( 3 * m_sphere_idx ) - m_r * m_plane.n();
}

void StaticPlaneSphereConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_plane.n();
}

unsigned StaticPlaneSphereConstraint::getStaticObjectIndex() const
{
  return m_plane_idx;
}

const StaticPlane& StaticPlaneSphereConstraint::plane() const
{
  return m_plane;
}

unsigned StaticPlaneSphereConstraint::planeIdx() const
{
  return m_plane_idx;
}

unsigned StaticPlaneSphereConstraint::sphereIdx() const
{
  return m_sphere_idx;
}
