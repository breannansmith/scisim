// StaticCylinderSphereConstraint.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "StaticCylinderSphereConstraint.h"

#include <iostream>

#include "FrictionUtilities.h"
#include "scisim/Math/MathUtilities.h"
#include "rigidbody3d/StaticGeometry/StaticCylinder.h"

bool StaticCylinderSphereConstraint::isActive( const Vector3s& center, const Vector3s& axis, const scalar& R, const Vector3s& x_sphere, const scalar& r )
{
  assert( fabs( axis.norm() - 1.0 ) <= 1.0e-6 );
  assert( R >= 0.0 );
  assert( r >= 0.0 );

  const Vector3s d{ x_sphere - center - axis.dot( x_sphere - center ) * axis };

  return d.squaredNorm() >= ( R - r ) * ( R - r );
}

StaticCylinderSphereConstraint::StaticCylinderSphereConstraint( const unsigned sphere_index, const scalar& r_sphere, const StaticCylinder& cyl, const unsigned cylinder_index )
: m_idx_sphere( sphere_index )
, m_r( r_sphere )
, m_cyl( cyl )
, m_cylinder_index( cylinder_index )
{
  assert( m_r >= 0.0 );
}

StaticCylinderSphereConstraint::~StaticCylinderSphereConstraint()
{}

scalar StaticCylinderSphereConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 );
  assert( 3 * m_idx_sphere + 2 < v.size() );
  return computeN( q ).dot( v.segment<3>( 3 * m_idx_sphere ) - computeCylinderCollisionPointVelocity( q ) );
}

void StaticCylinderSphereConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 );
  assert( col < G.cols() );
  assert( 3 * m_idx_sphere + 2 < unsigned( G.rows() ) );

  const Vector3s n{ computeN( q ) };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( n.dot( m_cyl.axis() ) ) <= 1.0e-6 );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.

  // Effect on center of mass
  G.insert( 3 * m_idx_sphere + 0, col ) = n.x();
  G.insert( 3 * m_idx_sphere + 1, col ) = n.y();
  G.insert( 3 * m_idx_sphere + 2, col ) = n.z();
}

void StaticCylinderSphereConstraint::computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& drel ) const
{
  assert( start_column >= 0 );
  assert( start_column < D.cols() );
  assert( num_samples > 0 );
  assert( start_column + num_samples - 1 < D.cols() );
  assert( q.size() % 12 == 0 );
  assert( q.size() == 2 * v.size() );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  const Vector3s n{ computeN( q ) };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( n.dot( m_cyl.axis() ) ) <= 1.0e-6 );

  std::vector<Vector3s> friction_disk( num_samples );
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
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-10 );
  assert( m_r >= 0.0 );
  const Vector3s r_world{ - m_r * n };

  // Cache the velocity of the collision point on the cylinder
  const Vector3s plane_collision_point_vel{ computeCylinderCollisionPointVelocity( q ) };

  // For each sample of the friction disk
  for( unsigned friction_sample = 0; friction_sample < unsigned( num_samples ); ++friction_sample )
  {
    const unsigned cur_col{ start_column + friction_sample };
    assert( cur_col < unsigned( D.cols() ) );

    // Effect on center of mass
    D.insert( 3 * m_idx_sphere + 0, cur_col ) = friction_disk[friction_sample].x();
    D.insert( 3 * m_idx_sphere + 1, cur_col ) = friction_disk[friction_sample].y();
    D.insert( 3 * m_idx_sphere + 2, cur_col ) = friction_disk[friction_sample].z();

    // Effect on orientation
    {
      const Vector3s ntilde{ r_world.cross( friction_disk[friction_sample] ) };
      D.insert( 3 * ( nbodies + m_idx_sphere ) + 0, cur_col ) = ntilde.x();
      D.insert( 3 * ( nbodies + m_idx_sphere ) + 1, cur_col ) = ntilde.y();
      D.insert( 3 * ( nbodies + m_idx_sphere ) + 2, cur_col ) = ntilde.z();
    }

    // Relative velocity contribution from kinematic scripting
    assert( cur_col < drel.size() );
    drel( cur_col ) = - friction_disk[friction_sample].dot( plane_collision_point_vel );
  }
}

void StaticCylinderSphereConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( column < unsigned( D.cols() ) );
  assert( q.size() % 12 == 0 );
  assert( t.size() == 3 );
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );

  const Vector3s n{ computeN( q ) };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( n.dot( t ) ) <= 1.0e-6 );

  // Compute the displacement from the center of mass to the point of contact
  assert( m_r >= 0.0 );
  const Vector3s r_world{ - m_r * n };

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  // Effect on center of mass of body i
  D.insert( 3 * m_idx_sphere + 0, column ) = t.x();
  D.insert( 3 * m_idx_sphere + 1, column ) = t.y();
  D.insert( 3 * m_idx_sphere + 2, column ) = t.z();
  // Effect on orientation of body i
  {
    const Vector3s ntilde{ r_world.cross( Eigen::Map<const Vector3s>{ t.data() } ) };
    D.insert( 3 * ( m_idx_sphere + nbodies ) + 0, column ) = ntilde.x();
    D.insert( 3 * ( m_idx_sphere + nbodies ) + 1, column ) = ntilde.y();
    D.insert( 3 * ( m_idx_sphere + nbodies ) + 2, column ) = ntilde.z();
  }
}

int StaticCylinderSphereConstraint::impactStencilSize() const
{
  return 3;
}

int StaticCylinderSphereConstraint::frictionStencilSize() const
{
  return 6;
}

void StaticCylinderSphereConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_idx_sphere;
  bodies.second = -1;
}

void StaticCylinderSphereConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  this->getSimulatedBodyIndices( bodies );
}

void StaticCylinderSphereConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 );
  assert( strt_idx < gdotN.size() );

  const Vector3s n{ computeN( q ) };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( n.dot( m_cyl.axis() ) ) <= 1.0e-6 );

  gdotN( strt_idx ) = - n.dot( computeCylinderCollisionPointVelocity( q ) );
}

void StaticCylinderSphereConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
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

  H0.block<1,3>( 0, 0 ) = n;
  H0.block<1,3>( 0, 3 ).setZero();

  H0.block<1,3>( 1, 0 ) = s;
  H0.block<1,3>( 1, 3 ) = r_world.cross( s );

  H0.block<1,3>( 2, 0 ) = t;
  H0.block<1,3>( 2, 3 ) = r_world.cross( t );
}

bool StaticCylinderSphereConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool StaticCylinderSphereConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool StaticCylinderSphereConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string StaticCylinderSphereConstraint::name() const
{
  return "static_cylinder_sphere";
}

Vector3s StaticCylinderSphereConstraint::computeN( const VectorXs& q ) const
{
  const Vector3s x_sphere{ q.segment<3>( 3 * m_idx_sphere ) };
  const Vector3s axis{ m_cyl.axis() };
  Vector3s n{ - ( x_sphere - m_cyl.x() - axis.dot( x_sphere - m_cyl.x() ) * axis ) };
  assert( n.norm() > 0.0 );
  return n.normalized();
}

Vector3s StaticCylinderSphereConstraint::computeCylinderCollisionPointVelocity( const VectorXs& q ) const
{
  // Vector perpendicular to axis in direction of particle 
  const Vector3s n{ -computeN( q ) };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( n.dot( m_cyl.axis() ) ) <= 1.0e-6 );

  // Closest point on axis to the particle
  const Vector3s p{ q.segment<3>( 3 * m_idx_sphere ) - m_cyl.x() - n.dot( q.segment<3>( 3 * m_idx_sphere ) - m_cyl.x() ) * n };
  assert( ( ( p - m_cyl.x() ).cross( m_cyl.axis() ) ).norm() <= 1.0e-6  );

  // Closest point on cylinder to the particle
  const Vector3s r{ p + m_cyl.r() * n };

  return m_cyl.v() + m_cyl.omega().cross( r );
}

void StaticCylinderSphereConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
{
  const Vector3s n{ computeN( q ) };

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

VectorXs StaticCylinderSphereConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( v.size() / 2 + 3 * m_idx_sphere + 2 < v.size() );

  const unsigned nbodies{ static_cast<unsigned>( v.size() / 6 ) };

  const Vector3s r_sphere{ - m_r * computeN( q ) };

  // v_point + omega_point x r_point - v_cylinder_collision_point
  return v.segment<3>( 3 * m_idx_sphere ) + v.segment<3>( 3 * ( nbodies + m_idx_sphere ) ).cross( r_sphere ) - computeCylinderCollisionPointVelocity( q );
}

void StaticCylinderSphereConstraint::setBodyIndex0( const unsigned idx )
{
  m_idx_sphere = idx;
}

// TODO: Test this code!!!!
scalar StaticCylinderSphereConstraint::computePenetrationDepth( const VectorXs& q ) const
{
  assert( 3 * m_idx_sphere + 2 < q.size() );
  assert( fabs( m_cyl.axis().norm() - 1.0 ) <= 1.0e-6 );
  const Vector3s d{ q.segment<3>( 3 * m_idx_sphere ) - m_cyl.x() - m_cyl.axis().dot( q.segment<3>( 3 * m_idx_sphere ) - m_cyl.x() ) * m_cyl.axis() };
  return std::min( 0.0, m_cyl.r() - d.norm() - m_r );
}

VectorXs StaticCylinderSphereConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  return computeCylinderCollisionPointVelocity( q );
}

void StaticCylinderSphereConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<3>( 3 * m_idx_sphere ) - m_r * computeN( q );
}

void StaticCylinderSphereConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = computeN( q );
}

unsigned StaticCylinderSphereConstraint::getStaticObjectIndex() const
{
  return m_cylinder_index;
}

unsigned StaticCylinderSphereConstraint::cylinderIdx() const
{
  return m_cylinder_index;
}

unsigned StaticCylinderSphereConstraint::sphereIdx() const
{
  return m_idx_sphere;
}
