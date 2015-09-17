// StaticCylinderBodyConstraint.cpp
//
// Breannan Smith
// Last updated: 09/16/2015

#include "StaticCylinderBodyConstraint.h"

#include "rigidbody3d/StaticGeometry/StaticCylinder.h"

#include <iostream>

//#include "FrictionUtilities.h"

StaticCylinderBodyConstraint::StaticCylinderBodyConstraint( const unsigned body_index, const Vector3s& collision_point, const StaticCylinder& cyl, const unsigned cylinder_index, const VectorXs& q )
: m_body_index( body_index )
, m_r( collision_point - q.segment<3>( 3 * body_index ) )
, m_cyl( cyl )
//, m_cylinder_index( cylinder_index )
{}

StaticCylinderBodyConstraint::~StaticCylinderBodyConstraint()
{}

scalar StaticCylinderBodyConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  std::cerr << "StaticCylinderBodyConstraint::evalNdotV not implemented." << std::endl;
  std::exit( EXIT_FAILURE );
}

void StaticCylinderBodyConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( q.size() % 12 == 0 );
  const unsigned nbodies{ unsigned( q.size() ) / 12 };

  const Vector3s n{ computeN( q ) };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( n.dot( m_cyl.axis() ) ) <= 1.0e-6 );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.

  // Effect on center of mass position
  assert( col >= 0 );
  assert( col < G.cols() );
  assert( 3 * m_body_index + 2 < unsigned( G.rows() ) );
  G.insert( 3 * m_body_index + 0, col ) = n.x();
  G.insert( 3 * m_body_index + 1, col ) = n.y();
  G.insert( 3 * m_body_index + 2, col ) = n.z();

  // Effect on orientation
  const Vector3s ntilde{ m_r.cross( n ) };
  assert( 3 * ( nbodies + m_body_index ) + 2 < unsigned( G.rows() ) );
  G.insert( 3 * ( nbodies + m_body_index ) + 0, col ) = ntilde.x();
  G.insert( 3 * ( nbodies + m_body_index ) + 1, col ) = ntilde.y();
  G.insert( 3 * ( nbodies + m_body_index ) + 2, col ) = ntilde.z();
}

//void StaticCylinderSphereConstraint::computeGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, const int num_samples, SparseMatrixsc& D, VectorXs& gdotD )
//{
//  assert( start_column >= 0 ); assert( start_column < D.cols() );
//  assert( num_samples > 0 ); assert( start_column+num_samples - 1 < D.cols() );
//  assert( q.size() % 12 == 0 ); assert( q.size() == 2 * v.size() );
//
//  const int nbodies = q.size() / 12;
//
//  const Vector3s n = computeN( q );
//
//  // Compute the displacement from the center of mass to the point of contact
//  assert( fabs( n.norm() - 1.0 ) <= 1.0e-10 ); assert( m_r > 0.0 );
//  const Vector3s r_world = - m_r * n;
//
//  // Compute the relative velocity
//  Vector3s tangent_suggestion = computeRelativeVelocityAtCylinderPoint( q, v );
//  if( tangent_suggestion.cross( n ).squaredNorm() < 1.0e-9 ) tangent_suggestion = FrictionUtilities::orthogonalVector( n );
//  tangent_suggestion *= -1.0;
//
//  // Sample the friction disk
//  std::vector<Vector3s> friction_disk( num_samples );
//  FrictionUtilities::generateOrthogonalVectors( n, friction_disk, tangent_suggestion );
//  assert( num_samples == int( friction_disk.size() ) );
//
//  // For each sample of the friction disk
//  for( int i = 0; i < num_samples; ++i )
//  {
//    const int cur_col = start_column + i;
//    assert( cur_col >= 0 ); assert( cur_col < D.cols() );
//
//    // Effect on center of mass
//    D.insert( 3 * m_i + 0, cur_col ) = friction_disk[i].x();
//    D.insert( 3 * m_i + 1, cur_col ) = friction_disk[i].y();
//    D.insert( 3 * m_i + 2, cur_col ) = friction_disk[i].z();
//
//    // Effect on orientation
//    const Vector3s ntilde = r_world.cross( friction_disk[i] );
//    D.insert( 3 * ( nbodies + m_i ) + 0, cur_col ) = ntilde.x();
//    D.insert( 3 * ( nbodies + m_i ) + 1, cur_col ) = ntilde.y();
//    D.insert( 3 * ( nbodies + m_i ) + 2, cur_col ) = ntilde.z();
//
//    // Relative velocity contribution from kinematic scripting
//    assert( cur_col < gdotD.size() );
//    gdotD( cur_col ) = - friction_disk[i].dot( computeCylinderCollisionPointVelocity( q ) );
//  }
//}
//
//void StaticCylinderSphereConstraint::computeSmoothGeneralizedFrictionDisk( const VectorXs& q, const VectorXs& v, const int start_column, SparseMatrixsc& D, VectorXs& gdotD )
//{
//  assert( start_column >= 0 ); assert( start_column < D.cols() ); assert( start_column + 1 < D.cols() );
//  assert( q.size() % 12 == 0 ); assert( q.size() == 2 * v.size() );
//
//  const Vector3s n = computeN( q );
//  assert( fabs( n.norm() - 1.0 ) <= 1.0e-10 );
//
//  std::vector<Vector3s> friction_disk( 2 );
//
//  // Compute the relative velocity to use as a direction for the tangent sample
//  friction_disk[0] = computeRelativeVelocityAtCylinderPoint( q, v );
//  // If the relative velocity is zero, any vector will do
//  if( friction_disk[0].cross( n ).squaredNorm() < 1.0e-9 )
//  {
//    friction_disk[0] = FrictionUtilities::orthogonalVector( n );
//  }
//  // Otherwise project out the component along the normal and normalize the relative velocity
//  else
//  {
//    friction_disk[0] = friction_disk[0] - friction_disk[0].dot( n ) * n;
//    friction_disk[0].normalize();
//  }
//  // Invert the tangent vector in order to oppose
//  friction_disk[0] *= -1.0;
//  assert( fabs( friction_disk[0].norm() - 1.0 ) < 1.0e-6 );
//  assert( fabs( friction_disk[0].dot( n ) ) < 1.0e-6 );
//
//  // Create a second orthogonal sample in the tangent plane
//  friction_disk[1] = friction_disk[0].cross( n ).normalized(); // Don't need to normalize but it won't hurt
//  assert( fabs( friction_disk[1].norm() - 1.0 ) < 1.0e-6 );
//
//  // Compute the displacement from the center of mass to the point of contact
//  assert( m_r > 0.0 );
//  const Vector3s r_world = - m_r * n;
//
//  // For each sample of the friction disk
//  const int nbodies = q.size() / 12;
//  for( int i = 0; i < 2; ++i )
//  {
//    const int cur_col = start_column + i;
//    assert( cur_col >= 0 ); assert( cur_col < D.cols() );
//
//    // Effect on center of mass
//    D.insert( 3 * m_i + 0, cur_col ) = friction_disk[i].x();
//    D.insert( 3 * m_i + 1, cur_col ) = friction_disk[i].y();
//    D.insert( 3 * m_i + 2, cur_col ) = friction_disk[i].z();
//
//    // Effect on orientation
//    const Vector3s ntilde = r_world.cross( friction_disk[i] );
//    D.insert( 3 * ( nbodies + m_i ) + 0, cur_col ) = ntilde.x();
//    D.insert( 3 * ( nbodies + m_i ) + 1, cur_col ) = ntilde.y();
//    D.insert( 3 * ( nbodies + m_i ) + 2, cur_col ) = ntilde.z();
//
//    // Relative velocity contribution from kinematic scripting
//    assert( cur_col < gdotD.size() );
//    gdotD( cur_col ) = - friction_disk[i].dot( computeCylinderCollisionPointVelocity( q ) );
//  }
//}
//
int StaticCylinderBodyConstraint::impactStencilSize() const
{
  return 6;
}

int StaticCylinderBodyConstraint::frictionStencilSize() const
{
  return 6;
}

//void StaticCylinderSphereConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
//{
//  bodies.first = m_i;
//  bodies.second = -1;
//}
//
//void StaticCylinderSphereConstraint::computeFrictionMask( const int nbodies, VectorXs& friction_mask ) const
//{
//  std::cerr << "StaticCylinderSphereConstraint::computeFrictionMask not implemented." << std::endl;
//  std::exit( EXIT_FAILURE );
////  assert( 3 * m_i + 2 < friction_mask.size() );
////  assert( 3 * ( m_i + nbodies ) + 2 < friction_mask.size() );
////
////  friction_mask.segment<3>( 3 * m_i ).setConstant( 1.0 );
////  friction_mask.segment<3>( 3 * ( m_i + nbodies ) ).setConstant( 1.0 );
//}
//
//void StaticCylinderSphereConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
//{
//  this->getSimulatedBodyIndices( bodies );
//}

void StaticCylinderBodyConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 ); assert( strt_idx < gdotN.size() );

  const Vector3s n{ computeN( q ) };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( n.dot( m_cyl.axis() ) ) <= 1.0e-6 );

  gdotN( strt_idx ) = - n.dot( computeCylinderCollisionPointVelocity( q ) );
}

bool StaticCylinderBodyConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool StaticCylinderBodyConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool StaticCylinderBodyConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string StaticCylinderBodyConstraint::name() const
{
  return "static_cylinder_body";
}

Vector3s StaticCylinderBodyConstraint::computeN( const VectorXs& q ) const
{
  const Vector3s x_body{ q.segment<3>( 3 * m_body_index ) };
  const Vector3s axis{ m_cyl.axis() };
  Vector3s n{ - ( x_body - m_cyl.x() - axis.dot( x_body - m_cyl.x() ) * axis ) };
  assert( n.norm() > 0.0 );
  return n / n.norm();
}

Vector3s StaticCylinderBodyConstraint::computeCylinderCollisionPointVelocity( const VectorXs& q ) const
{
  // Vector perpendicular to axis in direction of particle 
  const Vector3s n{ -computeN( q ) };
  assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( n.dot( m_cyl.axis() ) ) <= 1.0e-6 );

  // Closest point on axis to the particle
  const Vector3s p{ q.segment<3>( 3 * m_body_index ) - m_cyl.x() - n.dot( q.segment<3>( 3 * m_body_index ) - m_cyl.x() ) * n };
  assert( ( ( p - m_cyl.x() ).cross( m_cyl.axis() ) ).norm() <= 1.0e-6  );

  // Closest point on cylinder to the particle
  const Vector3s r{ p + m_cyl.r() * n };

  return m_cyl.v() + m_cyl.omega().cross( r );
}

VectorXs StaticCylinderBodyConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  std::cerr << "Code up StaticCylinderBodyConstraint::computeKinematicRelativeVelocity" << std::endl;
  std::exit( EXIT_FAILURE );
}

//Vector3s StaticCylinderSphereConstraint::computeRelativeVelocityAtCylinderPoint( const VectorXs& q, const VectorXs& v ) const
//{
//  assert( v.size() % 6 == 0 ); assert( v.size() / 2 + 3 * m_i + 2 < v.size() );
//
//  const int nbodies = v.size() / 6;
//  
//  const Vector3s r_sphere = - m_r * computeN( q );
//
//  // v + omega x r
//  return v.segment<3>( 3 * m_i ) + v.segment<3>( 3 * ( nbodies + m_i ) ).cross( r_sphere ) - computeCylinderCollisionPointVelocity( q );
//}

//void StaticCylinderSphereConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point )
//{
//  contact_point = q.segment<3>( 3 * m_i ) - m_r * computeN( q );
//}

//void StaticCylinderSphereConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal )
//{
//  contact_normal = computeN( q );
//}

//unsigned StaticCylinderSphereConstraint::getStaticObjectIndex() const
//{
//  return m_cylinder_index;
//}
