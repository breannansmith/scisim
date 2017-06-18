#include "KinematicObjectSphereConstraint.h"

#include "FrictionUtilities.h"
#ifndef NDEBUG
#include "scisim/Math/MathUtilities.h"
#endif

KinematicObjectSphereConstraint::KinematicObjectSphereConstraint( const unsigned sphere_idx, const scalar& r, const Vector3s& n, const unsigned kinematic_index, const Vector3s& X, const Vector3s& V, const Vector3s& omega )
: m_sphere_idx( sphere_idx )
, m_r( r )
, m_n( n )
, m_kinematic_index( kinematic_index )
, m_X( X )
, m_V( V )
, m_omega( omega )
{
  assert( m_r > 0.0 );
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
}

scalar KinematicObjectSphereConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 3 == 0 ); assert( 3 * m_sphere_idx + 2 < v.size() );
  return m_n.dot( v.segment<3>( 3 * m_sphere_idx ) - computeKinematicRelativeVelocity( q, v ) );
}

void KinematicObjectSphereConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( col >= 0 );
  assert( col < G.cols() );
  assert( 3 * m_sphere_idx + 2 < unsigned( G.rows() ) );

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  G.insert( 3 * m_sphere_idx + 0, col ) = m_n.x();
  G.insert( 3 * m_sphere_idx + 1, col ) = m_n.y();
  G.insert( 3 * m_sphere_idx + 2, col ) = m_n.z();
}

void KinematicObjectSphereConstraint::computeGeneralizedFrictionGivenTangentSample( const VectorXs& q, const VectorXs& t, const unsigned column, SparseMatrixsc& D ) const
{
  assert( column < unsigned( D.cols() ) );
  assert( q.size() % 12 == 0 );
  assert( t.size() == 3 );
  assert( fabs( t.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_n.dot( t ) ) <= 1.0e-6 );

  const unsigned nbodies{ static_cast<unsigned>( q.size() / 12 ) };

  // Effect on center of mass of body i
  D.insert( 3 * m_sphere_idx + 0, column ) = t.x();
  D.insert( 3 * m_sphere_idx + 1, column ) = t.y();
  D.insert( 3 * m_sphere_idx + 2, column ) = t.z();
  // Effect on orientation of body i
  {
    const Vector3s ri{ - m_r * m_n };
    assert( m_n.cross( ri ).norm() <= 1.0e-6 );
    const Vector3s ttilde{ ri.cross( Eigen::Map<const Vector3s>{ t.data() } ) };
    D.insert( 3 * ( m_sphere_idx + nbodies ) + 0, column ) = ttilde.x();
    D.insert( 3 * ( m_sphere_idx + nbodies ) + 1, column ) = ttilde.y();
    D.insert( 3 * ( m_sphere_idx + nbodies ) + 2, column ) = ttilde.z();
  }
}

int KinematicObjectSphereConstraint::impactStencilSize() const
{
  return 3;
}

int KinematicObjectSphereConstraint::frictionStencilSize() const
{
  return 6;
}

void KinematicObjectSphereConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_sphere_idx;
  bodies.second = -1;
}

void KinematicObjectSphereConstraint::getBodyIndices( std::pair<int,int>& bodies ) const
{
  bodies.first = m_sphere_idx;
  bodies.second = m_kinematic_index;
}

void KinematicObjectSphereConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 );
  assert( strt_idx < gdotN.size() );

  // TODO: Relax the zero velocity assumption
  assert( ( m_V.array() == 0.0 ).all() );
  assert( ( m_omega.array() == 0.0 ).all() );

  gdotN( strt_idx ) = 0.0;
}

void KinematicObjectSphereConstraint::evalH( const VectorXs& q, const MatrixXXsc& basis, MatrixXXsc& H0, MatrixXXsc& H1 ) const
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

bool KinematicObjectSphereConstraint::conservesTranslationalMomentum() const
{
  return false;
}

bool KinematicObjectSphereConstraint::conservesAngularMomentumUnderImpact() const
{
  return false;
}

bool KinematicObjectSphereConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return false;
}

std::string KinematicObjectSphereConstraint::name() const
{
  return "kinematic_object_sphere";
}

//Vector3s KinematicObjectSphereConstraint::computeKinematicCollisionPointVelocity( const VectorXs& q ) const
//{
//  std::cerr << "KinematicObjectSphereConstraint::computeKinematicCollisionPointVelocity" << std::endl;
//  std::exit( EXIT_FAILURE );
//
//  // Collision point relative to point kinematic object is rotating about
//  const Vector3s r_kinematic = q.segment<3>( 3 * m_kinematic_index ) - m_r * m_n - m_X;
//
//  return m_V + m_omega.cross( r_kinematic );
//}

//Vector3s KinematicObjectSphereConstraint::computeRelativeVelocityAtCollisionPoint( const VectorXs& q, const VectorXs& v ) const
//{
//  assert( v.size() % 6 == 0 ); assert( v.size() / 2 + 3 * m_i + 2 < v.size() );
//
//  const int nbodies = v.size() / 6;
//  
//  // Collision point relative to sphere's center of mass
//  const Vector3s r_sphere = - m_r * m_n;
//
//  // v_sphere + omega_sphere x r_sphere - v_kinematic - omega_kinematic x r_kinematic
//  return v.segment<3>( 3 * m_i ) + v.segment<3>( 3 * ( nbodies + m_i ) ).cross( r_sphere ) - computeKinematicCollisionPointVelocity( q );
//}

void KinematicObjectSphereConstraint::getWorldSpaceContactPoint( const VectorXs& q, VectorXs& contact_point ) const
{
  contact_point = q.segment<3>( 3 * m_sphere_idx ) - m_r * m_n;
}

void KinematicObjectSphereConstraint::getWorldSpaceContactNormal( const VectorXs& q, VectorXs& contact_normal ) const
{
  contact_normal = m_n;
}

void KinematicObjectSphereConstraint::computeContactBasis( const VectorXs& q, const VectorXs& v, MatrixXXsc& basis ) const
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

VectorXs KinematicObjectSphereConstraint::computeRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  assert( v.size() % 6 == 0 );
  assert( v.size() / 2 + 3 * m_sphere_idx + 2 < v.size() );

  const unsigned nbodies{ static_cast<unsigned>( v.size() / 6 ) };

  // Point of contact relative to each body's center of mass
  const Vector3s r{ - m_r * m_n };

  // v_0 + omega_0 x r_0 - ( v_1 + omega_1 x r_1 )
  return v.segment<3>( 3 * m_sphere_idx ) + v.segment<3>( 3 * ( nbodies + m_sphere_idx ) ).cross( r ) - computeKinematicRelativeVelocity( q, v );
}

void KinematicObjectSphereConstraint::setBodyIndex0( const unsigned idx )
{
  m_sphere_idx = idx;
}

VectorXs KinematicObjectSphereConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  // TODO: Relax the zero velocity assumption
  // No kinematic contribution
  return VectorXs::Zero( 3 );
}

unsigned KinematicObjectSphereConstraint::sphereIdx() const
{
  return m_sphere_idx;
}

unsigned KinematicObjectSphereConstraint::kinematicIdx() const
{
  return m_kinematic_index;
}

KinematicSphereSphereConstraint::KinematicSphereSphereConstraint( const unsigned sphere_idx, const scalar& r, const Vector3s& n, const unsigned kinematic_index, const Vector3s& X, const Vector3s& V, const Vector3s& omega, const scalar& r_kinematic )
: KinematicObjectSphereConstraint( sphere_idx, r, n, kinematic_index, X, V, omega )
, m_r_kinematic( r_kinematic )
{}

std::string KinematicSphereSphereConstraint::name() const
{
  return "kinematic_sphere_sphere";
}

scalar KinematicSphereSphereConstraint::computePenetrationDepth( const VectorXs& q ) const
{
  return std::min( 0.0, ( q.segment<3>( 3 * m_sphere_idx ) - q.segment<3>( 3 * m_kinematic_index ) ).norm() - m_r - m_r_kinematic );
}

scalar KinematicSphereSphereConstraint::evaluateGapFunction( const VectorXs& q ) const
{
  return ( q.segment<3>( 3 * m_sphere_idx ) - q.segment<3>( 3 * m_kinematic_index ) ).norm() - m_r - m_r_kinematic;
}
