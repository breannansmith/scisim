// RigidBody3DSim.cpp
//
// Breannan Smith
// Last updated: 10/01/2015

#include "RigidBody3DSim.h"

#include <iostream>

#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"
#include "scisim/ScriptingCallback.h"
#include "scisim/Utilities.h"
#include "scisim/HDF5File.h"
#include "scisim/Math/Rational.h"
#include "Forces/Force.h"
#include "Geometry/RigidBodyBox.h"
#include "Geometry/RigidBodySphere.h"
#include "Geometry/RigidBodyStaple.h"
#include "Geometry/RigidBodyTriangleMesh.h"
#include "Constraints/BoxBoxUtilities.h"
#include "Constraints/StapleStapleUtilities.h"
#include "Constraints/MeshMeshUtilities.h"
#include "Constraints/SphereSphereConstraint.h"
#include "Constraints/TeleportedSphereSphereConstraint.h"
#include "Constraints/SphereBodyConstraint.h"
#include "Constraints/BodyBodyConstraint.h"
#include "Constraints/StaticPlaneBodyConstraint.h"
#include "Constraints/StaticPlaneBoxConstraint.h"
#include "Constraints/StaticPlaneSphereConstraint.h"
#include "Constraints/StaticCylinderSphereConstraint.h"
#include "Constraints/StaticCylinderBodyConstraint.h"
#include "Constraints/KinematicObjectSphereConstraint.h"
#include "Constraints/KinematicObjectBodyConstraint.h"
#include "Constraints/CollisionUtilities.h"
#include "StaticGeometry/StaticCylinder.h"
#include "SpatialGridDetector.h"
#include "Portals/PlanarPortal.h"
#include "StateOutput.h"
#include "PythonScripting.h"

RigidBody3DSim::RigidBody3DSim()
: m_sim_state()
, m_impact_map( false )
{}

void RigidBody3DSim::setState( const RigidBody3DState& state )
{
  m_sim_state = state;
}

const RigidBody3DState& RigidBody3DSim::getState() const
{
  return m_sim_state;
}

RigidBody3DState& RigidBody3DSim::getState()
{
  return m_sim_state;
}

void RigidBody3DSim::computeBoundingBox( scalar& minx, scalar& maxx, scalar& miny, scalar& maxy, scalar& minz, scalar& maxz ) const
{
  // For each body
  Array3s min{ Array3s::Constant( SCALAR_INFINITY ) };
  Array3s max{ Array3s::Constant( -SCALAR_INFINITY ) };
  for( unsigned i = 0; i < m_sim_state.nbodies(); ++i )
  {
    const Vector3s cm{ m_sim_state.q().segment<3>( 3 * i ) };
    const Matrix33sr R{ Eigen::Map<const Matrix33sr>( m_sim_state.q().segment<9>( 3 * m_sim_state.nbodies() + 9 * i ).data() ) };
    Array3s body_min;
    Array3s body_max;
    m_sim_state.getGeometryOfBody( i ).computeAABB( cm, R, body_min, body_max );
    min = min.min( body_min );
    max = max.max( body_max );
  }
  assert( ( min < max ).all() );

  minx = min.x();
  miny = min.y();
  minz = min.z();
  maxx = max.x();
  maxy = max.y();
  maxz = max.z();
}

void RigidBody3DSim::computeBoundingSphere( scalar& radius, Vector3s& center ) const
{
  scalar minx;
  scalar maxx;
  scalar miny;
  scalar maxy;
  scalar minz;
  scalar maxz;
  computeBoundingBox( minx, maxx, miny, maxy, minz, maxz );
  center << 0.5 * ( minx + maxx ), 0.5 * ( miny + maxy ), 0.5 * ( minz + maxz );
  radius = ( center - Vector3s{ maxx, maxy, maxz } ).norm();
}

scalar RigidBody3DSim::computeKineticEnergy() const
{
  return 0.5 * m_sim_state.v().dot( m_sim_state.M() * m_sim_state.v() );
}

scalar RigidBody3DSim::computePotentialEnergy()
{
  scalar U = 0.0;
  for( std::vector<std::unique_ptr<Force>>::size_type frc_idx = 0; frc_idx < m_sim_state.forces().size(); ++frc_idx )
  {
    U += m_sim_state.forces()[frc_idx]->computePotential( m_sim_state.q(), m_sim_state.M() );
  }
  return U;
}

scalar RigidBody3DSim::computeTotalEnergy()
{
  return computeKineticEnergy() + computePotentialEnergy();
}

Vector3s RigidBody3DSim::computeTotalMomentum() const
{
  Vector3s p{ Vector3s::Zero() };

  for( unsigned bdy_idx = 0; bdy_idx < m_sim_state.nbodies(); ++bdy_idx )
  {
    p += m_sim_state.getTotalMass( bdy_idx ) * m_sim_state.v().segment<3>( 3 * bdy_idx );
  }

  return p;
}

Vector3s RigidBody3DSim::computeTotalAngularMomentum() const
{
  Vector3s L{ Vector3s::Zero() };

  // Contribution from center of mass
  for( unsigned bdy_idx = 0; bdy_idx < m_sim_state.nbodies(); ++bdy_idx )
  {
    L += m_sim_state.getTotalMass( bdy_idx ) * m_sim_state.q().segment<3>( 3 * bdy_idx ).cross( m_sim_state.v().segment<3>( 3 * bdy_idx ) );
  }

  // Contribution from rotation about center of mass
  for( unsigned bdy_idx = 0; bdy_idx < m_sim_state.nbodies(); ++bdy_idx )
  {
    L += m_sim_state.getInertia( bdy_idx ) * m_sim_state.v().segment<3>( 3 * m_sim_state.nbodies() + 3 * bdy_idx );
  }

  return L;
}

bool RigidBody3DSim::empty() const
{
  return nqdofs() == 0;
}

int RigidBody3DSim::nqdofs() const
{
  return m_sim_state.q().size();
}

int RigidBody3DSim::nvdofs() const
{
  return m_sim_state.v().size();
}

unsigned RigidBody3DSim::numVelDoFsPerBody() const
{
  return 6;
}

unsigned RigidBody3DSim::ambientSpaceDimensions() const
{
  return 3;
}

bool RigidBody3DSim::isKinematicallyScripted( const int i ) const
{
  assert( i >= 0 ); assert( nvdofs() % 3 == 0 ); assert( i < nvdofs() / 3 );
  return m_sim_state.isKinematicallyScripted( i );
}

void RigidBody3DSim::computeForce( const VectorXs& q, const VectorXs& v, const scalar& t, VectorXs& F )
{
  assert( q.size() == 12 * m_sim_state.nbodies() );
  assert( v.size() == 6 * m_sim_state.nbodies() );
  assert( v.size() == F.size() );

  F.setZero();

  for( std::vector<std::unique_ptr<Force>>::size_type frc_idx = 0; frc_idx < m_sim_state.forces().size(); ++frc_idx )
  {
    m_sim_state.forces()[frc_idx]->computeForce( m_sim_state.q(), m_sim_state.v(), m_sim_state.M(), F );
  }
}

const SparseMatrixsc& RigidBody3DSim::M() const
{
  return m_sim_state.M();
}

const SparseMatrixsc& RigidBody3DSim::Minv() const
{
  return m_sim_state.Minv();
}

const SparseMatrixsc& RigidBody3DSim::M0() const
{
  return m_sim_state.M0();
}

const SparseMatrixsc& RigidBody3DSim::Minv0() const
{
  return m_sim_state.Minv0();
}

void RigidBody3DSim::computeMomentum( const VectorXs& v, VectorXs& p ) const
{
  p = Vector3s::Zero();
  for( unsigned bdy_idx = 0; bdy_idx < m_sim_state.nbodies(); ++bdy_idx )
  {
    p += m_sim_state.getTotalMass( bdy_idx ) * v.segment<3>( 3 * bdy_idx );
  }
}

void RigidBody3DSim::computeAngularMomentum( const VectorXs& v, VectorXs& L ) const
{
  L = Vector3s::Zero();
  // Contribution from center of mass
  for( unsigned bdy_idx = 0; bdy_idx < m_sim_state.nbodies(); ++bdy_idx )
  {
    L += m_sim_state.getTotalMass( bdy_idx ) * m_sim_state.q().segment<3>( 3 * bdy_idx ).cross( v.segment<3>( 3 * bdy_idx ) );
  }
  // Contribution from rotation about center of mass
  for( unsigned bdy_idx = 0; bdy_idx < m_sim_state.nbodies(); ++bdy_idx )
  {
    L += m_sim_state.getInertia( bdy_idx ) * v.segment<3>( 3 * m_sim_state.nbodies() + 3 * bdy_idx );
  }
}

void RigidBody3DSim::computeActiveSet( const VectorXs& q0, const VectorXs& qp, std::vector<std::unique_ptr<Constraint>>& active_set )
{
  assert( q0.size() == qp.size() );
  assert( active_set.empty() );

  // Detect body-body collisions
  computeActiveSetBodyBodySpatialGrid( q0, qp, active_set );

  // Detect body-plane collisions
  computeBodyPlaneActiveSetAllPairs( q0, qp, active_set );
  // Detect body-cylinder collisions
  computeBodyCylinderActiveSetAllPairs( q0, qp, active_set );
}

void RigidBody3DSim::computeImpactBases( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& active_set, MatrixXXsc& impact_bases ) const
{
  const unsigned ncols{ static_cast<unsigned>( active_set.size() ) };
  impact_bases.resize( 3, ncols );
  for( unsigned col_num = 0; col_num < ncols; ++col_num )
  {
    VectorXs current_normal;
    active_set[col_num]->getWorldSpaceContactNormal( q, current_normal );
    assert( fabs( current_normal.norm() - 1.0 ) <= 1.0e-6 );
    impact_bases.col( col_num ) = current_normal;
  }
}

void RigidBody3DSim::computeContactBases( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& active_set, MatrixXXsc& contact_bases ) const
{
  const unsigned ncols{ static_cast<unsigned>( active_set.size() ) };
  contact_bases.resize( 3, 3 * ncols );
  for( unsigned col_num = 0; col_num < ncols; ++col_num )
  {
    MatrixXXsc basis;
    active_set[col_num]->computeBasis( q, v, basis );
    assert( basis.rows() == basis.cols() ); assert( basis.rows() == 3 );
    assert( ( basis * basis.transpose() - MatrixXXsc::Identity( 3, 3 ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-6 );
    contact_bases.block<3,3>( 0, 3 * col_num ) = basis;
  }
}

void RigidBody3DSim::clearConstraintCache()
{
  m_constraint_cache.clear();
}

void RigidBody3DSim::cacheConstraint( const Constraint& constraint, const VectorXs& r )
{
  m_constraint_cache.cacheConstraint( constraint, r );
}

void RigidBody3DSim::getCachedConstraintImpulse( const Constraint& constraint, VectorXs& r ) const
{
  m_constraint_cache.getCachedConstraint( constraint, r );
}

void RigidBody3DSim::computeNumberOfCollisions( std::map<std::string,unsigned>& collision_counts, std::map<std::string,scalar>& collision_depths, std::map<std::string,scalar>& overlap_volumes )
{
  collision_counts.clear();
  collision_depths.clear();
  overlap_volumes.clear();
  std::vector<std::unique_ptr<Constraint>> active_set;
  computeActiveSet( m_sim_state.q(), m_sim_state.q(), active_set );
  for( const std::unique_ptr<Constraint>& constraint : active_set )
  {
    const std::string constraint_name{ constraint->name() };

    // Update the overlap volume given this constraint
    {
      std::map<std::string,unsigned>::iterator count_iterator{ collision_counts.find( constraint_name ) };
      if( count_iterator == collision_counts.end() )
      {
        const std::pair<std::map<std::string,unsigned>::iterator,bool> count_return{ collision_counts.insert( std::make_pair( constraint_name, 0 ) ) };
        assert( count_return.second );
        assert( count_return.first != collision_counts.end() );
        count_iterator = count_return.first;
      }
      ++count_iterator->second;
    }

    // Update the overlap volume given this constraint
    {
      std::map<std::string,scalar>::iterator depth_iterator{ collision_depths.find( constraint_name ) };
      if( depth_iterator == collision_depths.end() )
      {
        const std::pair<std::map<std::string,scalar>::iterator,bool> depths_return{ collision_depths.insert( std::make_pair( constraint_name, 0 ) ) };
        assert( depths_return.second );
        assert( depths_return.first != collision_depths.end() );
        depth_iterator = depths_return.first;
      }
      depth_iterator->second += constraint->penetrationDepth( m_sim_state.q() );
    }

    // Update the overlap volume given this constraint
    {
      std::map<std::string,scalar>::iterator volume_iterator{ overlap_volumes.find( constraint_name ) };
      if( volume_iterator == overlap_volumes.end() )
      {
        const std::pair<std::map<std::string,scalar>::iterator,bool> volumes_return{ overlap_volumes.insert( std::make_pair( constraint_name, 0 ) ) };
        assert( volumes_return.second );
        assert( volumes_return.first != overlap_volumes.end() );
        volume_iterator = volumes_return.first;
      }
      volume_iterator->second += constraint->overlapVolume( m_sim_state.q() );
    }
  }
}

void RigidBody3DSim::flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap )
{
  call_back.setState( m_sim_state );
  call_back.startOfStepCallback( iteration, dt );
  call_back.forgetState();

  // Ensure that fixed bodies do not move
  #ifndef NDEBUG
  VectorXs fixed_q{ VectorXs::Zero( m_sim_state.q().size() ) };
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      fixed_q.segment<3>( 3 * body_index ) = m_sim_state.q().segment<3>( 3 * body_index );
      fixed_q.segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index ) = m_sim_state.q().segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index );
    }
  }
  VectorXs fixed_v{ VectorXs::Zero( m_sim_state.v().size() ) };
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      fixed_v.segment<3>( 3 * body_index ) = m_sim_state.v().segment<3>( 3 * body_index );
      fixed_v.segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index ) = m_sim_state.v().segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index );
    }
  }
  #endif

  VectorXs q1{ m_sim_state.q().size() };
  VectorXs v1{ m_sim_state.v().size() };

  umap.flow( m_sim_state.q(), m_sim_state.v(), *this, iteration, scalar( dt ), q1, v1 );

  q1.swap( m_sim_state.q() );
  v1.swap( m_sim_state.v() );
  m_sim_state.updateMandMinv();

  #ifndef NDEBUG
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      assert( ( fixed_q.segment<3>( 3 * body_index ).array() == m_sim_state.q().segment<3>( 3 * body_index ).array() ).all() );
      assert( ( fixed_q.segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index ).array() == m_sim_state.q().segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index ).array() ).all() );
    }
  }
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      assert( ( fixed_v.segment<3>( 3 * body_index ).array() == m_sim_state.v().segment<3>( 3 * body_index ).array() ).all() );
      assert( ( fixed_v.segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index ).array() == m_sim_state.v().segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index ).array() ).all() );
    }
  }
  #endif

  enforcePeriodicBoundaryConditions();

  call_back.setState( m_sim_state );
  call_back.endOfStepCallback( iteration, dt );
  call_back.forgetState();
}

void RigidBody3DSim::flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap, ImpactOperator& imap, const scalar& CoR )
{
  call_back.setState( m_sim_state );
  call_back.startOfStepCallback( iteration, dt );
  call_back.forgetState();

  // Ensure that fixed bodies do not move
  #ifndef NDEBUG
  VectorXs fixed_q{ VectorXs::Zero( m_sim_state.q().size() ) };
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      fixed_q.segment<3>( 3 * body_index ) = m_sim_state.q().segment<3>( 3 * body_index );
      fixed_q.segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index ) = m_sim_state.q().segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index );
    }
  }
  VectorXs fixed_v{ VectorXs::Zero( m_sim_state.v().size() ) };
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      fixed_v.segment<3>( 3 * body_index ) = m_sim_state.v().segment<3>( 3 * body_index );
      fixed_v.segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index ) = m_sim_state.v().segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index );
    }
  }
  #endif

  VectorXs q1{ m_sim_state.q().size() };
  VectorXs v1{ m_sim_state.v().size() };

  m_impact_map.flow( call_back, *this, *this, umap, imap, iteration, scalar( dt ), CoR, m_sim_state.q(), m_sim_state.v(), q1, v1 );

  q1.swap( m_sim_state.q() );
  v1.swap( m_sim_state.v() );
  m_sim_state.updateMandMinv();

  #ifndef NDEBUG
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      assert( ( fixed_q.segment<3>( 3 * body_index ).array() == m_sim_state.q().segment<3>( 3 * body_index ).array() ).all() );
      assert( ( fixed_q.segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index ).array() == m_sim_state.q().segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index ).array() ).all() );
    }
  }
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      assert( ( fixed_v.segment<3>( 3 * body_index ).array() == m_sim_state.v().segment<3>( 3 * body_index ).array() ).all() );
      assert( ( fixed_v.segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index ).array() == m_sim_state.v().segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index ).array() ).all() );
    }
  }
  #endif

  enforcePeriodicBoundaryConditions();

  call_back.setState( m_sim_state );
  call_back.endOfStepCallback( iteration, dt );
  call_back.forgetState();
}

void RigidBody3DSim::flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap, const scalar& CoR, const scalar& mu, FrictionSolver& solver, ImpactFrictionMap& ifmap )
{
  call_back.setState( m_sim_state );
  call_back.startOfStepCallback( iteration, dt );
  call_back.forgetState();

  // Ensure that fixed bodies do not move
  #ifndef NDEBUG
  VectorXs fixed_q{ VectorXs::Zero( m_sim_state.q().size() ) };
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      fixed_q.segment<3>( 3 * body_index ) = m_sim_state.q().segment<3>( 3 * body_index );
      fixed_q.segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index ) = m_sim_state.q().segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index );
    }
  }
  VectorXs fixed_v{ VectorXs::Zero( m_sim_state.v().size() ) };
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      fixed_v.segment<3>( 3 * body_index ) = m_sim_state.v().segment<3>( 3 * body_index );
      fixed_v.segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index ) = m_sim_state.v().segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index );
    }
  }
  #endif

  VectorXs q1{ m_sim_state.q().size() };
  VectorXs v1{ m_sim_state.v().size() };

  ifmap.flow( call_back, *this, *this, umap, solver, iteration, scalar( dt ), CoR, mu, m_sim_state.q(), m_sim_state.v(), q1, v1 );

  q1.swap( m_sim_state.q() );
  v1.swap( m_sim_state.v() );
  m_sim_state.updateMandMinv();

  #ifndef NDEBUG
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      assert( ( fixed_q.segment<3>( 3 * body_index ).array() == m_sim_state.q().segment<3>( 3 * body_index ).array() ).all() );
      assert( ( fixed_q.segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index ).array() == m_sim_state.q().segment<9>( 3 * m_sim_state.nbodies() + 9 * body_index ).array() ).all() );
    }
  }
  for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
  {
    if( m_sim_state.isKinematicallyScripted( body_index ) )
    {
      assert( ( fixed_v.segment<3>( 3 * body_index ).array() == m_sim_state.v().segment<3>( 3 * body_index ).array() ).all() );
      assert( ( fixed_v.segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index ).array() == m_sim_state.v().segment<3>( 3 * m_sim_state.nbodies() + 3 * body_index ).array() ).all() );
    }
  }
  #endif

  enforcePeriodicBoundaryConditions();

  call_back.setState( m_sim_state );
  call_back.endOfStepCallback( iteration, dt );
  call_back.forgetState();
}

const RigidBody3DState& RigidBody3DSim::state() const
{
  return m_sim_state;
}

RigidBody3DState& RigidBody3DSim::state()
{
  return m_sim_state;
}

void RigidBody3DSim::enforcePeriodicBoundaryConditions()
{
  assert( m_sim_state.q().size() % 12 == 0 ); assert( m_sim_state.nbodies() == m_sim_state.q().size() / 12 );

  const unsigned nbodies{ m_sim_state.nbodies() };

  // For each portal
  for( std::vector<PlanarPortal>::size_type prtl_idx = 0; prtl_idx < m_sim_state.numPlanarPortals(); ++prtl_idx )
  {
    // For each body
    for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
    {
      Vector3s cm{ m_sim_state.q().segment<3>( 3 * bdy_idx ) };
      // TODO: Calling pointInsidePortal and teleportPointInsidePortal is a bit redundant, clean this up!
      // If the body is inside a portal
      if( m_sim_state.planarPortal( prtl_idx ).pointInsidePortal( cm ) )
      {
        // Teleport to the other side of the portal
        m_sim_state.planarPortal( prtl_idx ).teleportPointInsidePortal( cm, cm );
        m_sim_state.q().segment<3>( 3 * bdy_idx ) = cm;
      }
    }
  }
}

void RigidBody3DSim::boxBoxNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const RigidBodyBox& box0, const RigidBodyBox& box1, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  assert( !isKinematicallyScripted( first_body ) ); // Kinematic rigid body should be listed second

  const Matrix33sr R0{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * first_body ).data() } };
  const Matrix33sr R1{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * second_body ).data() } };
  // TODO: Assert that R0 and R1 are orthonormal and orientation preserving

  Vector3s n;
  std::vector<Vector3s> points;
  BoxBoxUtilities::isActive( q1.segment<3>( 3 * first_body ), R0, box0.halfWidths(),
                             q1.segment<3>( 3 * second_body ), R1, box1.halfWidths(),
                             n, points );

  if( !isKinematicallyScripted( second_body ) )
  {
    for( const Vector3s& point : points )
    {
      active_set.emplace_back( new BodyBodyConstraint{ first_body, second_body, point, n, q0 } );
    }
  }
  else
  {
    for( const Vector3s& point : points )
    {
      active_set.emplace_back( new KinematicObjectBodyConstraint{ first_body, second_body, point, n, q0 } );
    }
  }
}

void RigidBody3DSim::boxSphereNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const RigidBodyBox& box, const RigidBodySphere& sphere, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  // TODO: Kinematic spheres in sphere-box not coded up yet! (easy though, just do it)
  assert( !isKinematicallyScripted( second_body ) );

  // Only need the orientation of the box, sphere's geometry is rotationally invariant about center of mass
  const Matrix33sc Rb{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * first_body ).data() } };

  // Check for collisions
  std::vector<Vector3s> points;
  std::vector<Vector3s> normals;
  CollisionUtilities::computeBoxSphereActiveSet( q1.segment<3>( 3 * first_body ), Rb, box.halfWidths(), q1.segment<3>( 3 * second_body ), sphere.r(), points, normals );
  assert( points.size() == normals.size() );

  std::cerr << "Error, bring box-sphere back up" << std::endl;
  std::exit( EXIT_FAILURE );

//  // Create a constraint from each of the detected collisions
//  for( std::vector<Vector3s>::size_type i = 0; i < points.size(); ++i )
//  {
//    if( !isKinematicallyScripted( first_body ) )
//    {
//      std::cerr << "Box-sphere is buggy, fix it!" << std::endl;
//      std::exit( EXIT_FAILURE );
//      //active_set.push_back( std::unique_ptr<Constraint>( new SphereBodyConstraint( second_body, first_body, points[i], normals[i], q0 ) ) );
//    }
//    else
//    {
//      std::cerr << "Code up kinematic box vs. sphere constraint" << std::endl;
//      std::exit( EXIT_FAILURE );
////      // Box shouldn't be moving if it is kinematic, for now
////      #ifndef NDEBUG
////        assert( q0.size() % 12 == 0 );
////        const int nbodies = q0.size() / 12;
////        assert( ( q0.segment<3>( 3 * first_body ).array() == q1.segment<3>( 3 * first_body ).array() ).all() );
////        assert( ( q0.segment<9>( 3 * nbodies + 9 * first_body ).array() == q1.segment<9>( 3 * nbodies + 9 * first_body ).array() ).all() );
////      #endif
////      // TODO: Total mess! Clean up!
////      // -x: 0 +x: 1 -y: 2 +y: 3 -z: 4 +z: 5
////      unsigned face_idx;
////      {
////        assert( fabs( ( Rb.transpose() * normals[i] ).norm() - 1.0 ) <= 1.0e-6 );
////        // Get the index of the max element of the array
////        const Vector3s body_space_normal = -Rb.transpose() * normals[i];
////        std::cout << "body_space_normal: " << body_space_normal.transpose() << std::endl;
////        int max_idx = -1;
////        body_space_normal.array().abs().matrix().maxCoeff( &max_idx );
////        std::cout << "max idx: " << max_idx << std::endl;
////        assert( max_idx == 0 || max_idx == 1 || max_idx == 2 );
////        if( max_idx == 0 )
////        {
////          if( body_space_normal[max_idx] < 0 )
////          {
////            assert( fabs( body_space_normal[max_idx] + 1.0 ) <= 1.0e-6 );
////            face_idx = 0;
////          }
////          else
////          {
////            assert( fabs( body_space_normal[max_idx] - 1.0 ) <= 1.0e-6 );
////            face_idx = 1;
////          }
////        }
////        else if( max_idx == 1 )
////        {
////          if( body_space_normal[max_idx] < 0 )
////          {
////            assert( fabs( body_space_normal[max_idx] + 1.0 ) <= 1.0e-6 );
////            face_idx = 2;
////          }
////          else
////          {
////            assert( fabs( body_space_normal[max_idx] - 1.0 ) <= 1.0e-6 );
////            face_idx = 3;
////          }
////        }
////        else
////        {
////          if( body_space_normal[max_idx] < 0 )
////          {
////            assert( fabs( body_space_normal[max_idx] + 1.0 ) <= 1.0e-6 );
////            face_idx = 4;
////          }
////          else
////          {
////            assert( fabs( body_space_normal[max_idx] - 1.0 ) <= 1.0e-6 );
////            face_idx = 5;
////          }
////        }
////      }
////      std::cout << "face_idx: " << face_idx << std::endl;
////      assert( face_idx <= 5 );
//      //active_set.push_back( new KinematicObjectSphereConstraint( second_body, sphere.r(), -normals[i], q1.segment<3>( 3 * first_body ), Vector3s::Zero(), Vector3s::Zero() ) );
////      active_set.push_back( std::unique_ptr<Constraint>( new KinematicBoxSphereConstraint( second_body, sphere.r(), -normals[i], q1.segment<3>( 3 * first_body ), Vector3s::Zero(), Vector3s::Zero(), first_body ) ) );
//    }
//  }
}

void RigidBody3DSim::sphereSphereNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const RigidBodySphere& sphere0, const RigidBodySphere& sphere1, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  assert( !isKinematicallyScripted( first_body ) ); // Kinematic rigid body should be listed second

  // Evaluation of active set at q1
  if( SphereSphereConstraint::isActive( q1.segment<3>( 3 * first_body ), q1.segment<3>( 3 * second_body ), sphere0.r(), sphere1.r() ) )
  {
    // Creation of constraints at q0 to preserve angular momentum
    const Vector3s n{ ( q0.segment<3>( 3 * first_body ) - q0.segment<3>( 3 * second_body ) ).normalized() };
    assert( fabs( n.norm() - 1.0 ) <= 1.0e-6 );

    if( !isKinematicallyScripted( second_body ) )
    {
      // Compute the contact point in q0
      const Vector3s p{ q0.segment<3>( 3 * first_body ) + ( sphere0.r() / ( sphere0.r() + sphere1.r() ) ) * ( q0.segment<3>( 3 * second_body ) - q0.segment<3>( 3 * first_body ) ) };
      active_set.emplace_back( new SphereSphereConstraint{ first_body, second_body, n, p, sphere0.r(), sphere1.r() } );
    }
    else
    {
      // TODO: When velocity is passed in, assert zero if keeping zero assumption
      active_set.emplace_back( new KinematicObjectSphereConstraint{ first_body, sphere0.r(), n, second_body, q0.segment<3>( 3 * second_body ), Vector3s::Zero(), Vector3s::Zero() } );
    }
  }
}

void RigidBody3DSim::stapleStapleNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const RigidBodyStaple& staple0, const RigidBodyStaple& staple1, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  // TODO: Staple-staple kinematic collisions not currently supported
  assert( !isKinematicallyScripted( first_body ) );
  assert( !isKinematicallyScripted( second_body) );

  const Matrix33sr R0{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * first_body ).data() } };
  const Matrix33sr R1{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * second_body ).data() } };

  // TODO: Should probably combine these functions
  if( StapleStapleUtilities::isActive( q1.segment<3>( 3 * first_body ), R0, staple0, q1.segment<3>( 3 * second_body ), R1, staple1 ) )
  {
    std::vector<Vector3s> p;
    std::vector<Vector3s> n;
    StapleStapleUtilities::computeConstraints( q1.segment<3>( 3 * first_body ), R0, staple0, q1.segment<3>( 3 * second_body ), R1, staple1, p, n );
    assert( p.size() == n.size() );
    for( std::vector<Vector3s>::size_type i = 0; i < p.size(); ++i )
    {
      active_set.emplace_back( new BodyBodyConstraint{ first_body, second_body, p[i], n[i], q0 } );
    }
  }
}

void RigidBody3DSim::meshMeshNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const RigidBodyTriangleMesh& mesh0, const RigidBodyTriangleMesh& mesh1, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  // TODO: Mesh-mesh kinematic collisions not currently supported
  assert( !isKinematicallyScripted( first_body ) );
  assert( !isKinematicallyScripted( second_body) );

  const Matrix33sr R0{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * first_body ).data() } };
  const Matrix33sr R1{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * second_body ).data() } };

  // Contact points in world-space
  std::vector<Vector3s> p;
  // Contact normals in world-space
  std::vector<Vector3s> n;

  // Compute any collision points and normals
  MeshMeshUtilities::computeActiveSet( q1.segment<3>( 3 * first_body ), R0, mesh0, q1.segment<3>( 3 * second_body ), R1, mesh1, p, n );
  assert( p.size() == n.size() );

  // Save the collisions
  for( std::vector<Vector3s>::size_type i = 0; i < p.size(); ++i )
  {
    active_set.emplace_back( new BodyBodyConstraint{ first_body, second_body, p[i], n[i], q0 } );
  }
}

// TODO: clean up this implementation like with the portal system
void RigidBody3DSim::dispatchNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  // Ignore kinematic-kinematic collisions
  if( isKinematicallyScripted( first_body ) && isKinematicallyScripted( second_body ) )
  {
    return;
  }

  // Ensure that the kinematic body is always listed second
  unsigned body0{ first_body };
  unsigned body1{ second_body };
  if( isKinematicallyScripted( body0 ) )
  {
    std::swap( body0, body1 );
  }

  if( m_sim_state.getGeometryOfBody( body0 ).getType() == RigidBodyGeometryType::BOX )
  {
    // Box-Box
    if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::BOX )
    {
      const RigidBodyBox& body_geo0{ sd_cast<const RigidBodyBox&>( m_sim_state.getGeometryOfBody( body0 ) ) };
      const RigidBodyBox& body_geo1{ sd_cast<const RigidBodyBox&>( m_sim_state.getGeometryOfBody( body1 ) ) };
      boxBoxNarrowPhaseCollision( body0, body1, body_geo0, body_geo1, q0, q1, active_set );
      return;
    }
    // Box-sphere
    else if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::SPHERE )
    {
      std::cerr << "Sphere-Box narrow phase is currently broken." << std::endl;
      std::exit( EXIT_FAILURE );
      //const RigidBodyBox& body_geo0 = sd_cast<const RigidBodyBox&>( m_sim_state.getGeometryOfBody( body0 ) );
      //const RigidBodySphere& body_geo1 = sd_cast<const RigidBodySphere&>( m_sim_state.getGeometryOfBody( body1 ) );
      //boxSphereNarrowPhaseCollision( body0, body1, body_geo0, body_geo1, q0, q1, active_set );
      //return;
    }
  }
  else if( m_sim_state.getGeometryOfBody( body0 ).getType() == RigidBodyGeometryType::SPHERE )
  {
    // Sphere-Sphere
    if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::SPHERE )
    {
      const RigidBodySphere& body_geo0{ sd_cast<const RigidBodySphere&>( m_sim_state.getGeometryOfBody( body0 ) ) };
      const RigidBodySphere& body_geo1{ sd_cast<const RigidBodySphere&>( m_sim_state.getGeometryOfBody( body1 ) ) };
      sphereSphereNarrowPhaseCollision( body0, body1, body_geo0, body_geo1, q0, q1, active_set );
      return;
    }
    // Sphere-Box
    else if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::BOX )
    {
      std::cerr << "Sphere-Box narrow phase is currently broken." << std::endl;
      std::exit( EXIT_FAILURE );
      //const RigidBodySphere& body_geo0 = sd_cast<const RigidBodySphere&>( m_sim_state.getGeometryOfBody( body0 ) );
      //const RigidBodyBox& body_geo1 = sd_cast<const RigidBodyBox&>( m_sim_state.getGeometryOfBody( body1 ) );
      //boxSphereNarrowPhaseCollision( body1, body0, body_geo1, body_geo0, q0, q1, active_set );
      //return;
    }
  }
  else if( m_sim_state.getGeometryOfBody( body0 ).getType() == RigidBodyGeometryType::STAPLE )
  {
    // Staple-Staple
    if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::STAPLE )
    {
      const RigidBodyStaple& body_geo0{ sd_cast<const RigidBodyStaple&>( m_sim_state.getGeometryOfBody( body0 ) ) };
      const RigidBodyStaple& body_geo1{ sd_cast<const RigidBodyStaple&>( m_sim_state.getGeometryOfBody( body1 ) ) };
      stapleStapleNarrowPhaseCollision( body0, body1, body_geo0, body_geo1, q0, q1, active_set );
      return;
    }
  }
  else if( m_sim_state.getGeometryOfBody( body0 ).getType() == RigidBodyGeometryType::TRIANGLE_MESH )
  {
    // Mesh-Mesh
    if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::TRIANGLE_MESH )
    {
      const RigidBodyTriangleMesh& body_geo0{ sd_cast<const RigidBodyTriangleMesh&>( m_sim_state.getGeometryOfBody( body0 ) ) };
      const RigidBodyTriangleMesh& body_geo1{ sd_cast<const RigidBodyTriangleMesh&>( m_sim_state.getGeometryOfBody( body1 ) ) };
      meshMeshNarrowPhaseCollision( body0, body1, body_geo0, body_geo1, q0, q1, active_set );
      return;
    }
  }

  std::cerr << "Collision between " << m_sim_state.getGeometryOfBody(body0).name() << " and " << m_sim_state.getGeometryOfBody(second_body).name() << " not supported. Exiting." << std::endl;
  std::exit(EXIT_FAILURE);
}

// TODO: Cleanup as above
bool RigidBody3DSim::collisionIsActive( const unsigned first_body, const unsigned second_body, const VectorXs& q0, const VectorXs& q1 ) const
{
  // Ignore kinematic-kinematic collisions
  if( isKinematicallyScripted( first_body ) && isKinematicallyScripted( second_body ) )
  {
    return false;
  }

  // Ensure that the kinematic body is always listed second
  unsigned body0{ first_body };
  unsigned body1{ second_body };
  if( isKinematicallyScripted( body0 ) )
  {
    std::swap( body0, body1 );
  }

  if( m_sim_state.getGeometryOfBody( body0 ).getType() == RigidBodyGeometryType::BOX )
  {
    // Box-Box
    if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::BOX )
    {
      const RigidBodyBox& body_geo0{ sd_cast<const RigidBodyBox&>( m_sim_state.getGeometryOfBody( body0 ) ) };
      const RigidBodyBox& body_geo1{ sd_cast<const RigidBodyBox&>( m_sim_state.getGeometryOfBody( body1 ) ) };
      std::vector<std::unique_ptr<Constraint>> temp_active_set;
      boxBoxNarrowPhaseCollision( body0, body1, body_geo0, body_geo1, q0, q1, temp_active_set );
      return !temp_active_set.empty();
    }
    // Box-sphere
    else if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::SPHERE )
    {
      std::cerr << "Sphere-Box narrow phase is currently broken." << std::endl;
      std::exit( EXIT_FAILURE );
      //const RigidBodyBox& body_geo0 = sd_cast<const RigidBodyBox&>( m_sim_state.getGeometryOfBody( body0 ) );
      //const RigidBodySphere& body_geo1 = sd_cast<const RigidBodySphere&>( m_sim_state.getGeometryOfBody( body1 ) );
      //std::vector<std::unique_ptr<Constraint>> temp_active_set;
      //boxSphereNarrowPhaseCollision( body0, body1, body_geo0, body_geo1, q0, q1, temp_active_set );
      //return !temp_active_set.empty();
    }
  }
  else if( m_sim_state.getGeometryOfBody( body0 ).getType() == RigidBodyGeometryType::SPHERE )
  {
    // Sphere-Sphere
    if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::SPHERE )
    {
      const RigidBodySphere& body_geo0{ sd_cast<const RigidBodySphere&>( m_sim_state.getGeometryOfBody( body0 ) ) };
      const RigidBodySphere& body_geo1{ sd_cast<const RigidBodySphere&>( m_sim_state.getGeometryOfBody( body1 ) ) };
      std::vector<std::unique_ptr<Constraint>> temp_active_set;
      sphereSphereNarrowPhaseCollision( body0, body1, body_geo0, body_geo1, q0, q1, temp_active_set );
      return !temp_active_set.empty();
    }
    // Sphere-Box
    else if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::BOX )
    {
      std::cerr << "Sphere-Box narrow phase is currently broken." << std::endl;
      std::exit( EXIT_FAILURE );
      //const RigidBodySphere& body_geo0 = sd_cast<const RigidBodySphere&>( m_sim_state.getGeometryOfBody( body0 ) );
      //const RigidBodyBox& body_geo1 = sd_cast<const RigidBodyBox&>( m_sim_state.getGeometryOfBody( body1 ) );
      //std::vector<std::unique_ptr<Constraint>> temp_active_set;
      //boxSphereNarrowPhaseCollision( body1, body0, body_geo1, body_geo0, q0, q1, temp_active_set );
      //return !temp_active_set.empty();
    }
  }
  else if( m_sim_state.getGeometryOfBody( body0 ).getType() == RigidBodyGeometryType::STAPLE )
  {
    // Staple-Staple
    if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::STAPLE )
    {
      const RigidBodyStaple& body_geo0{ sd_cast<const RigidBodyStaple&>( m_sim_state.getGeometryOfBody( body0 ) ) };
      const RigidBodyStaple& body_geo1{ sd_cast<const RigidBodyStaple&>( m_sim_state.getGeometryOfBody( body1 ) ) };
      std::vector<std::unique_ptr<Constraint>> temp_active_set;
      stapleStapleNarrowPhaseCollision( body0, body1, body_geo0, body_geo1, q0, q1, temp_active_set );
      return !temp_active_set.empty();
    }
  }
  else if( m_sim_state.getGeometryOfBody( body0 ).getType() == RigidBodyGeometryType::TRIANGLE_MESH )
  {
    // Mesh-Mesh
    if( m_sim_state.getGeometryOfBody( body1 ).getType() == RigidBodyGeometryType::TRIANGLE_MESH )
    {
      const RigidBodyTriangleMesh& body_geo0{ sd_cast<const RigidBodyTriangleMesh&>( m_sim_state.getGeometryOfBody( body0 ) ) };
      const RigidBodyTriangleMesh& body_geo1{ sd_cast<const RigidBodyTriangleMesh&>( m_sim_state.getGeometryOfBody( body1 ) ) };
      std::vector<std::unique_ptr<Constraint>> temp_active_set;
      meshMeshNarrowPhaseCollision( body0, body1, body_geo0, body_geo1, q0, q1, temp_active_set );
      return !temp_active_set.empty();
    }
  }

  std::cerr << "Collision between " << m_sim_state.getGeometryOfBody(body0).name() << " and " << m_sim_state.getGeometryOfBody(second_body).name() << " not supported. Exiting." << std::endl;
  std::exit( EXIT_FAILURE );
}


void RigidBody3DSim::generateAABBs( std::vector<AABB>& aabbs, const VectorXs& q )
{
  aabbs.resize( m_sim_state.nbodies() );
  for( unsigned body = 0; body < m_sim_state.nbodies(); ++body )
  {
    const Vector3s cm{ q.segment<3>( 3 * body ) };
    const Matrix33sr R{ Eigen::Map<const Matrix33sr>{ q.segment<9>( 3 * m_sim_state.nbodies() + 9 * body ).data() } };
    assert( ( R * R.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    assert( fabs( R.determinant() - 1.0 ) <= 1.0e-6 );
    m_sim_state.getGeometryOfBody( body ).computeAABB( cm, R, aabbs[body].min(), aabbs[body].max() );
    assert( ( aabbs[body].min() < aabbs[body].max() ).all() );
  }
}

// TODO: Move as much of this code into helper methods as possible
void RigidBody3DSim::computeActiveSetBodyBodySpatialGrid( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set )
{
  assert( q0.size() == 12 * m_sim_state.nbodies() );
  assert( q0.size() == q1.size() );

  const unsigned nbodies{ m_sim_state.nbodies() };
  
  // Candidate bodies that might overlap
  std::set<std::pair<unsigned,unsigned>> possible_overlaps;
  // Map from teleported AABB indices and body and portal indices
  std::map<unsigned,TeleportedBody> teleported_aabb_body_indices;
  {
    // Compute an AABB for each body
    std::vector<AABB> aabbs;
    generateAABBs( aabbs, q1 );
    assert( aabbs.size() == nbodies );

    // Compute an AABB for each teleported particle
    std::map<unsigned,TeleportedBody>::iterator aabb_bdy_map_itr{ teleported_aabb_body_indices.begin() };
    // For each portal
    for( std::vector<PlanarPortal>::size_type prtl_idx = 0; prtl_idx < m_sim_state.numPlanarPortals(); ++prtl_idx )
    {
      // For each body
      for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
      {
        // If the body is inside a portal
        bool intersecting_plane_index;
        if( m_sim_state.planarPortal( prtl_idx ).aabbTouchesPortal( aabbs[bdy_idx].min(), aabbs[bdy_idx].max(), intersecting_plane_index )  )
        {
          // Teleport to the other side of the portal
          Vector3s x_out;
          m_sim_state.planarPortal( prtl_idx ).teleportPoint( q1.segment<3>( 3 * bdy_idx ), intersecting_plane_index, x_out );

          // Compute an AABB for the teleported body
          AABB new_aabb;
          const Matrix33sr R{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * bdy_idx ).data() } };
          assert( ( R * R.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
          assert( fabs( R.determinant() - 1.0 ) <= 1.0e-6 );
          m_sim_state.getGeometryOfBody( bdy_idx ).computeAABB( x_out, R, new_aabb.min(), new_aabb.max() );
          assert( ( new_aabb.min() < new_aabb.max() ).all() );
          aabbs.emplace_back( new_aabb );
          aabb_bdy_map_itr = teleported_aabb_body_indices.insert( aabb_bdy_map_itr, std::make_pair( aabbs.size() - 1, TeleportedBody{ bdy_idx, unsigned( prtl_idx ), intersecting_plane_index } ) );
        }
      }
    }

    // Determine which bodies possibly overlap
    SpatialGridDetector::getPotentialOverlaps( aabbs, possible_overlaps );
  }

  std::set<TeleportedCollision> teleported_collisions;

  #ifndef NDEBUG
  std::vector<std::pair<unsigned,unsigned>> duplicate_indices;
  #endif

  // Create constraints for bodies that actually overlap
  for( const std::pair<unsigned,unsigned>& possible_overlap_pair : possible_overlaps )
  {
    const bool first_teleported{ possible_overlap_pair.first >= nbodies };
    const bool second_teleported{ possible_overlap_pair.second >= nbodies };

    // If neither ball in the current collision was teleported
    if( !first_teleported && !second_teleported )
    {
      if( isKinematicallyScripted( possible_overlap_pair.first ) && isKinematicallyScripted( possible_overlap_pair.second ) )
      {
        continue;
      }
      dispatchNarrowPhaseCollision( possible_overlap_pair.first, possible_overlap_pair.second, q0, q1, active_set );
    }
    // If at least one of the balls was teleported
    else
    {
      unsigned bdy_idx_0{ possible_overlap_pair.first };
      unsigned bdy_idx_1{ possible_overlap_pair.second };
      unsigned prtl_idx_0{ std::numeric_limits<unsigned>::max() };
      unsigned prtl_idx_1{ std::numeric_limits<unsigned>::max() };
      bool prtl_plane_0{ 0 };
      bool prtl_plane_1{ 0 };

      if( first_teleported )
      {
        const std::map<unsigned,TeleportedBody>::const_iterator map_itr{ teleported_aabb_body_indices.find( possible_overlap_pair.first ) };
        assert( map_itr != teleported_aabb_body_indices.end() );
        bdy_idx_0 = map_itr->second.bodyIndex();
        assert( bdy_idx_0 < nbodies );
        prtl_idx_0 = map_itr->second.portalIndex();
        assert( prtl_idx_0 < m_sim_state.numPlanarPortals() );
        prtl_plane_0 = map_itr->second.planeIndex();
      }
      if( second_teleported )
      {
        const std::map<unsigned,TeleportedBody>::const_iterator map_itr{ teleported_aabb_body_indices.find( possible_overlap_pair.second ) };
        assert( map_itr != teleported_aabb_body_indices.end() );
        bdy_idx_1 = map_itr->second.bodyIndex();
        assert( bdy_idx_1 < nbodies );
        prtl_idx_1 = map_itr->second.portalIndex();
        assert( prtl_idx_1 < m_sim_state.numPlanarPortals() );
        prtl_plane_1 = map_itr->second.planeIndex();
      }

      // Check if the collision will be detected in the unteleported state
      if( first_teleported && second_teleported )
      {
        if( collisionIsActive( bdy_idx_0, bdy_idx_1, q0, q1 ) )
        {
          #ifndef NDEBUG
          duplicate_indices.emplace_back( bdy_idx_0, bdy_idx_1 );
          #endif
          continue;
        }
      }

      // TODO: Merge this into generateTeleportedBallBallCollisions as in rb2d implementation
      // Check if the collision actually happens
      if( isKinematicallyScripted( bdy_idx_0 ) && isKinematicallyScripted( bdy_idx_1 ) )
      {
        continue;
      }
      const TeleportedCollision possible_collision{ bdy_idx_0, bdy_idx_1, prtl_idx_0,  prtl_idx_1, prtl_plane_0, prtl_plane_1 };
      if( teleportedCollisionHappens( q1, possible_collision ) )
      {
        teleported_collisions.insert( possible_collision );
      }
    }
  }
  possible_overlaps.clear();
  teleported_aabb_body_indices.clear();

  #ifndef NDEBUG
  // Double check that non-teleport duplicate collisions were actually duplicates
  for( const std::pair<unsigned,unsigned>& dup_col : duplicate_indices )
  {
    bool entry_found{ false };
    const unsigned dup_idx_0{ std::min( dup_col.first, dup_col.second ) };
    const unsigned dup_idx_1{ std::max( dup_col.first, dup_col.second ) };
    for( const std::unique_ptr<Constraint>& col : active_set )
    {
      std::pair<int,int> bodies;
      col->getBodyIndices( bodies );
      if( bodies.first < 0 || bodies.second < 0 )
      {
        continue;
      }
      const unsigned idx_0{ std::min( unsigned( bodies.first ), unsigned( bodies.second ) ) };
      const unsigned idx_1{ std::max( unsigned( bodies.first ), unsigned( bodies.second ) ) };
      if( dup_idx_0 == idx_0 && dup_idx_1 == idx_1 )
      {
        entry_found = true;
        break;
      }
    }
    assert( entry_found );
  }
  #endif

  // Create constraints for teleported collisions
  for( const TeleportedCollision& teleported_collision : teleported_collisions )
  {
    assert( teleported_collision.bodyIndex0() < nbodies );
    assert( teleported_collision.bodyIndex1() < nbodies );
    assert( teleported_collision.bodyIndex0() != teleported_collision.bodyIndex1( ) );
    generateTeleportedCollision( q0, teleported_collision, active_set );
  }

  // TODO: This test only works if one collision is possible between any given pair.
  //       Need a nice way to check collisions for equality.
  //#ifndef NDEBUG
  //// Do an all pairs check for duplicates
  //const unsigned ncons = active_set.size();
  //for( unsigned con_idx_0 = 0; con_idx_0 < ncons; ++con_idx_0 )
  //{
  //  std::pair<int,int> indices0;
  //  active_set[con_idx_0]->getBodyIndices( indices0 );
  //  const int idx_0a = std::min( indices0.first, indices0.second );
  //  const int idx_1a = std::max( indices0.first, indices0.second );
  //  for( unsigned con_idx_1 = con_idx_0 + 1; con_idx_1 < ncons; ++con_idx_1 )
  //  {
  //    std::pair<int,int> indices1;
  //    active_set[con_idx_1]->getBodyIndices( indices1 );
  //    const int idx_0b = std::min( indices1.first, indices1.second );
  //    const int idx_1b = std::max( indices1.first, indices1.second );
  //    assert( !( ( idx_0a == idx_0b ) && ( idx_1a == idx_1b ) ) );
  //  }
  //}
  //#endif
}

bool RigidBody3DSim::teleportedCollisionHappens( const VectorXs& q, const TeleportedCollision& teleported_collision ) const
{
  assert( q.size() % 12 == 0 );

  // Center of mass of each body
  Vector3s x0;
  Vector3s x1;
  getTeleportedCollisionCenters( q, teleported_collision, x0, x1 );

  // Orientation of each body
  //const Matrix33sr R0 = Eigen::Map<const Matrix33sr>( q.segment<9>( 3 * m_sim_state.nbodies() + 9 * teleported_collision.bodyIndex0() ).data() );
  //const Matrix33sr R1 = Eigen::Map<const Matrix33sr>( q.segment<9>( 3 * m_sim_state.nbodies() + 9 * teleported_collision.bodyIndex1() ).data() );

  // Geometry of each body
  const RigidBodyGeometry& geo0{ m_sim_state.getGeometryOfBody( teleported_collision.bodyIndex0() ) };
  const RigidBodyGeometry& geo1{ m_sim_state.getGeometryOfBody( teleported_collision.bodyIndex1() ) };

  // Sphere-
  if( geo0.getType() == RigidBodyGeometryType::SPHERE )
  {
    // Sphere-Sphere
    if( geo1.getType() == RigidBodyGeometryType::SPHERE )
    {
      const RigidBodySphere& sphere0{ sd_cast<const RigidBodySphere&>( geo0 ) };
      const RigidBodySphere& sphere1{ sd_cast<const RigidBodySphere&>( geo1 ) };
      return SphereSphereConstraint::isActive( x0, x1, sphere0.r(), sphere1.r() );
    }
  }

  std::cerr << "Teleported collision happens not supported for: ";
  std::cerr << geo0.name() << " vs " << geo1.name() << std::endl;
  std::exit( EXIT_FAILURE );
}

void RigidBody3DSim::getTeleportedCollisionCenters( const VectorXs& q, const TeleportedCollision& teleported_collision, Vector3s& x0, Vector3s& x1 ) const
{
  assert( q.size() % 12 == 0 );

  // Indices of the colliding bodies
  const unsigned idx0{ teleported_collision.bodyIndex0() };
  assert( idx0 < unsigned( m_sim_state.nbodies() ) );
  const unsigned idx1{ teleported_collision.bodyIndex1() };
  assert( idx1 < unsigned( m_sim_state.nbodies() ) );

  // Indices of the portals
  const unsigned prtl_idx0{ teleported_collision.portalIndex0() };
  assert( prtl_idx0 < m_sim_state.numPlanarPortals() || prtl_idx0 == std::numeric_limits<unsigned>::max() );
  const unsigned prtl_idx1{ teleported_collision.portalIndex1() };
  assert( prtl_idx1 < m_sim_state.numPlanarPortals() || prtl_idx1 == std::numeric_limits<unsigned>::max() );

  // If the first object was teleported
  assert( 3 * idx0 + 2 < q.size() );
  x0 = q.segment<3>( 3 * idx0 );
  if( prtl_idx0 != std::numeric_limits<unsigned>::max() )
  {
    if( teleported_collision.plane0() == 0 )
    {
      m_sim_state.planarPortal( prtl_idx0 ).teleportPointThroughPlaneA( x0, x0 );
    }
    else
    {
      m_sim_state.planarPortal( prtl_idx0 ).teleportPointThroughPlaneB( x0, x0 );
    }
  }

  // If the second object was teleported
  assert( 3 * idx1 + 2 < q.size() );
  x1 = q.segment<3>( 3 * idx1 );
  if( prtl_idx1 != std::numeric_limits<unsigned>::max() )
  {
    if( teleported_collision.plane1() == 0 )
    {
      m_sim_state.planarPortal( prtl_idx1 ).teleportPointThroughPlaneA( x1, x1 );
    }
    else
    {
      m_sim_state.planarPortal( prtl_idx1 ).teleportPointThroughPlaneB( x1, x1 );
    }
  }
}

void RigidBody3DSim::generateTeleportedCollision( const VectorXs& q, const TeleportedCollision& teleported_collision, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  assert( q.size() % 12 == 0 );

  // Indices of the colliding bodies
  const unsigned idx0{ teleported_collision.bodyIndex0() };
  assert( idx0 < unsigned( m_sim_state.nbodies() ) );
  const unsigned idx1{ teleported_collision.bodyIndex1() };
  assert( idx1 < unsigned( m_sim_state.nbodies() ) );

  // Get the geometry of each body
  const RigidBodyGeometry& geo0{ m_sim_state.getGeometryOfBody( idx0 ) };
  const RigidBodyGeometry& geo1{ m_sim_state.getGeometryOfBody( idx1 ) };

  // Only sphere-sphere supported
  if( geo0.getType() != RigidBodyGeometryType::SPHERE || geo1.getType() != RigidBodyGeometryType::SPHERE )
  {
    std::cerr << "Generate teleported collision not supported for: ";
    std::cerr << geo0.name() << " vs " << geo1.name() << std::endl;
    std::exit( EXIT_FAILURE );
  }

  const scalar& r0{ sd_cast<const RigidBodySphere&>( geo0 ).r() };
  assert( r0 > 0.0 );
  const scalar& r1{ sd_cast<const RigidBodySphere&>( geo1 ).r() };
  assert( r1 > 0.0 );

  Vector3s x0;
  Vector3s x1;
  getTeleportedCollisionCenters( q, teleported_collision, x0, x1 );

  // Handle kinematically scripted bodies
  assert( !( isKinematicallyScripted( idx0 ) && isKinematicallyScripted( idx1 ) ) );
  if( isKinematicallyScripted( idx0 ) && !isKinematicallyScripted( idx1 ) )
  {
    // Creation of constraints at q0 to preserve angular momentum
    const Vector3s n{ ( x1 - x0 ).normalized() };
    active_set.emplace_back( new KinematicObjectSphereConstraint{ idx1, r1, n, idx0, x0, Vector3s::Zero(), Vector3s::Zero() } );
  }
  else if( !isKinematicallyScripted( idx0 ) && isKinematicallyScripted( idx1 ) )
  {
    // Creation of constraints at q0 to preserve angular momentum
    const Vector3s n{ ( x0 - x1 ).normalized() };
    active_set.emplace_back( new KinematicObjectSphereConstraint{ idx0, r0, n, idx1, x1, Vector3s::Zero(), Vector3s::Zero() } );
  }
  else if( !isKinematicallyScripted( idx0 ) && !isKinematicallyScripted( idx1 ) )
  {
    active_set.emplace_back( new TeleportedSphereSphereConstraint{ idx0, idx1, x0, x1, r0, r1 } );
  }
  else
  {
    std::cerr << "Impossible case hit." << std::endl;
    std::exit( EXIT_FAILURE );
  }
}


//void RigidBody3DSim::computeActiveSetBodyBodyAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
//{
//  assert( q0.size() == q1.size() ); assert( q0.size() == 12 * m_sim_state.nbodies() );
//
//  for( int first_body = 0; first_body < m_sim_state.nbodies(); ++first_body )
//  {
//    for( int second_body = first_body + 1; second_body < m_sim_state.nbodies(); ++second_body )
//    {
//      dispatchNarrowPhaseCollision( first_body, second_body, q0, q1, active_set );
//    }
//  }
//}

// TODO: Move all of the ugly code bits in here into their own functions
void RigidBody3DSim::computeBodyPlaneActiveSetAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  assert( q0.size() == q1.size() );
  assert( q0.size() == 12 * m_sim_state.nbodies() );

  // Check all plane-body pairs
  for( std::vector<StaticPlane>::size_type plane = 0; plane < m_sim_state.staticPlanes().size(); ++plane )
  {
    for( unsigned body = 0; body < m_sim_state.nbodies(); ++body )
    {
      // Skip kinematically scripted bodies
      if( isKinematicallyScripted( body ) )
      {
        continue;
      }

      if( m_sim_state.getGeometryOfBody(body).getType() == RigidBodyGeometryType::BOX )
      {
        const RigidBodyBox& box{ sd_cast<const RigidBodyBox&>( m_sim_state.getGeometryOfBody( body ) ) };
        const Matrix33sr R{ Eigen::Map<const Matrix33sr>( q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * body ).data() ) };
        std::vector<short> active_corners;
        const bool box_plane_happens{ StaticPlaneBoxConstraint::isActive( m_sim_state.staticPlanes()[plane].x(), m_sim_state.staticPlanes()[plane].n(), q1.segment<3>( 3 * body ), R, box.halfWidths(), active_corners ) };
        if( box_plane_happens )
        {
          assert( !active_corners.empty() );
          for( std::vector<short>::size_type i = 0; i < active_corners.size(); ++i )
          {
            active_set.emplace_back( new StaticPlaneBoxConstraint{ body, active_corners[i], m_sim_state.staticPlanes()[plane].n(), box.halfWidths(), q0, static_cast<unsigned>( plane ) } );
          }
        }
      }
      else if( m_sim_state.getGeometryOfBody(body).getType() == RigidBodyGeometryType::SPHERE )
      {
        const RigidBodySphere& sphere{ sd_cast<const RigidBodySphere&>( m_sim_state.getGeometryOfBody( body ) ) };
        if( StaticPlaneSphereConstraint::isActive( m_sim_state.staticPlanes()[plane].x(), m_sim_state.staticPlanes()[plane].n(), q1.segment<3>( 3 * body ), sphere.r() ) )
        {
          active_set.emplace_back( new StaticPlaneSphereConstraint{ body, sphere.r(), m_sim_state.staticPlane( plane ), static_cast<unsigned>( plane ) } );
        }
      }
      else if( m_sim_state.getGeometryOfBody(body).getType() == RigidBodyGeometryType::STAPLE )
      {
        const RigidBodyStaple& staple{ sd_cast<const RigidBodyStaple&>( m_sim_state.getGeometryOfBody( body ) ) };
        const Matrix33sr R{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * body ).data() } };
        std::vector<int> points;
        StapleStapleUtilities::computeStapleHalfPlaneActiveSet( q1.segment<3>( 3 * body ), R, staple, m_sim_state.staticPlanes()[plane].x(), m_sim_state.staticPlanes()[plane].n(), points );
        assert( points.size() <= 4 );
        std::cerr << "This code is buggy. It is confusing the rotation at q0 and q1. Fix up like I did below." << std::endl;
        std::exit( EXIT_FAILURE );
        //for( std::vector<int>::size_type i = 0; i < points.size(); ++i )
        //{
        //  assert( points[i] >= 0 ); assert( points[i] <= 4 );
        //  std::cerr << "This code is buggy. It is confusing the rotation at q0 and q1. Fix up like I did below." << std::endl;
        //  std::exit( EXIT_FAILURE );
        //  // Compute the point of the collision
        //  const Vector3s point = q0.segment<3>( 3 * body ) + R * staple.points()[points[i]] - staple.r() * m_sim_state.staticPlanes()[plane].n();
        //  // Create a constraint
        //  active_set.push_back( std::unique_ptr<Constraint>( new StaticPlaneBodyConstraint( body, point, m_sim_state.staticPlane(plane).n(), q0, plane ) ) );
        //}
      }
      else if( m_sim_state.getGeometryOfBody(body).getType() == RigidBodyGeometryType::TRIANGLE_MESH )
      {
        const RigidBodyTriangleMesh& mesh{ sd_cast<const RigidBodyTriangleMesh&>( m_sim_state.getGeometryOfBody( body ) ) };
        // Determine which vertices of the mesh collide with the half plane
        std::vector<unsigned> colliding_convex_hull_vertices;
        {
          const Vector3s cm1{ q1.segment<3>( 3 * body ) };
          const Matrix33sr R1{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * body ).data() } };
          MeshMeshUtilities::computeMeshHalfPlaneActiveSet( cm1, R1, mesh, m_sim_state.staticPlanes()[plane].x(), m_sim_state.staticPlanes()[plane].n(), colliding_convex_hull_vertices );
        }
        // Create constraints for each vertex
        {
          const Vector3s cm0{ q0.segment<3>( 3 * body ) };
          const Matrix33sr R0{ Eigen::Map<const Matrix33sr>{ q0.segment<9>( 3 * m_sim_state.nbodies() + 9 * body ).data() } };
          for( std::vector<unsigned>::size_type vrt_idx = 0; vrt_idx < colliding_convex_hull_vertices.size(); ++vrt_idx )
          {
            // Compute the point of the collision
            const Vector3s point{ cm0 + R0 * mesh.convexHullVertices().col( colliding_convex_hull_vertices[vrt_idx] ) };
            active_set.emplace_back( new StaticPlaneBodyConstraint{ body, point, m_sim_state.staticPlane(plane).n(), q0, static_cast<unsigned>( plane ) } );
          }
        }
      }
      else
      {
        std::cerr << "Collision between static planes and " << m_sim_state.getGeometryOfBody(body).name() << " not supported. Exiting." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
  }
}

void RigidBody3DSim::computeBodyCylinderActiveSetAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  assert( q0.size() == q1.size() );
  assert( q0.size() == 12 * m_sim_state.nbodies() );

  // Check all cylinder-body pairs
  for( std::vector<StaticCylinder>::size_type cyl = 0; cyl < m_sim_state.staticCylinders().size(); ++cyl )
  {
    for( unsigned body = 0; body < m_sim_state.nbodies(); ++body )
    {
      // Skip kinematically scripted bodies
      if( isKinematicallyScripted( body ) )
      {
        continue;
      }

      if( m_sim_state.getGeometryOfBody(body).getType() == RigidBodyGeometryType::SPHERE )
      {
        const RigidBodySphere& sphere{ sd_cast<const RigidBodySphere&>( m_sim_state.getGeometryOfBody( body ) ) };
        if( StaticCylinderSphereConstraint::isActive( m_sim_state.staticCylinder(cyl).x(), m_sim_state.staticCylinder(cyl).axis(), m_sim_state.staticCylinder(cyl).r(), q1.segment<3>( 3 * body ), sphere.r() ) )
        {
          active_set.emplace_back( new StaticCylinderSphereConstraint{ body, sphere.r(), m_sim_state.staticCylinder(cyl), static_cast<unsigned>( cyl ) } );
        }
      }
      else if( m_sim_state.getGeometryOfBody(body).getType() == RigidBodyGeometryType::TRIANGLE_MESH )
      {
        const RigidBodyTriangleMesh& mesh{ sd_cast<const RigidBodyTriangleMesh&>( m_sim_state.getGeometryOfBody( body ) ) };
        // Determine which vertices of the mesh collide with the cylinder
        std::vector<unsigned> colliding_convex_hull_vertices;
        {
          const Vector3s cm1{ q1.segment<3>( 3 * body ) };
          const Matrix33sr R1{ Eigen::Map<const Matrix33sr>{ q1.segment<9>( 3 * m_sim_state.nbodies() + 9 * body ).data() } };
          MeshMeshUtilities::computeMeshCylinderActiveSet( cm1, R1, mesh, m_sim_state.staticCylinder(cyl).x(), m_sim_state.staticCylinder(cyl).axis(), m_sim_state.staticCylinder(cyl).r(), colliding_convex_hull_vertices );
        }
        // Create constraints for each vertex
        {
          const Vector3s cm0{ q0.segment<3>( 3 * body ) };
          const Matrix33sr R0{ Eigen::Map<const Matrix33sr>{ q0.segment<9>( 3 * m_sim_state.nbodies() + 9 * body ).data() } };
          for( std::vector<unsigned>::size_type vrt_idx = 0; vrt_idx < colliding_convex_hull_vertices.size(); ++vrt_idx )
          {
            // Compute the point of the collision
            const Vector3s point{ cm0 + R0 * mesh.convexHullVertices().col( colliding_convex_hull_vertices[vrt_idx] ) };
            active_set.emplace_back( new StaticCylinderBodyConstraint{ body, point, m_sim_state.staticCylinder(cyl), static_cast<unsigned>( cyl ), q0 } );
          }
        }
      }
      else
      {
        std::cerr << "Collision between static cylinders and " << m_sim_state.getGeometryOfBody(body).name() << " not supported. Exiting." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
  }
}

void RigidBody3DSim::writeBinaryState( HDF5File& output_file ) const
{
  #ifdef USE_HDF5
  // Output the simulated geometry
  output_file.createGroup( "geometry" );
  StateOutput::writeGeometryIndices( m_sim_state.geometry(), m_sim_state.indices(), "geometry", output_file );
  StateOutput::writeGeometry( m_sim_state.geometry(), "geometry", output_file );
  // Output the static geometry
  output_file.createGroup( "static_geometry" );
  if( !m_sim_state.staticPlanes().empty() )
  {
    StateOutput::writeStaticPlanes( m_sim_state.staticPlanes(), "static_geometry", output_file );
  }
  if( !m_sim_state.staticCylinders().empty() )
  {
    StateOutput::writeStaticCylinders( m_sim_state.staticCylinders(), "static_geometry", output_file );
  }
  // Write out the state of each body
  output_file.createGroup( "state" );
  output_file.writeMatrix( "state", "q", m_sim_state.q() );
  output_file.writeMatrix( "state", "v", m_sim_state.v() );
  output_file.writeSparseMatrix( "state", "M0", m_sim_state.M0() );
  {
    VectorXu fixed{ m_sim_state.nbodies() };
    for( unsigned body_index = 0; body_index < m_sim_state.nbodies(); ++body_index )
    {
      fixed( body_index ) = m_sim_state.isKinematicallyScripted( body_index ) ? 1 : 0;
    }
    output_file.writeMatrix( "state", "kinematically_scripted", fixed );
  }
  #else
  std::cerr << "Error, RigidBody3DSim::writeBinaryState requires HDF5 support." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

void RigidBody3DSim::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  m_sim_state.serialize( output_stream );
  // Nothing to serialize for m_impact_map
  m_constraint_cache.serialize( output_stream );
}

void RigidBody3DSim::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );
  m_sim_state.deserialize( input_stream );
  // Nothing to deserialize for m_impact_map
  m_constraint_cache.deserialize( input_stream );
}

ImpactMap& RigidBody3DSim::impactMap()
{
  return m_impact_map;
}
