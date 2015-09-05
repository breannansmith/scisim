// Ball2DSim.cpp
//
// Breannan Smith
// Last updated: 09/05/2015

#include "Ball2DSim.h"

#include "SCISim/UnconstrainedMaps/UnconstrainedMap.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "SCISim/ConstrainedMaps/ImpactFrictionMap.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "SCISim/Math/MathUtilities.h"
#include "SCISim/Utilities.h"
#include "SCISim/HDF5File.h"
#include "SCISim/Math/Rational.h"

#include "Constraints/BallBallConstraint.h"
#include "Constraints/KinematicKickBallBallConstraint.h"
#include "Constraints/BallStaticDrumConstraint.h"
#include "Constraints/BallStaticPlaneConstraint.h"

#include "Portals/PlanarPortal.h"

#include "StaticGeometry/StaticDrum.h"
#include "StaticGeometry/StaticPlane.h"

#include "SpatialGridDetector.h"

#include "PythonScripting.h"

#include <iostream>

Ball2DSim::Ball2DSim()
: m_state()
, m_constraint_cache()
{}

void Ball2DSim::setState( const Ball2DState& new_state )
{
  m_state = new_state;
  m_constraint_cache.clear();
}

void Ball2DSim::swapState( Ball2DState& new_state )
{
  // TODO: Also swap constraint cache when that is back
  swap( m_state, new_state );
  m_constraint_cache.clear();
}

Ball2DState& Ball2DSim::state()
{
  return m_state;
}

const Ball2DState& Ball2DSim::state() const
{
  return m_state;
}

bool Ball2DSim::empty() const
{
  return nqdofs() == 0;
}

int Ball2DSim::nqdofs() const
{
  return m_state.q().size();
}

int Ball2DSim::nvdofs() const
{
  return m_state.v().size();
}

unsigned Ball2DSim::numVelDoFsPerBody() const
{
  return 2;
}

unsigned Ball2DSim::ambientSpaceDimensions() const
{
  return 2;
}

bool Ball2DSim::isKinematicallyScripted( const int i ) const
{
  assert( i >= 0 ); assert( nvdofs() % 2 == 0 ); assert( i < nvdofs() / 2 );
  return false;
}

void Ball2DSim::computeForce( const VectorXs& q, const VectorXs& v, const scalar& t, VectorXs& F )
{
  assert( q.size() == v.size() ); assert( q.size() == F.size() );

  F.setZero();
  m_state.accumulateForce( q, v, F );
}

const SparseMatrixsc& Ball2DSim::M() const
{
  return m_state.M();
}

const SparseMatrixsc& Ball2DSim::Minv() const
{
  return m_state.Minv();
}

const SparseMatrixsc& Ball2DSim::M0() const
{
  // Mass matrix is invaraint to configuration for this system
  return m_state.M();
}

const SparseMatrixsc& Ball2DSim::Minv0() const
{
  // Mass matrix is invaraint to configuration for this system
  return m_state.Minv();
}

void Ball2DSim::computeMomentum( const VectorXs& v, VectorXs& p ) const
{
  const unsigned nballs{ m_state.nballs() };
  assert( v.size() == 2 * nballs );
  p = Vector2s::Zero();
  for( unsigned ball_idx = 0; ball_idx < nballs; ++ball_idx )
  {
    assert( m_state.M().valuePtr()[ 2 * ball_idx ] == m_state.M().valuePtr()[ 2 * ball_idx + 1 ] );
    p += m_state.M().valuePtr()[ 2 * ball_idx ] * v.segment<2>( 2 * ball_idx );
  }
}

void Ball2DSim::computeAngularMomentum( const VectorXs& v, VectorXs& L ) const
{
  const unsigned nballs{ m_state.nballs() };
  assert( v.size() == 2 * nballs );
  L = VectorXs::Zero( 1 );
  for( unsigned ball_idx = 0; ball_idx < nballs; ++ball_idx )
  {
    assert( m_state.M().valuePtr()[ 2 * ball_idx ] == m_state.M().valuePtr()[ 2 * ball_idx + 1 ] );
    L(0) += m_state.M().valuePtr()[ 2 * ball_idx ] * mathutils::cross( m_state.q().segment<2>( 2 * ball_idx ), v.segment<2>( 2 * ball_idx ) );
  }
}

void Ball2DSim::computeActiveSet( const VectorXs& q0, const VectorXs& qp, std::vector<std::unique_ptr<Constraint>>& active_set )
{
  assert( q0.size() % 2 == 0 );
  assert( q0.size() / 2 == m_state.r().size() );
  assert( q0.size() == qp.size() );
  assert( active_set.empty() );

  // Detect ball-ball collisions
  computeBallBallActiveSetSpatialGrid( q0, qp, active_set );

  // Check all ball-drum pairs
  computeBallDrumActiveSetAllPairs( q0, qp, active_set );

  // Check all ball-half-plane pairs
  computeBallPlaneActiveSetAllPairs( q0, qp, active_set );
}

void Ball2DSim::computeImpactBases( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& active_set, MatrixXXsc& impact_bases ) const
{
  const unsigned ncols{ static_cast<unsigned>( active_set.size() ) };
  impact_bases.resize( 2, ncols );
  for( unsigned col_num = 0; col_num < ncols; ++col_num )
  {
    VectorXs current_normal;
    active_set[col_num]->getWorldSpaceContactNormal( q, current_normal );
    assert( fabs( current_normal.norm() - 1.0 ) <= 1.0e-6 );
    impact_bases.col( col_num ) = current_normal;
  }
}

void Ball2DSim::computeContactBases( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& active_set, MatrixXXsc& contact_bases ) const
{
  const unsigned ncols{ static_cast<unsigned>( active_set.size() ) };
  contact_bases.resize( 2, 2 * ncols );
  for( unsigned col_num = 0; col_num < ncols; ++col_num )
  {
    MatrixXXsc basis;
    active_set[col_num]->computeBasis( q, v, basis );
    assert( basis.rows() == basis.cols() ); assert( basis.rows() == 2 );
    assert( ( basis * basis.transpose() - MatrixXXsc::Identity( 2, 2 ) ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-6 );
    contact_bases.block<2,2>( 0, 2 * col_num ) = basis;
  }
}

void Ball2DSim::clearConstraintCache()
{
  m_constraint_cache.clear();
}

void Ball2DSim::cacheConstraint( const Constraint& constraint, const VectorXs& r )
{
  m_constraint_cache.cacheConstraint( constraint, r );
}

void Ball2DSim::getCachedConstraintImpulse( const Constraint& constraint, VectorXs& r ) const
{
  m_constraint_cache.getCachedConstraint( constraint, r );
}

bool Ball2DSim::equal( const Constraint& constraint0, const Constraint& constraint1 ) const
{
  std::cerr << "Code up Ball2DSim::equal" << std::endl;
  std::exit( EXIT_FAILURE );
}

void Ball2DSim::computeNumberOfCollisions( std::map<std::string,unsigned>& collision_counts, std::map<std::string,scalar>& collision_depths )
{
  collision_counts.clear();
  collision_depths.clear();
  std::vector<std::unique_ptr<Constraint>> active_set;
  computeActiveSet( m_state.q(), m_state.q(), active_set );
  for( const std::unique_ptr<Constraint>& constraint : active_set )
  {
    const std::string constraint_name{ constraint->name() };

    // Update the collision count for this constraint
    {
      std::map<std::string,unsigned>::iterator count_iterator{ collision_counts.find( constraint_name ) };
      if( count_iterator == collision_counts.end() )
      {
        const std::pair<std::map<std::string,unsigned>::iterator,bool> count_return{ collision_counts.insert( std::make_pair( constraint_name, 0 ) ) };
        assert( count_return.second ); assert( count_return.first != collision_counts.end() );
        count_iterator = count_return.first;
      }
      ++count_iterator->second;
    }

    // Update the penetration depth for this constraint
    {
      std::map<std::string,scalar>::iterator depth_iterator{ collision_depths.find( constraint_name ) };
      if( depth_iterator == collision_depths.end() )
      {
        const std::pair<std::map<std::string,scalar>::iterator,bool> depths_return{ collision_depths.insert( std::make_pair( constraint_name, 0 ) ) };
        assert( depths_return.second ); assert( depths_return.first != collision_depths.end() );
        depth_iterator = depths_return.first;
      }
      depth_iterator->second += constraint->penetrationDepth( m_state.q() );
    }
  }
}

void Ball2DSim::flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap )
{
  call_back.setState( m_state );
  call_back.startOfStepCallback( iteration, dt );
  call_back.forgetState();

  VectorXs q1{ m_state.q().size() };
  VectorXs v1{ m_state.v().size() };

  updatePeriodicBoundaryConditionsStartOfStep( iteration, scalar(dt) );

  umap.flow( m_state.q(), m_state.v(), *this, iteration, scalar(dt), q1, v1 );

  q1.swap( m_state.q() );
  v1.swap( m_state.v() );

  enforcePeriodicBoundaryConditions();

  call_back.setState( m_state );
  call_back.endOfStepCallback( iteration, dt );
  call_back.forgetState();
}

void Ball2DSim::flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap, ImpactOperator& iop, const scalar& CoR, ImpactMap& imap )
{
  call_back.setState( m_state );
  call_back.startOfStepCallback( iteration, dt );
  call_back.forgetState();

  VectorXs q1{ m_state.q().size() };
  VectorXs v1{ m_state.v().size() };

  updatePeriodicBoundaryConditionsStartOfStep( iteration, scalar(dt) );

  imap.flow( call_back, *this, *this, umap, iop, iteration, scalar(dt), CoR, m_state.q(), m_state.v(), q1, v1 );

  q1.swap( m_state.q() );
  v1.swap( m_state.v() );

  enforcePeriodicBoundaryConditions();

  call_back.setState( m_state );
  call_back.endOfStepCallback( iteration, dt );
  call_back.forgetState();
}

void Ball2DSim::flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap, const scalar& CoR, const scalar& mu, FrictionSolver& solver, ImpactFrictionMap& ifmap )
{
  call_back.setState( m_state );
  call_back.startOfStepCallback( iteration, dt );
  call_back.forgetState();

  VectorXs q1{ m_state.q().size() };
  VectorXs v1{ m_state.v().size() };

  updatePeriodicBoundaryConditionsStartOfStep( iteration, scalar(dt) );

  ifmap.flow( call_back, *this, *this, umap, solver, iteration, scalar(dt), CoR, mu, m_state.q(), m_state.v(), q1, v1 );

  q1.swap( m_state.q() );
  v1.swap( m_state.v() );

  enforcePeriodicBoundaryConditions();

  call_back.setState( m_state );
  call_back.endOfStepCallback( iteration, dt );
  call_back.forgetState();
}

void Ball2DSim::updatePeriodicBoundaryConditionsStartOfStep( const unsigned next_iteration, const scalar& dt )
{
  const scalar t{ next_iteration * dt };
  for( PlanarPortal& planar_portal : m_state.planarPortals() )
  {
    planar_portal.updateMovingPortals( t );
  }
}

void Ball2DSim::enforcePeriodicBoundaryConditions()
{
  assert( m_state.q().size() % 2 == 0 );

  const unsigned nbodies{ m_state.nballs() };

  // For each portal
  for( const PlanarPortal& planar_portal : m_state.planarPortals() )
  {
    // For each body
    for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
    {
      // TODO: Calling pointInsidePortal and teleportPointInsidePortal is a bit redundant, clean this up!
      // If the body is inside a portal
      if( planar_portal.pointInsidePortal( m_state.q().segment<2>( 2 * bdy_idx ) ) )
      {
        // Teleport to the other side of the portal
        Vector2s x_out;
        planar_portal.teleportPointInsidePortal( m_state.q().segment<2>( 2 * bdy_idx ), x_out );
        m_state.q().segment<2>( 2 * bdy_idx ) = x_out;
      }
    }
  }
}

void Ball2DSim::computeBallBallActiveSetSpatialGrid( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  assert( q0.size() % 2 == 0 ); assert( q0.size() == q1.size() );
  assert( m_state.r().size() == q0.size() / 2 );

  const unsigned nbodies{ m_state.nballs() };

  // Candidate bodies that might overlap
  std::set<std::pair<unsigned,unsigned>> possible_overlaps;
  // Map from teleported AABB indices and body and portal indices
  std::map<unsigned,TeleportedBall> teleported_aabb_body_indices;
  {
    // Compute an AABB for each ball
    std::vector<AABB> aabbs;
    aabbs.reserve( nbodies );
    for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
    {
      aabbs.emplace_back( q1.segment<2>( 2 * bdy_idx ).array() - m_state.r()( bdy_idx ), q1.segment<2>( 2 * bdy_idx ).array() + m_state.r()( bdy_idx ) );
    }
    assert( aabbs.size() == nbodies );

    // Compute an AABB for each teleported particle
    std::map<unsigned,TeleportedBall>::iterator aabb_bdy_map_itr = teleported_aabb_body_indices.begin();
    // For each portal
    for( const PlanarPortal& planar_portal : m_state.planarPortals() )
    {
      // For each body
      for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
      {
        // If the body is inside a portal
        bool intersecting_plane_index;
        if( planar_portal.ballTouchesPortal( q1.segment<2>( 2 * bdy_idx ), m_state.r()( bdy_idx ), intersecting_plane_index )  )
        {
          // Teleport to the other side of the portal
          Vector2s x_out;
          planar_portal.teleportBall( q1.segment<2>( 2 * bdy_idx ), m_state.r()( bdy_idx ), x_out );
          // Compute an AABB for the teleported particle
          aabbs.emplace_back( x_out.array() - m_state.r()( bdy_idx ), x_out.array() + m_state.r()( bdy_idx ) );

          const unsigned prtl_idx{ Utilities::index( m_state.planarPortals(), planar_portal ) };
          aabb_bdy_map_itr = teleported_aabb_body_indices.insert( aabb_bdy_map_itr, std::make_pair( aabbs.size() - 1, TeleportedBall{ bdy_idx, prtl_idx, intersecting_plane_index } ) );
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

  // Create constraints for balls that actually overlap
  for( const std::pair<unsigned,unsigned>& possible_overlap_pair : possible_overlaps )
  {
    const bool first_teleported{ possible_overlap_pair.first >= nbodies };
    const bool second_teleported{ possible_overlap_pair.second >= nbodies };

    // If neither ball in the current collision was teleported
    if( !first_teleported && !second_teleported )
    {
      // TODO: Abstract this out like the other simulation codes
      // We can run standard narrow phase
      if( BallBallConstraint::isActive( possible_overlap_pair.first, possible_overlap_pair.second, q1, m_state.r() ) )
      {
        active_set.emplace_back( new BallBallConstraint{ possible_overlap_pair.first, possible_overlap_pair.second, q0, m_state.r()( possible_overlap_pair.first ), m_state.r()( possible_overlap_pair.second ), false } );
      }
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
        const std::map<unsigned,TeleportedBall>::const_iterator map_itr{ teleported_aabb_body_indices.find( possible_overlap_pair.first ) };
        assert( map_itr != teleported_aabb_body_indices.end() );
        bdy_idx_0 = map_itr->second.bodyIndex();
        assert( bdy_idx_0 < nbodies );
        prtl_idx_0 = map_itr->second.portalIndex();
        assert( prtl_idx_0 < m_state.planarPortals().size() );
        prtl_plane_0 = map_itr->second.planeIndex();
      }
      if( second_teleported )
      {
        const std::map<unsigned,TeleportedBall>::const_iterator map_itr{ teleported_aabb_body_indices.find( possible_overlap_pair.second ) };
        assert( map_itr != teleported_aabb_body_indices.end() );
        bdy_idx_1 = map_itr->second.bodyIndex();
        assert( bdy_idx_1 < nbodies );
        prtl_idx_1 = map_itr->second.portalIndex();
        assert( prtl_idx_1 < m_state.planarPortals().size() );
        prtl_plane_1 = map_itr->second.planeIndex();
      }

      // Check if the collision will be detected in the unteleported state
      if( first_teleported && second_teleported )
      {
        if( BallBallConstraint::isActive( bdy_idx_0, bdy_idx_1, q1, m_state.r() ) )
        {
          #ifndef NDEBUG
          duplicate_indices.push_back( std::make_pair( bdy_idx_0, bdy_idx_1 ) );
          #endif
          continue;
        }
      }

      // TODO: Merge this into generateTeleportedBallBallCollisions as in rb2d implementation
      // Check if the collision actually happens
      const TeleportedCollision possible_collision{ bdy_idx_0, bdy_idx_1, prtl_idx_0,  prtl_idx_1, prtl_plane_0, prtl_plane_1 };
      if( teleportedBallBallCollisionHappens( q1, possible_collision ) )
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
    assert( teleported_collision.bodyIndex0() < nbodies ); assert( teleported_collision.bodyIndex1() < nbodies );
    assert( teleported_collision.bodyIndex0() != teleported_collision.bodyIndex1( ) );
    generateTeleportedBallBallCollision( q0, q1, m_state.r(), teleported_collision, active_set );
  }

  #ifndef NDEBUG
  // Do an all pairs check for duplicates
  const unsigned ncons{ static_cast<unsigned>( active_set.size() ) };
  for( unsigned con_idx_0 = 0; con_idx_0 < ncons; ++con_idx_0 )
  {
    std::pair<int,int> indices0;
    active_set[con_idx_0]->getBodyIndices( indices0 );
    const int idx_0a{ std::min( indices0.first, indices0.second ) };
    const int idx_1a{ std::max( indices0.first, indices0.second ) };
    for( unsigned con_idx_1 = con_idx_0 + 1; con_idx_1 < ncons; ++con_idx_1 )
    {
      std::pair<int,int> indices1;
      active_set[con_idx_1]->getBodyIndices( indices1 );
      const int idx_0b{ std::min( indices1.first, indices1.second ) };
      const int idx_1b{ std::max( indices1.first, indices1.second ) };
      assert( !( ( idx_0a == idx_0b ) && ( idx_1a == idx_1b ) ) );
    }
  }
  #endif
}

void Ball2DSim::getTeleportedBallBallCenters( const VectorXs& q, const TeleportedCollision& teleported_collision, Vector2s& x0, Vector2s& x1 ) const
{
  assert( q.size() % 2 == 0 );

  assert( 2 * teleported_collision.bodyIndex0() + 1 < q.size() );
  x0 = q.segment<2>( 2 * teleported_collision.bodyIndex0() );
  // If the first object was teleported
  if( teleported_collision.portalIndex0() != std::numeric_limits<unsigned>::max() )
  {
    if( teleported_collision.plane0() == 0 )
    {
      m_state.planarPortals()[teleported_collision.portalIndex0()].teleportPointThroughPlaneA( q.segment<2>( 2 * teleported_collision.bodyIndex0() ), x0 );
    }
    else
    {
      m_state.planarPortals()[teleported_collision.portalIndex0()].teleportPointThroughPlaneB( q.segment<2>( 2 * teleported_collision.bodyIndex0() ), x0 );
    }
  }

  assert( 2 * teleported_collision.bodyIndex1() + 1 < q.size() );
  x1 = q.segment<2>( 2 * teleported_collision.bodyIndex1() );
  // If the second object was teleported
  if( teleported_collision.portalIndex1() != std::numeric_limits<unsigned>::max() )
  {
    if( teleported_collision.plane1() == 0 )
    {
      m_state.planarPortals()[teleported_collision.portalIndex1()].teleportPointThroughPlaneA( q.segment<2>( 2 * teleported_collision.bodyIndex1() ), x1 );
    }
    else
    {
      m_state.planarPortals()[teleported_collision.portalIndex1()].teleportPointThroughPlaneB( q.segment<2>( 2 * teleported_collision.bodyIndex1() ), x1 );
    }
  }
}

bool Ball2DSim::teleportedBallBallCollisionHappens( const VectorXs& q, const TeleportedCollision& teleported_collision ) const
{
  assert( q.size() % 2 == 0 );
  Vector2s x0;
  Vector2s x1;
  getTeleportedBallBallCenters( q, teleported_collision, x0, x1 );
  return BallBallConstraint::isActive( x0, x1, m_state.r()( teleported_collision.bodyIndex0() ), m_state.r()( teleported_collision.bodyIndex1() ) );
}

void Ball2DSim::generateTeleportedBallBallCollision( const VectorXs& q0, const VectorXs& q1, const VectorXs& r, const TeleportedCollision& teleported_collision, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  assert( q0.size() % 2 == 0 ); assert( q0.size() == q1.size() );

  // Get the center of mass of each body after the teleportation using start of step state
  Vector2s x0;
  Vector2s x1;
  getTeleportedBallBallCenters( q0, teleported_collision, x0, x1 );
  const scalar ri{ r( teleported_collision.bodyIndex0() ) };
  const scalar rj{ r( teleported_collision.bodyIndex1() ) };

  // At least one of the bodies must have been teleported
  assert( teleported_collision.portalIndex0() != std::numeric_limits<unsigned>::max() || teleported_collision.portalIndex1() != std::numeric_limits<unsigned>::max() );

  // Determine whether the first body was teleported
  bool portal0_is_lees_edwards{ false };
  #ifndef NDEBUG
  bool first_was_teleported{ false };
  #endif
  if( teleported_collision.portalIndex0() != std::numeric_limits<unsigned>::max() )
  {
    #ifndef NDEBUG
    first_was_teleported = true;
    #endif
    assert( teleported_collision.portalIndex0() < m_state.planarPortals().size() );
    if( m_state.planarPortals()[teleported_collision.portalIndex0()].isLeesEdwards() )
    {
      portal0_is_lees_edwards = true;
    }
  }

  // Determine whether the second body was teleported
  bool portal1_is_lees_edwards{ false };
  #ifndef NDEBUG
  bool second_was_teleported{ false };
  #endif
  if( teleported_collision.portalIndex1() != std::numeric_limits<unsigned>::max() )
  {
    #ifndef NDEBUG
    second_was_teleported = true;
    #endif
    assert( teleported_collision.portalIndex1() < m_state.planarPortals().size() );
    if( m_state.planarPortals()[teleported_collision.portalIndex1()].isLeesEdwards() )
    {
      portal1_is_lees_edwards = true;
    }
  }

  assert( !( portal0_is_lees_edwards && portal1_is_lees_edwards ) );

  // If neither portal is Lees-Edwards
  if( !portal0_is_lees_edwards && !portal1_is_lees_edwards )
  {
    active_set.emplace_back( new BallBallConstraint{ teleported_collision.bodyIndex0(), teleported_collision.bodyIndex1(), x0, x1, ri, rj, true } );
  }
  // Otherwise, there is a relative velocity contribution from the Lees-Edwards boundary condition
  else
  {
    Vector2s kinematic_kick;
    assert( portal0_is_lees_edwards != portal1_is_lees_edwards );
    if( portal1_is_lees_edwards )
    {
      assert( second_was_teleported );
      // N.B. q1 because collision detection was performed with q1
      kinematic_kick = m_state.planarPortals()[teleported_collision.portalIndex1()].getKinematicVelocityOfBall( q1.segment<2>( 2 * teleported_collision.bodyIndex1() ), rj );
    }
    else // portal0_is_lees_edwards
    {
      assert( first_was_teleported );
      // N.B. q1 because collision detection was performed with q1
      kinematic_kick = -m_state.planarPortals()[teleported_collision.portalIndex0()].getKinematicVelocityOfBall( q1.segment<2>( 2 * teleported_collision.bodyIndex0() ), ri );
    }
    active_set.emplace_back( new KinematicKickBallBallConstraint{ teleported_collision.bodyIndex0(), teleported_collision.bodyIndex1(), x0, x1, ri, rj, kinematic_kick, true } );
  }
}

void Ball2DSim::computeBallDrumActiveSetAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  assert( q0.size() == q1.size() ); assert( q0.size() % 2 == 0 ); assert( q0.size() / 2 == m_state.r().size() );
  // Check all ball-drum pairs
  for( const StaticDrum& static_drum : m_state.staticDrums() )
  {
    for( unsigned ball_idx = 0; ball_idx < unsigned( m_state.r().size() ); ++ball_idx )
    {
      if( StaticDrumConstraint::isActive( ball_idx, q1, m_state.r(), static_drum.x(), static_drum.r() ) )
      {
        active_set.emplace_back( std::unique_ptr<Constraint>( new StaticDrumConstraint{ ball_idx, q0, m_state.r()( ball_idx ), static_drum.x(), Utilities::index( m_state.staticDrums(), static_drum ) } ) );
      }
    }
  }
}

void Ball2DSim::computeBallPlaneActiveSetAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const
{
  assert( q0.size() == q1.size() ); assert( q0.size() % 2 == 0 ); assert( q0.size() / 2 == m_state.r().size() );
  // Check all ball-plane pairs
  for( const StaticPlane& static_plane : m_state.staticPlanes() )
  {
    for( unsigned ball_idx = 0; ball_idx < unsigned( m_state.r().size() ); ++ball_idx )
    {
      if( StaticPlaneConstraint::isActive( ball_idx, q1, m_state.r(), static_plane.x(), static_plane.n() ) )
      {
        active_set.push_back( std::unique_ptr<Constraint>( new StaticPlaneConstraint{ ball_idx, m_state.r()( ball_idx ), static_plane, Utilities::index( m_state.staticPlanes(), static_plane ) } ) );
      }
    }
  }
}

// TODO: 0 size matrices are not output due to a bug in an older version of HDF5
void Ball2DSim::writeBinaryState( HDF5File& output_file ) const
{
  if( m_state.q().size() != 0 )
  {
    // Output the ball positions
    output_file.writeMatrix( "", "q", m_state.q() );
    // Output the ball velocities
    output_file.writeMatrix( "", "v", m_state.v() );
    // Output the ball radii
    output_file.writeMatrix( "", "r", m_state.r() );
    // Output the mass
    {
      // Assemble the mass into a single flat vector like q, v, and r
      assert( m_state.M().nonZeros() == m_state.q().size() );
      const VectorXs m{ Eigen::Map<const VectorXs>( &m_state.M().data().value(0), m_state.q().size() ) };
      output_file.writeMatrix( "", "m", m );
    }
  }
  // Output the static planes
  if( !m_state.staticPlanes().empty() )
  {
    // Collect the centers of planes into a single matrix
    Matrix2Xsc static_plane_centers{ 2 , m_state.staticPlanes().size() };
    for( std::vector<StaticPlane>::size_type pln_idx = 0; pln_idx < m_state.staticPlanes().size(); ++pln_idx )
    {
      static_plane_centers.col( pln_idx ) = m_state.staticPlanes()[pln_idx].x();
    }
    // Save out the plane centers
    output_file.writeMatrix( "", "static_plane_centers", static_plane_centers );
  }
  if( !m_state.staticPlanes().empty() )
  {
    // Collect the normals of the planes into a single matrix
    Matrix2Xsc static_plane_normals{ 2 , m_state.staticPlanes().size() };
    for( std::vector<StaticPlane>::size_type pln_idx = 0; pln_idx < m_state.staticPlanes().size(); ++pln_idx )
    {
      static_plane_normals.col( pln_idx ) = m_state.staticPlanes()[pln_idx].n();
    }
    // Save out the plane normals
    output_file.writeMatrix( "", "static_plane_normals", static_plane_normals );
  }
}

void Ball2DSim::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  m_state.serialize( output_stream );
  m_constraint_cache.serialize( output_stream );
}

void Ball2DSim::deserialize( std::istream& input_stream )
{
  assert( input_stream.good() );
  m_state.deserialize( input_stream );
  m_constraint_cache.deserialize( input_stream );
}
