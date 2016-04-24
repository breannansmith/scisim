// GeometricImpactFrictionMap.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "GeometricImpactFrictionMap.h"

#include <memory>
#include <iostream>

#include "scisim/Math/MathUtilities.h"
#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/Constraints/Constraint.h"
#include "scisim/Constraints/ConstrainedSystem.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ScriptingCallback.h"
#include "scisim/UnconstrainedMaps/FlowableSystem.h"
#include "scisim/Utilities.h"

GeometricImpactFrictionMap::GeometricImpactFrictionMap( const scalar& abs_tol, const unsigned max_iters, const ImpulsesToCache impulses_to_cache )
: m_f( VectorXs::Zero( 0 ) )
, m_abs_tol( abs_tol )
, m_max_iters( max_iters )
, m_impulses_to_cache( impulses_to_cache )
, m_write_constraint_forces( false )
, m_constraint_force_stream( nullptr )
{
  assert( m_abs_tol >= 0.0 );
}

GeometricImpactFrictionMap::GeometricImpactFrictionMap( std::istream& input_stream )
: m_f( MathUtilities::deserialize<VectorXs>( input_stream ) )
, m_abs_tol( Utilities::deserialize<scalar>( input_stream ) )
, m_max_iters( Utilities::deserialize<unsigned>( input_stream ) )
, m_impulses_to_cache( Utilities::deserialize<ImpulsesToCache>( input_stream ) )
, m_write_constraint_forces( false )
, m_constraint_force_stream( nullptr )
{
  assert( m_abs_tol >= 0.0 );
}

//void GeometricImpactFrictionMap::initializeImpulses( const std::vector<std::unique_ptr<Constraint>>& active_set, const VectorXs& q0, const SparseMatrixsc& D, ConstrainedSystem& csys, const int num_impulses_per_normal, const int ambient_space_dims, VectorXs& alpha, VectorXs& beta )
//{
//  // No warm start, initialize from 0
//  if( !m_external_warm_start_alpha && !m_external_warm_start_beta )
//  {
//    alpha.setZero();
//    beta.setZero();
//  }
//  // Warm start alpha, 1 scalar to read in
//  else if( m_external_warm_start_alpha && !m_external_warm_start_beta )
//  {
//    std::cerr << "Code up decache 1, 0" << std::endl;
//    std::exit( EXIT_FAILURE );
//  }
//  // Warm start beta, 3 scalars to read in
//  else if( !m_external_warm_start_alpha && m_external_warm_start_beta )
//  {
//    std::cerr << "Code up decache 0, 1" << std::endl;
//    std::exit( EXIT_FAILURE );
//  }
//  // Warm start alpha and beta, 4 scalars to read in
//  else if( m_external_warm_start_alpha && m_external_warm_start_beta )
//  {
//    std::cerr << "Update third case to work again" << std::endl;
//    std::exit( EXIT_FAILURE );
//    //// alpha -- N -- D beta -- lambda
//    //const int num_cached_dofs = 2 * ambient_space_dims + 2;
//    //unsigned col_num = 0;
//    //for( const std::unique_ptr<Constraint>& constraint : active_set )
//    //{
//    //VectorXs cached_impulse( num_cached_dofs );
//    //csys.getCachedConstraintImpulse( *constraint, cached_impulse );
//    //if( ( cached_impulse.array() == 0.0 ).all() )
//    //{
//    //  alpha( col_num ) = 0.0;
//    //  beta.segment( num_impulses_per_normal * col_num, num_impulses_per_normal ).setZero();
//    //  ++col_num;
//    //  continue;
//    //}
//    //// Extract the cached contact normal
//    //const VectorXs n0 = cached_impulse.segment( 1, ambient_space_dims );
//    //assert( fabs( n0.norm() - 1.0 ) <= 1.0e-6 );
//    //// Extract the cached friction force
//    //const VectorXs f0 = cached_impulse.segment( ambient_space_dims + 1, ambient_space_dims );
//    //// Grab the new contact normal
//    //const VectorXs n1 = constraint->computeWorldSpaceContactNormal( q0 );
//    //assert( fabs( n1.norm() - 1.0 ) <= 1.0e-6 );
//    //// Parallel transport friction force from last frame to current frame
//    //const VectorXs f1 = MathUtilities::parallelTransport( n0, n1, f0 );
//    //assert( fabs( f0.norm() - f1.norm() ) <= 1.0e-6 );
//    //// Project the transported friction force on the current friction basis
//    //const VectorXs beta0 = constraint->projectOnFrictionBasis( q0, f1 );
//    //assert( beta0.size() == num_impulses_per_normal );
//    //// If the basis spans the tangent plane, should recover the same force
//    //#ifndef NDEBUG
//    //if( constraint->basisSpansTangentPlane() )
//    //{
//    //  //const VectorXs computed_force = constraint->computeFrictionBasis( q0, v0 ) * beta0;
//    //  //assert( ( computed_force - f1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
//    //  std::cerr << "Re-implement test code in GeometricImpactFrictionMap::initializeImpulses" << std::endl;
//    //  std::exit( EXIT_FAILURE );
//    //}
//    //#endif
//    //// Copy over the cached impulse
//    //alpha( col_num ) = cached_impulse( 0 );
//    //assert( alpha( col_num ) >= 0.0 );
//    //beta.segment( num_impulses_per_normal * col_num, num_impulses_per_normal ) = beta0;
//    //
//    //++col_num;
//    //}
//    //assert( col_num == active_set.size() );
//    //m_f = D * beta;
//  }
//  csys.clearConstraintCache();
//}

//void GeometricImpactFrictionMap::cacheImpulses( const std::vector<std::unique_ptr<Constraint>>& active_set, const VectorXs& q0, ConstrainedSystem& csys, const int num_impulses_per_normal, const int ambient_space_dims, const VectorXs& alpha, const VectorXs& beta )
//{
//  // if( !m_external_warm_start_alpha && !m_external_warm_start_beta )
//  if( m_external_warm_start_alpha && !m_external_warm_start_beta )
//  {
//    std::cerr << "Code up cache 1, 0" << std::endl;
//    std::exit( EXIT_FAILURE );
//  }
//  else if( !m_external_warm_start_alpha && m_external_warm_start_beta )
//  {
//    std::cerr << "Code up cache 0, 1" << std::endl;
//    std::exit( EXIT_FAILURE );
//  }
//  else if( m_external_warm_start_alpha && m_external_warm_start_beta )
//  {
//    std::cerr << "Bring third case in cache impulses up and running again" << std::endl;
//    std::exit( EXIT_FAILURE );
//    ////std::cout << "ALPHA: " << alpha.transpose() << std::endl;
//    ////std::cout << "BETA: " << beta.transpose() << std::endl;
//    //// alpha -- N -- D beta -- lambda
//    //const int num_cached_dofs = 2 * ambient_space_dims + 2;
//    //unsigned col_num = 0;
//    //for( const std::unique_ptr<Constraint>& constraint : active_set )
//    //{
//    //  VectorXs impulse_to_cache( num_cached_dofs );
//    //  // Normal impulse
//    //  impulse_to_cache( 0 ) = alpha( col_num );
//    //  // Contact normal in world space scaled by contact impulse
//    //  impulse_to_cache.segment( 1, ambient_space_dims ) = constraint->computeWorldSpaceContactNormal( q0 );
//    //  // Contact basis in world space scaled by friction impulse
//    //  //impulse_to_cache.segment( ambient_space_dims + 1, ambient_space_dims ) = constraint->computeWorldSpaceFrictionBasis( q0 ) * beta.segment( num_impulses_per_normal * col_num, num_impulses_per_normal );
//    //  std::cerr << "Bring code in GeometricImpactFrictionMap::cacheImpulses up and running again" << std::endl;
//    //  std::exit( EXIT_FAILURE );
//    //  //impulse_to_cache( impulse_to_cache.size() - 1 ) = lambda( col_num );
//    //  // Save out the constraint
//    //  csys.cacheConstraint( *constraint, impulse_to_cache );
//    //
//    //  // Doesn't work because of staggering!
//    //  //if( impulse_to_cache.segment<2>( 0 ).norm() == 0.0 )
//    //  //{
//    //  //  std::cout << "Time: " << iteration * dt << std::endl;
//    //  //  std::cout << "Bad beta: " << impulse_to_cache.segment<2>( 3 ).transpose() << std::endl;
//    //  //  std::cout << "    norm: " << impulse_to_cache.segment<2>( 3 ).norm() << std::endl;
//    //  //  assert( impulse_to_cache.segment<2>( 3 ).norm() <= 1.0e-9 );
//    //  //}
//    //
//    //  ++col_num;
//    //}
//    //assert( col_num == active_set.size() );
//  }
//}

static void initializeImpulses( const ImpulsesToCache cache_mode, const unsigned ambient_dims, const std::vector<std::unique_ptr<Constraint>>& active_set, ConstrainedSystem& csys, VectorXs& alpha, VectorXs& beta )
{
  switch( cache_mode )
  {
    case ImpulsesToCache::NONE:
      alpha.setZero();
      beta.setZero();
      assert( csys.constraintCacheEmpty() );
      break;
    case ImpulsesToCache::NORMAL:
    {
      unsigned col_num{ 0 };
      for( const std::unique_ptr<Constraint>& constraint : active_set )
      {
        VectorXs cached_impulse{ 1 };
        csys.getCachedConstraintImpulse( *constraint, cached_impulse );
        alpha( col_num++ ) = cached_impulse( 0 );
      }
      assert( col_num == active_set.size() );
      beta.setZero();
      csys.clearConstraintCache();
      break;
    }
    case ImpulsesToCache::NORMAL_AND_FRICTION:
    {
      if( ambient_dims == 2 )
      {
        unsigned col_num{ 0 };
        for( const std::unique_ptr<Constraint>& constraint : active_set )
        {
          VectorXs cached_impulse{ 2 };
          csys.getCachedConstraintImpulse( *constraint, cached_impulse );
          alpha( col_num ) = cached_impulse( 0 );
          beta( col_num++ ) = cached_impulse( 1 );
        }
        assert( col_num == active_set.size() );
      }
      else
      {
        std::cerr << "Decaching in " << ambient_dims << " space not yet supported." << std::endl;
        std::exit( EXIT_FAILURE );
      }
      csys.clearConstraintCache();
      break;
    }
  }
}

static void cacheImpulses( const ImpulsesToCache cache_mode, const unsigned ambient_dims, const std::vector<std::unique_ptr<Constraint>>& active_set, ConstrainedSystem& csys, const VectorXs& alpha, const VectorXs& beta )
{
  switch( cache_mode )
  {
    case ImpulsesToCache::NONE:
      assert( csys.constraintCacheEmpty() );
      break;
    case ImpulsesToCache::NORMAL:
    {
      assert( csys.constraintCacheEmpty() );
      unsigned col_num = 0;
      for( const std::unique_ptr<Constraint>& constraint : active_set )
      {
        VectorXs cached_impulse{ 1 };
        cached_impulse( 0 ) = alpha( col_num++ );
        csys.cacheConstraint( *constraint, cached_impulse );
      }
      assert( col_num == active_set.size() );
      break;
    }
    case ImpulsesToCache::NORMAL_AND_FRICTION:
      assert( csys.constraintCacheEmpty() );
      if( ambient_dims == 2 )
      {
        assert( alpha.size() == beta.size() );
        unsigned col_num = 0;
        for( const std::unique_ptr<Constraint>& constraint : active_set )
        {
          VectorXs cached_impulse{ 2 };
          cached_impulse( 0 ) = alpha( col_num );
          cached_impulse( 1 ) = beta( col_num++ );
          csys.cacheConstraint( *constraint, cached_impulse );
        }
        assert( col_num == active_set.size() );
      }
      else
      {
        std::cerr << "Caching in " << ambient_dims << " space not yet supported." << std::endl;
        std::exit( EXIT_FAILURE );
      }
      break;
  }
}


void GeometricImpactFrictionMap::flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 )
{
  // TODO: Sanity check input sizes

  // TODO: Do something more elegant when the number of bodies changes due to insertions and deletions, like registering the maps with the scripting callbacks so they can update the cache.
  // Initialize the friction impulse cache
  if( m_f.size() != v0.size() )
  {
    m_f = VectorXs::Zero( v0.size() );
  }

  // Compute an unconstrained predictor step
  umap.flow( q0, v0, fsys, iteration, dt, q1, v1 );

  // Using the configuration at the predictor step, compute the set of active constraints
  std::vector<std::unique_ptr<Constraint>> active_set;
  csys.computeActiveSet( q0, q1, v0, active_set );

  // If there are no active constraints, there is no need to perform collision response
  if( active_set.empty() )
  {
    csys.clearConstraintCache();
    if( m_write_constraint_forces )
    {
      exportConstraintForcesToBinary( q0, active_set, MatrixXXsc{ fsys.ambientSpaceDimensions(), 0 }, VectorXs::Zero(0), VectorXs::Zero(0), dt );
    }
    m_write_constraint_forces = false;
    m_constraint_force_stream = nullptr;
    return;
  }

  const unsigned ncollisions{ static_cast<unsigned>( active_set.size() ) };

  // Pre-compute the full contact basis
  MatrixXXsc contact_bases;
  csys.computeContactBases( q0, v0, active_set, contact_bases );
  assert( contact_bases.rows() == fsys.ambientSpaceDimensions() );
  assert( contact_bases.cols() == fsys.ambientSpaceDimensions() * ncollisions );

  // Set the coefficients of friction to the default
  VectorXs mu{ VectorXs::Constant( ncollisions, mu_default ) };
  // If scripting is enabled, use the scripted version
  call_back.frictionCoefficientCallback( active_set, mu );
  assert( ( mu.array() >= 0.0 ).all() );

  // Coefficients of restitution
  VectorXs CoR{ VectorXs::Constant( ncollisions, CoR_default ) };
  // If scripting is enabled, use the scripted version
  call_back.restitutionCoefficientCallback( active_set, CoR );
  assert( ( CoR.array() >= 0.0 ).all() ); assert( ( CoR.array() <= 1.0 ).all() );

  // Normal impulse magnitudes
  VectorXs alpha{ ncollisions };
  // Friction impulses magnitudes
  VectorXs beta{ friction_solver.numFrictionImpulsesPerNormal( fsys.ambientSpaceDimensions() ) * ncollisions };

  initializeImpulses( m_impulses_to_cache, fsys.ambientSpaceDimensions(), active_set, csys, alpha, beta );

  // Compute the initial momentum and angular momentum
  #ifndef NDEBUG
  const bool momentum_should_be_conserved{ constraintSetShouldConserveMomentum( active_set ) };
  VectorXs p0;
  if( momentum_should_be_conserved ) { fsys.computeMomentum( v0, p0 ); }
  const bool angular_momentum_should_be_conserved{ constraintSetShouldConserveAngularMomentum( active_set ) };
  VectorXs L0;
  if( angular_momentum_should_be_conserved ) { fsys.computeAngularMomentum( v0, L0 ); }
  #endif

  // Perform the coupled impact/friction solve
  VectorXs v2{ v0.size() };
  {
    scalar error;
    bool solve_succeeded;
    friction_solver.solve( iteration, dt, fsys, fsys.M(), fsys.Minv(), CoR, mu, q0, v0, active_set, contact_bases, m_max_iters, m_abs_tol, m_f, alpha, beta, v2, solve_succeeded, error );
    assert( error >= 0.0 );
    if( !solve_succeeded )
    {
      std::cerr << "Warning, coupled impact/friction solve exceeded max iterations " << m_max_iters;
      std::cerr << " with absolute error " << error << " stepping to time " << iteration * dt << std::endl;
    }
  }

  // Cache the impulses, verify the expected uncached impulses
  // Note: temporarily disabled test as it requires impulse caching support for all collision types
  //#ifndef NDEBUG
  //{
  //  cacheImpulses( ImpulsesToCache::NONE, fsys.ambientSpaceDimensions(), active_set, csys, alpha, beta );
  //  VectorXs alpha_test{ alpha.size() };
  //  VectorXs beta_test{ beta.size() };
  //  initializeImpulses( ImpulsesToCache::NONE, fsys.ambientSpaceDimensions(), active_set, csys, alpha_test, beta_test );
  //  assert( ( alpha_test.array() == 0.0 ).all() );
  //  assert( ( beta_test.array() == 0.0 ).all() );
  //}
  //{
  //  cacheImpulses( ImpulsesToCache::NORMAL, fsys.ambientSpaceDimensions(), active_set, csys, alpha, beta );
  //  VectorXs alpha_test{ alpha.size() };
  //  VectorXs beta_test{ beta.size() };
  //  initializeImpulses( ImpulsesToCache::NORMAL, fsys.ambientSpaceDimensions(), active_set, csys, alpha_test, beta_test );
  //  assert( ( alpha_test.array() == alpha.array() ).all() );
  //  assert( ( beta_test.array() == 0.0 ).all() );
  //}
  //{
  //  cacheImpulses( ImpulsesToCache::NORMAL_AND_FRICTION, fsys.ambientSpaceDimensions(), active_set, csys, alpha, beta );
  //  VectorXs alpha_test{ alpha.size() };
  //  VectorXs beta_test{ beta.size() };
  //  initializeImpulses( ImpulsesToCache::NORMAL_AND_FRICTION, fsys.ambientSpaceDimensions(), active_set, csys, alpha_test, beta_test );
  //  assert( ( alpha_test.array() == alpha.array() ).all() );
  //  assert( ( beta_test.array() == beta.array() ).all() );
  //}
  //#endif

  // For the special case mu == 0, friction should be zero
  #ifndef NDEBUG
  {
    const unsigned friction_impulses_per_collision{ friction_solver.numFrictionImpulsesPerNormal( fsys.ambientSpaceDimensions() ) };
    for( unsigned col_idx = 0; col_idx < ncollisions; ++col_idx )
    {
      if( mu(col_idx) == 0.0 )
      {
        assert( ( beta.segment( friction_impulses_per_collision, col_idx * friction_impulses_per_collision ).array() == 0.0 ).all() );
      }
    }
  }
  #endif

  // Verify that momentum and angular momentum are conserved
  #ifndef NDEBUG
  if( momentum_should_be_conserved )
  {
    VectorXs p1;
    if( momentum_should_be_conserved ) { fsys.computeMomentum( v2, p1 ); }
    assert( ( p0 - p1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  }
  if( angular_momentum_should_be_conserved )
  {
    VectorXs L1;
    if( angular_momentum_should_be_conserved ) { fsys.computeAngularMomentum( v2, L1 ); }
    assert( ( L0 - L1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  }
  #endif

  // TODO: Derive bounds for gdotN and gdotD != 0.0
  // Verify that the total energy decreased
  //if( ( gdotN.array() == 0.0 ).all() && ( gdotD.array() == 0.0 ).all() )
  //{
  //  const scalar T_init = v0.dot( fsys.M() * v0 );
  //  const scalar T_final = v2.dot( fsys.M() * v2 );
  //  if( T_final > T_init + 1.0e-6 )
  //  {
  //    std::cerr << "WARNING, energy increase detected in GeometricImpactFrictionMap: " << T_final - T_init << "     error: " << error << std::endl;
  //  }
  //}

  // Sanity check: no impulses should apply to kinematic geometry
  //assert( ImpactFrictionMap::noImpulsesToKinematicGeometry( fsys, N, alpha, D, beta, v0 ) );

  // Cache the constraints for warm starting
  cacheImpulses( m_impulses_to_cache, fsys.ambientSpaceDimensions(), active_set, csys, alpha, beta );

  // Export constraint forces, if requested
  if( m_write_constraint_forces )
  {
    exportConstraintForcesToBinary( q0, active_set, contact_bases, alpha, beta, dt );
  }
  m_write_constraint_forces = false;
  m_constraint_force_stream = nullptr;

  // Using the initial configuration and the new velocity, compute the final state
  umap.flow( q0, v2, fsys, iteration, dt, q1, v1 );
}

void GeometricImpactFrictionMap::resetCachedData()
{
  m_f = VectorXs::Zero( 0 );
}

void GeometricImpactFrictionMap::exportConstraintForcesToBinary( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& contact_bases, const VectorXs& alpha, const VectorXs& beta, const scalar& dt )
{
  assert( m_write_constraint_forces );
  assert( m_constraint_force_stream != nullptr );
  ImpactFrictionMap::exportConstraintForcesToBinaryFile( q, constraints, contact_bases, alpha, beta, dt, *m_constraint_force_stream );
}

void GeometricImpactFrictionMap::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  MathUtilities::serialize( m_f, output_stream );
  Utilities::serialize( m_abs_tol, output_stream );
  Utilities::serialize( m_max_iters, output_stream );
  Utilities::serialize( m_impulses_to_cache, output_stream );
  assert( m_write_constraint_forces == false );
  assert( m_constraint_force_stream == nullptr );
}

std::string GeometricImpactFrictionMap::name() const
{
  return "geometric_impact_friction_map";
}

void GeometricImpactFrictionMap::exportForcesNextStep( HDF5File& output_file )
{
  m_write_constraint_forces = true;
  m_constraint_force_stream = &output_file;
}
