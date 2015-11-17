// StabilizedImpactFrictionMap.cpp
//
// Breannan Smith
// Last updated: 11/16/2015

#include "StabilizedImpactFrictionMap.h"

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

StabilizedImpactFrictionMap::StabilizedImpactFrictionMap( const scalar& abs_tol, const unsigned max_iters, const bool external_warm_start_alpha, const bool external_warm_start_beta )
: m_f( VectorXs::Zero( 0 ) )
, m_abs_tol( abs_tol )
, m_max_iters( max_iters )
, m_external_warm_start_alpha( external_warm_start_alpha )
, m_external_warm_start_beta( external_warm_start_beta )
, m_write_constraint_forces( false )
, m_constraint_force_stream( nullptr )
{
  assert( m_abs_tol >= 0.0 );
}

StabilizedImpactFrictionMap::StabilizedImpactFrictionMap( std::istream& input_stream )
: m_f( MathUtilities::deserialize<VectorXs>( input_stream ) )
, m_abs_tol( Utilities::deserialize<scalar>( input_stream ) )
, m_max_iters( Utilities::deserialize<unsigned>( input_stream ) )
, m_external_warm_start_alpha( Utilities::deserialize<bool>( input_stream ) )
, m_external_warm_start_beta( Utilities::deserialize<bool>( input_stream ) )
, m_write_constraint_forces( false )
, m_constraint_force_stream( nullptr )
{
  assert( m_abs_tol >= 0.0 );
}

void StabilizedImpactFrictionMap::flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 )
{
  // TODO: Sanity check input sizes

  // Initialize the friction impulse cache
  if( m_f.size() == 0 )
  {
    m_f = VectorXs::Zero( v0.size() );
  }

  // Compute an unconstrained predictor step
  umap.flow( q0, v0, fsys, iteration, dt, q1, v1 );

  // Using the configuration at the predictor step, compute the set of active constraints
  std::vector<std::unique_ptr<Constraint>> active_set;
  csys.computeActiveSet( q0, q1, active_set );

  // If there are no active constraints, there is no need to perform collision response
  if( active_set.empty() )
  {
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
  // NB: v1, not v0, so linear samples are aligned with the incoming velocity
  csys.computeContactBases( q0, v1, active_set, contact_bases );
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

  // TODO: Get warm starting working again
  alpha.setZero();
  beta.setZero();

  // Compute the initial momentum and angular momentum
  #ifndef NDEBUG
  const bool momentum_should_be_conserved{ constraintSetShouldConserveMomentum( active_set ) };
  VectorXs p0;
  if( momentum_should_be_conserved ) { fsys.computeMomentum( v1, p0 ); }
  const bool angular_momentum_should_be_conserved{ constraintSetShouldConserveAngularMomentum( active_set ) };
  VectorXs L0;
  if( angular_momentum_should_be_conserved ) { fsys.computeAngularMomentum( v1, L0 ); }
  #endif

  // Perform the coupled impact/friction solve
  {
    scalar error;
    bool solve_succeeded;
    VectorXs v2{ v1.size() };
    friction_solver.solve( iteration, dt, fsys, fsys.M(), fsys.Minv(), CoR, mu, q0, v1, active_set, contact_bases, m_max_iters, m_abs_tol, m_f, alpha, beta, v2, solve_succeeded, error );
    //std::cout << "alpha: " << alpha.transpose() << std::endl;
    //std::cout << "beta: " << beta.transpose() << std::endl;
    assert( error >= 0.0 );
    if( !solve_succeeded )
    {
      std::cerr << "Warning, coupled impact/friction solve exceeded max iterations " << m_max_iters;
      std::cerr << " with absolute error " << error << " stepping to time " << iteration * dt << std::endl;
    }
    v1.swap( v2 );
  }

  // Verify that momentum and angular momentum are conserved
  #ifndef NDEBUG
  if( momentum_should_be_conserved )
  {
    VectorXs p1;
    if( momentum_should_be_conserved ) { fsys.computeMomentum( v1, p1 ); }
    assert( ( p0 - p1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  }
  if( angular_momentum_should_be_conserved )
  {
    VectorXs L1;
    if( angular_momentum_should_be_conserved ) { fsys.computeAngularMomentum( v1, L1 ); }
    assert( ( L0 - L1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  }
  #endif

  // TODO: Derive bounds for gdotN and gdotD != 0.0
  // Verify that the total energy decreased
  //if( ( gdotN.array() == 0.0 ).all() && ( gdotD.array() == 0.0 ).all() )
  //{
  //  const scalar T_init = v0.dot( fsys.M() * v0 );
  //  const scalar T_final = v1.dot( fsys.M() * v1 );
  //  if( T_final > T_init + 1.0e-6 )
  //  {
  //    std::cerr << "WARNING, energy increase detected in StabilizedImpactFrictionMap: " << T_final - T_init << std::endl;
  //  }
  //}

  // Sanity check: no impulses should apply to kinematic geometry
  //assert( ImpactFrictionMap::noImpulsesToKinematicGeometry( fsys, N, alpha, D, beta, v0 ) );

  // Export constraint forces, if requested
  if( m_write_constraint_forces )
  {
    exportConstraintForcesToBinary( q0, active_set, contact_bases, alpha, beta, dt );
  }
  m_write_constraint_forces = false;
  m_constraint_force_stream = nullptr;

  active_set.clear();

  fsys.linearInertialConfigurationUpdate( q0, v1, dt, q1 );
}

void StabilizedImpactFrictionMap::resetCachedData()
{
  m_f = VectorXs::Zero( 0 );
}

void StabilizedImpactFrictionMap::exportConstraintForcesToBinary( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& contact_bases, const VectorXs& alpha, const VectorXs& beta, const scalar& dt )
{
  assert( m_write_constraint_forces );
  assert( m_constraint_force_stream != nullptr );
  ImpactFrictionMap::exportConstraintForcesToBinaryFile( q, constraints, contact_bases, alpha, beta, dt, *m_constraint_force_stream );
}

void StabilizedImpactFrictionMap::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  MathUtilities::serialize( m_f, output_stream );
  Utilities::serializeBuiltInType( m_abs_tol, output_stream );
  Utilities::serializeBuiltInType( m_max_iters, output_stream );
  Utilities::serializeBuiltInType( m_external_warm_start_alpha, output_stream );
  Utilities::serializeBuiltInType( m_external_warm_start_beta, output_stream );
  assert( m_write_constraint_forces == false );
  assert( m_constraint_force_stream == nullptr );
}

std::string StabilizedImpactFrictionMap::name() const
{
  return "stabilized_impact_friction_map";
}

void StabilizedImpactFrictionMap::exportForcesNextStep( HDF5File& output_file )
{
  m_write_constraint_forces = true;
  m_constraint_force_stream = &output_file;
}
