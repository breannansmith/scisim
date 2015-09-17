// StabilizedImpactFrictionMap.cpp
//
// Breannan Smith
// Last updated: 09/07/2015

#include "StabilizedImpactFrictionMap.h"

#include <iostream>

#include "scisim/Math/MathUtilities.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperatorUtilities.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"
#include "scisim/ConstrainedMaps/StaggeredProjections.h"
#include "scisim/Constraints/ConstrainedSystem.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ScriptingCallback.h"
#include "scisim/UnconstrainedMaps/FlowableSystem.h"

StabilizedImpactFrictionMap::StabilizedImpactFrictionMap( const scalar& abs_tol, const unsigned max_iters )
: m_f( VectorXs::Zero( 0 ) )
, m_abs_tol( abs_tol )
, m_max_iters( max_iters )
, m_write_constraint_forces( false )
, m_constraint_force_stream( nullptr )
{
  assert( m_abs_tol >= 0.0 );
}

StabilizedImpactFrictionMap::StabilizedImpactFrictionMap( std::istream& input_stream )
: m_f( MathUtilities::deserialize<VectorXs>( input_stream ) )
, m_abs_tol( Utilities::deserialize<scalar>( input_stream ) )
, m_max_iters( Utilities::deserialize<unsigned>( input_stream ) )
, m_write_constraint_forces( Utilities::deserialize<bool>( input_stream ) )
, m_constraint_force_stream( nullptr )
{
  assert( m_abs_tol >= 0.0 );
}

StabilizedImpactFrictionMap::~StabilizedImpactFrictionMap()
{}

void StabilizedImpactFrictionMap::flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 )
{
  std::cerr << "Bring StabilizedImpactFrictionMap back up and running" << std::endl;
  std::exit( EXIT_FAILURE );
//  // TODO: Sanity check input sizes
//
//  // Check if the given operators are currently supported
//  if( !ImpactFrictionMap::impactOperatorSupported( imap.getName() ) )
//  {
//    std::cerr << "Error, impact operator " << imap.getName() << " not supported in StabilizedImpactFrictionMap::flow" << std::endl;
//    std::exit( EXIT_FAILURE );
//  }
//  if( !ImpactFrictionMap::frictionOperatorSupported( fmap.getName() ) )
//  {
//    std::cerr << "Error, friction operator " << fmap.getName() << " not supported in StabilizedImpactFrictionMap::flow" << std::endl;
//    std::exit( EXIT_FAILURE );
//  }
//
//  // Initialize the friction impulse cache
//  if( m_f.size() == 0 )
//  {
//    m_f = VectorXs::Zero( v0.size() );
//  }
//
//  // Compute an unconstrained predictor step
//  umap.flow( q0, v0, fsys, iteration, dt, q1, v1 );
//
//  // Using the configuration at the predictor step, compute the set of active constraints
//  std::vector<std::unique_ptr<Constraint>> active_set;
//  csys.computeActiveSet( q0, q1, active_set );
//
//  // If there are no active constraints, there is no need to perform collision response
//  if( active_set.empty() )
//  {
//    if( m_write_constraint_forces )
//    {
//      exportConstraintForcesToBinary( fsys.ambientSpaceDimensions(), q0, active_set, SparseMatrixsc(), VectorXs::Zero(0), SparseMatrixsc(), VectorXs::Zero(0), dt );
//    }
//    m_write_constraint_forces = false;
//    m_constraint_force_stream = nullptr;
//    return;
//  }
//
//  const unsigned ncollisions = active_set.size();
//
//  // Set the coefficients of friction to the default
//  VectorXs mu = VectorXs::Constant( ncollisions, mu_default );
//  // If scripting is enabled, use the scripted version
//  call_back.frictionCoefficientCallback( active_set, mu );
//  assert( ( mu.array() >= 0.0 ).all() );
//
//  // Coefficients of restitution
//  VectorXs CoR = VectorXs::Constant( ncollisions, CoR_default );
//  // If scripting is enabled, use the scripted version
//  call_back.restitutionCoefficientCallback( active_set, CoR );
//  assert( ( CoR.array() >= 0.0 ).all() ); assert( ( CoR.array() <= 1.0 ).all() );
//
//  // TODO: Warm starting from previous timestep goes here
//  // Normal impulse magnitudes
//  VectorXs alpha = VectorXs::Zero( ncollisions );
//  // Friction impulses magnitudes
//  VectorXs beta = VectorXs::Zero( fmap.numFrictionImpulsesPerNormal() * ncollisions );
//  // Disk constraint multipliers
//  VectorXs lambda = VectorXs::Zero( ncollisions );
//
//  // TODO: Same type of interface for gdotN and gdotD
//  // Generalized normal basis
//  SparseMatrixsc N( fsys.Minv().cols(), ncollisions );
//  ImpactOperatorUtilities::computeN( fsys, active_set, q0, N );
//  // Project the kinematically scripted object's velocity onto the constraint set
//  VectorXs gdotN;
//  ImpactOperatorUtilities::evalKinematicRelativeVelocityN( q0, active_set, gdotN );
//
//  // Evaluate the friction basis and project the kinematically scripted object's velocity onto the friction basis
//  SparseMatrixsc D( v0.size(), beta.size() );
//  VectorXs gdotD( beta.size() );
//  fmap.formGeneralizedFrictionBasis( q0, v0, active_set, D, gdotD );
//
//  // Compute the initial momentum and angular momentum
//  #ifndef NDEBUG
//  const bool momentum_should_be_conserved = constraintSetShouldConserveMomentum( active_set );
//  VectorXs p0;
//  if( momentum_should_be_conserved ) fsys.computeMomentum( v0, p0 );
//  const bool angular_momentum_should_be_conserved = constraintSetShouldConserveAngularMomentum( active_set );
//  VectorXs L0;
//  if( angular_momentum_should_be_conserved ) fsys.computeAngularMomentum( v0, L0 );
//  #endif
//
//  // Perform the coupled impact/friction solve
//  {
//    bool solve_succeeded;
//    scalar error;
//    StaggeredProjections::solve( fsys.M(), fsys.Minv(), N, gdotN, CoR, D, gdotD, mu, q0, v0, active_set, m_max_iters, m_abs_tol, m_internal_warm_start_alpha, m_internal_warm_start_beta, imap, fmap, m_f, alpha, beta, solve_succeeded, error );
//    assert( ( m_f - D * beta ).lpNorm<Eigen::Infinity>() == 0.0 );
//    assert( error >= 0.0 );
//    if( !solve_succeeded )
//    {
//      std::cerr << "Warning, coupled impact/friction solve exceeded max iterations " << m_max_iters;
//      std::cerr << " with absolute error " << error << " stepping to time " << iteration * dt << std::endl;
//    }
//  }
//
//  v1 = v0 + fsys.Minv() * ( N * alpha + m_f );
//
//  // Verify that momentum and angular momentum are conserved
//  #ifndef NDEBUG
//  if( momentum_should_be_conserved )
//  {
//    VectorXs p1;
//    if( momentum_should_be_conserved ) fsys.computeMomentum( v1, p1 );
//    assert( ( p0 - p1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
//  }
//  if( angular_momentum_should_be_conserved )
//  {
//    VectorXs L1;
//    if( angular_momentum_should_be_conserved ) fsys.computeAngularMomentum( v1, L1 );
//    assert( ( L0 - L1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
//  }
//  #endif
//
//  // TODO: Derive bounds for gdotN and gdotD != 0.0
//  // Verify that the total energy decreased
//  if( ( gdotN.array() == 0.0 ).all() && ( gdotD.array() == 0.0 ).all() )
//  {
//    const scalar T_init = v0.dot( fsys.M() * v0 );
//    const scalar T_final = v1.dot( fsys.M() * v1 );
//    if( T_final > T_init + 1.0e-6 )
//    {
//      std::cerr << "WARNING, energy increase detected in StabilizedImpactFrictionMap: " << T_final - T_init << std::endl;
//    }
//  }
//
//  // Sanity check: no impulses should apply to kinematic geometry
//  assert( ImpactFrictionMap::noImpulsesToKinematicGeometry( fsys, N, alpha, D, beta, v0 ) );
//
//  // Export constraint forces, if requested
//  if( m_write_constraint_forces )
//  {
//    exportConstraintForcesToBinary( fsys.ambientSpaceDimensions(), q0, active_set, N, alpha, D, beta, dt );
//  }
//  m_write_constraint_forces = false;
//  m_constraint_force_stream = nullptr;
//
//  // TODO: unique_ptr for constraints
//  active_set.clear();
//
//  umap.linearInertialConfigurationUpdate( q0, v1, dt, q1 );
}

void StabilizedImpactFrictionMap::resetCachedData()
{
  m_f = VectorXs::Zero( 0 );
}

void StabilizedImpactFrictionMap::exportConstraintForcesToBinary( const unsigned ambient_space_dims, const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const SparseMatrixsc& N, const VectorXs& alpha, const SparseMatrixsc& D, const VectorXs& beta, const scalar& dt )
{
  assert( m_write_constraint_forces );
  assert( m_constraint_force_stream != nullptr );
  std::cerr << "StabilizedImpactFrictionMap::exportConstraintForcesToBinary" << std::endl;
  std::exit( EXIT_FAILURE );
  //ImpactFrictionMap::exportConstraintForcesToBinaryFile( ambient_space_dims, q, constraints, N, alpha, D, beta, dt, *m_constraint_force_stream );
}

// TODO: This constructor should use the utility functions to serialize
void StabilizedImpactFrictionMap::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );

  MathUtilities::serialize( m_f, output_stream );
  output_stream.write( (char*) &m_abs_tol, sizeof(scalar) );
  output_stream.write( (char*) &m_max_iters, sizeof(unsigned) );

  output_stream.write( (char*) &m_write_constraint_forces, sizeof(bool) );
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
