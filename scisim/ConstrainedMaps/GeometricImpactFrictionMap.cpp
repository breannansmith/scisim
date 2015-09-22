// GeometricImpactFrictionMap.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "GeometricImpactFrictionMap.h"

#include <memory>
#include <iostream>

#include "scisim/Math/MathUtilities.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperatorUtilities.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"
#include "scisim/ConstrainedMaps/StaggeredProjections.h"
#include "scisim/Constraints/Constraint.h"
#include "scisim/Constraints/ConstrainedSystem.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ScriptingCallback.h"
#include "scisim/UnconstrainedMaps/FlowableSystem.h"
#include "scisim/Utilities.h"

GeometricImpactFrictionMap::GeometricImpactFrictionMap( const scalar& abs_tol, const unsigned max_iters, const bool external_warm_start_alpha, const bool external_warm_start_beta )
: m_use_staggered_projections( true )
, m_f( VectorXs::Zero( 0 ) )
, m_abs_tol( abs_tol )
, m_max_iters( max_iters )
, m_external_warm_start_alpha( external_warm_start_alpha )
, m_external_warm_start_beta( external_warm_start_beta )
, m_write_constraint_forces( false )
, m_constraint_force_stream( nullptr )
{
  assert( m_abs_tol >= 0.0 );
}

GeometricImpactFrictionMap::GeometricImpactFrictionMap( std::istream& input_stream )
: m_use_staggered_projections( Utilities::deserialize<bool>( input_stream ) )
, m_f( MathUtilities::deserialize<VectorXs>( input_stream ) )
, m_abs_tol( Utilities::deserialize<scalar>( input_stream ) )
, m_max_iters( Utilities::deserialize<unsigned>( input_stream ) )
, m_external_warm_start_alpha( Utilities::deserialize<bool>( input_stream ) )
, m_external_warm_start_beta( Utilities::deserialize<bool>( input_stream ) )
, m_write_constraint_forces( false )
, m_constraint_force_stream( nullptr )
{
  assert( m_abs_tol >= 0.0 );
}

void GeometricImpactFrictionMap::flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 )
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
      exportConstraintForcesToBinary( q0, active_set, MatrixXXsc( fsys.ambientSpaceDimensions(), 0 ), VectorXs::Zero(0), VectorXs::Zero(0), dt );
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

  //initializeImpulses( active_set, q0, D, csys, fmap.numFrictionImpulsesPerNormal(), fsys.ambientSpaceDimensions(), alpha, beta );

  // TODO: Get warm starting working again
  alpha.setZero();
  beta.setZero();

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

  //{
  //  VectorXs bogus_f( m_f.size() );
  //  VectorXs bogus_alpha( alpha.size() );
  //  VectorXs bogus_beta( 2 * alpha.size() );
  //  VectorXs bogus_v2( v2.size() );
  //  scalar bogus_error;
  //  bool bogus_success;
  //  Sobogus so_bogus;
  //  so_bogus.solve( iteration * dt, fsys, fsys.M(), fsys.Minv(), CoR, mu, q0, v0, active_set, contact_bases, 0, m_abs_tol, bogus_f, bogus_alpha, bogus_beta, bogus_v2, bogus_success, bogus_error );
  //  if( ( bogus_alpha - alpha ).lpNorm<Eigen::Infinity>() > 1.0e-6 || ( bogus_beta - beta ).lpNorm<Eigen::Infinity>() > 1.0e-6 ||
  //            ( v2 - bogus_v2 ).lpNorm<Eigen::Infinity>() > 1.0e-6 )
  //  {
  //    std::cout << "Failure time: " << iteration * dt << std::endl;
  //    //std::cout << "delta alpha: " << ( alpha - bogus_alpha ).lpNorm<Eigen::Infinity>() << std::endl;
  //    std::cout << "      alpha: " << alpha.transpose() << std::endl;
  //    std::cout << "bogus_alpha: " << bogus_alpha.transpose() << std::endl;
  //    std::cout << "      beta: " << beta.transpose() << std::endl;
  //    std::cout << "bogus_beta: " << bogus_beta.transpose() << std::endl;
  //    //std::cout << "    delta f: " << ( m_f - bogus_f ).lpNorm<Eigen::Infinity>() << std::endl;
  //    std::cout << "          f: " << m_f.transpose() << std::endl;
  //    std::cout << "    bogus_f: " << bogus_f.transpose() << std::endl;
  //    std::cout << "         v2: " << v2.transpose() << std::endl;
  //    std::cout << "   bogus_v2: " << bogus_v2.transpose() << std::endl;
  //    std::cout << "   delta_v2: " << ( v2 - bogus_v2 ).lpNorm<Eigen::Infinity>() << std::endl;
  //    std::cout << "      error: " << error << std::endl;
  //    std::cout << "bogus_error: " << bogus_error << std::endl;
  //  }
  //  assert( ( alpha - bogus_alpha ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  //  assert( ( beta - bogus_beta ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  //  assert( ( m_f - bogus_f ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  //  assert( ( v2 - bogus_v2 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  //}

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
  //cacheImpulses( active_set, q0, csys, fmap.numFrictionImpulsesPerNormal(), fsys.ambientSpaceDimensions(), alpha, beta );

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

//{
//    if( m_use_staggered_projections )
//    {
//      StaggeredProjections::solve( fsys.M(), fsys.Minv(), N, gdotN, CoR, D, gdotD, mu, q0, v0, active_set, m_max_iters, m_abs_tol, m_internal_warm_start_alpha, m_internal_warm_start_beta, imap, fmap, m_f, alpha, beta, solve_succeeded, error );
//    }
//    else
//    {
//      Sobogus::solve( fsys.M(), q0, v0, active_set, CoR, mu, m_abs_tol, m_f, alpha, beta, solve_succeeded, error );
//    }
//    assert( ( m_f - D * beta ).lpNorm<Eigen::Infinity>() <= 1.0e-10 );

//VectorXs bogus_lambda( mu.size() );
//recoverLambda( active_set, q0, v0, fsys.Minv(), N, bogus_alpha, D, bogus_beta, bogus_lambda );
//VectorXs sp_lambda( mu.size() );
//recoverLambda( active_set, q0, v0, fsys.Minv(), N, alpha, D, beta, sp_lambda );
//{
//const scalar sp_error = fabs( StaggeredProjections::globalObjective( active_set, fsys.Minv(), q0, v0, CoR, mu, N, alpha, D, beta, gdotN, gdotD ) );
//assert( sp_error == error );
//const scalar sb_error = fabs( StaggeredProjections::globalObjective( active_set, fsys.Minv(), q0, v0, CoR, mu, N, bogus_alpha, D, bogus_beta, gdotN, gdotD ) );
//std::cout << "bogus_alpha: " << bogus_alpha.transpose() << std::endl;
//std::cout << "stagg_alpha: " << alpha.transpose() << std::endl;
//std::cout << "delta_alpha: " << ( bogus_alpha - alpha ).transpose() << std::endl;
//std::cout << "bogus_beta: " << bogus_beta.transpose() << std::endl;
//std::cout << "stagg_beta: " << beta.transpose() << std::endl;
//std::cout << "bf: " << bogus_f.transpose() << std::endl;
//std::cout << "sf: " << m_f.transpose() << std::endl;
//std::cout << "bogus_error: " << bogus_error << std::endl;
//std::cout << "stagg_error: " << error << std::endl;
//}
//assert( ( bogus_alpha - alpha ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
//assert( ( bogus_beta - beta ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
//assert( ( bogus_lambda - sp_lambda ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
//assert( ( bogus_f - m_f ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
//}

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
  Utilities::serializeBuiltInType( m_use_staggered_projections, output_stream );
  MathUtilities::serialize( m_f, output_stream );
  Utilities::serializeBuiltInType( m_abs_tol, output_stream );
  Utilities::serializeBuiltInType( m_max_iters, output_stream );
  Utilities::serializeBuiltInType( m_external_warm_start_alpha, output_stream );
  Utilities::serializeBuiltInType( m_external_warm_start_beta, output_stream );
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
