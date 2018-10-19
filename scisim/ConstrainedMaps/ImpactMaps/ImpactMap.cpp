#include "ImpactMap.h"

#include <memory>

#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperatorUtilities.h"
#include "scisim/Constraints/ConstrainedSystem.h"
#include "scisim/Constraints/Constraint.h"
#include "scisim/UnconstrainedMaps/FlowableSystem.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ScriptingCallback.h"
#include "scisim/Utilities.h"
#include "ImpactOperator.h"

#ifdef USE_HDF5
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactSolution.h"
#endif

ImpactMap::ImpactMap( const bool warm_start )
: m_warm_start( warm_start )
#ifdef USE_HDF5
, m_write_constraint_forces( false )
, m_impact_solution( nullptr )
#endif
{}

ImpactMap::ImpactMap( std::istream& input_stream )
: m_warm_start( Utilities::deserialize<bool>( input_stream ) )
#ifdef USE_HDF5
, m_write_constraint_forces( false )
, m_impact_solution( nullptr )
#endif
{}

#ifndef NDEBUG
static bool constraintSetShouldConserveMomentum( const std::vector<std::unique_ptr<Constraint>>& cons )
{
  return std::all_of( std::cbegin( cons ), std::cend( cons ), [](const auto& c){ return c->conservesTranslationalMomentum(); } );
}

static bool constraintSetShouldConserveAngularMomentum( const std::vector<std::unique_ptr<Constraint>>& cons )
{
  return std::all_of( std::cbegin( cons ), std::cend( cons ), [](const auto& c){ return c->conservesAngularMomentumUnderImpact(); } );
}
#endif

void ImpactMap::flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, ImpactOperator& imap, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 )
{
  // Compute an unconstrained predictor step, save result into q1 and v1
  umap.flow( q0, v0, fsys, iteration, dt, q1, v1 );

  // Using the configuration at the predictor step, compute the set of active constraints.
  std::vector<std::unique_ptr<Constraint>> active_set;
  csys.computeActiveSet( q0, q1, v0, active_set );

  // If there are no active constraints, there is no need to perform collision response
  if( active_set.empty() )
  {
    csys.clearConstraintCache();
    #ifdef USE_HDF5
    if( m_write_constraint_forces )
    {
      assert( m_impact_solution != nullptr );
      m_impact_solution->setSolution( q0, active_set, MatrixXXsc{ fsys.ambientSpaceDimensions(), 0 }, VectorXs::Zero(0), dt );
    }
    m_write_constraint_forces = false;
    m_impact_solution = nullptr;
    #endif
    return;
  }

  const unsigned ncollisions{ static_cast<unsigned>( active_set.size() ) };

  VectorXs alpha{ ncollisions };
  VectorXs v2{ v0.size() };

  // If desired, read in previous values for warm starting
  if( m_warm_start )
  {
    unsigned col_num{ 0 };
    for( const std::unique_ptr<Constraint>& constraint : active_set )
    {
      VectorXs cached_impulse{ 1 };
      csys.getCachedConstraintImpulse( *constraint, cached_impulse );
      alpha( col_num++ ) = cached_impulse( 0 );
    }
    assert( col_num == ncollisions );
  }
  // Otherwise default to an initial guess of 0
  else
  {
    alpha.setZero();
  }
  csys.clearConstraintCache();

  // Coefficients of restitution
  VectorXs CoR{ VectorXs::Constant( ncollisions, CoR_default ) };
  // If scripting is enabled, use the scripted version
  call_back.restitutionCoefficientCallback( active_set, CoR );

  // Generalized normal basis
  SparseMatrixsc N{ fsys.Minv().cols(), SparseMatrixsc::Index( ncollisions ) };
  ImpactOperatorUtilities::computeN( fsys, active_set, q0, N );

  // Quadratic term in LCP QP
  const SparseMatrixsc Q{ N.transpose() * fsys.Minv() * N };

  // Evaluate the kinematic scripted object's velocity projected onto the constraint set
  VectorXs gdotN;
  ImpactOperatorUtilities::evalKinematicRelativeVelocityN( q0, active_set, gdotN );

  // Compute the initial momentum and angular momentum
  #ifndef NDEBUG
  const bool momentum_should_be_conserved{ constraintSetShouldConserveMomentum( active_set ) };
  VectorXs p0;
  if( momentum_should_be_conserved )
  {
    fsys.computeMomentum( v0, p0 );
  }
  const bool angular_momentum_should_be_conserved{ constraintSetShouldConserveAngularMomentum( active_set ) };
  VectorXs L0;
  if( angular_momentum_should_be_conserved )
  {
    fsys.computeAngularMomentum( v0, L0 );
  }
  #endif

  // Note: No friction, so initial velocity passed in twice
  imap.flow( active_set, fsys.M(), fsys.Minv(), q0, v0, v0, N, Q, gdotN, CoR, alpha );
  v2 = v0 + fsys.Minv() * N * alpha;

  // Verify that momentum and angular momentum are conserved
  #ifndef NDEBUG
  if( momentum_should_be_conserved )
  {
    VectorXs p1;
    if( momentum_should_be_conserved )
    {
      fsys.computeMomentum( v2, p1 );
    }
    assert( ( p0 - p1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  }
  if( angular_momentum_should_be_conserved )
  {
    VectorXs L1;
    if( angular_momentum_should_be_conserved )
    {
      fsys.computeAngularMomentum( v2, L1 );
    }
    assert( ( L0 - L1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  }
  #endif

  // Cache the constraints for warm starting
  assert( csys.constraintCacheEmpty() );
  if( m_warm_start )
  {
    unsigned col_num = 0;
    for( const std::unique_ptr<Constraint>& constraint : active_set )
    {
      VectorXs cached_impulse( 1 );
      cached_impulse( 0 ) = alpha( col_num++ );
      csys.cacheConstraint( *constraint, cached_impulse );
    }
  }

  // Export constraint forces, if requested
  #ifdef USE_HDF5
  if( m_write_constraint_forces )
  {
    assert( m_impact_solution != nullptr );
    MatrixXXsc impact_bases;
    csys.computeImpactBases( q0, active_set, impact_bases );
    m_impact_solution->setSolution( q0, active_set, impact_bases, alpha, dt );
  }
  m_write_constraint_forces = false;
  m_impact_solution = nullptr;
  #endif

  active_set.clear();

  // Using the initial configuration and the new velocity, compute the final state
  umap.flow( q0, v2, fsys, iteration, dt, q1, v1 );
}

void ImpactMap::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  Utilities::serialize( m_warm_start, output_stream );
  #ifdef USE_HDF5
  assert( m_write_constraint_forces == false );
  assert( m_impact_solution == nullptr );
  #endif
}

#ifdef USE_HDF5
void ImpactMap::exportForcesNextStep( ImpactSolution& impact_solution )
{
  m_write_constraint_forces = true;
  m_impact_solution = &impact_solution;
}
#endif

std::unique_ptr<ImpactMap> ImpactMap::clone() const
{
  #ifdef USE_HDF5
  assert( m_write_constraint_forces == false );
  assert( m_impact_solution == nullptr );
  #endif
  return std::make_unique<ImpactMap>( m_warm_start );
}
