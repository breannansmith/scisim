#include "SymplecticEulerImpactFrictionMap.h"

#include <iostream>

#include "scisim/Math/MathUtilities.h"
#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/Constraints/Constraint.h"
#include "scisim/Constraints/ConstrainedSystem.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ScriptingCallback.h"
#include "scisim/UnconstrainedMaps/FlowableSystem.h"
#include "scisim/Utilities.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperatorUtilities.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"

SymplecticEulerImpactFrictionMap::SymplecticEulerImpactFrictionMap( const scalar& abs_tol, const unsigned max_iters, const ImpulsesToCache impulses_to_cache, const bool stabilize, const scalar& penetration_threshold )
: m_f( VectorXs::Zero( 0 ) )
, m_abs_tol( abs_tol )
, m_max_iters( max_iters )
, m_stabilize( stabilize )
, m_penetration_threshold( penetration_threshold )
, m_impulses_to_cache( impulses_to_cache )
#ifdef USE_HDF5
, m_write_constraint_forces( false )
, m_constraint_force_stream( nullptr )
#endif
{
  assert( m_abs_tol >= 0.0 );
  assert( m_penetration_threshold >= 0.0 );
}

SymplecticEulerImpactFrictionMap::SymplecticEulerImpactFrictionMap( std::istream& input_stream )
: m_f( MathUtilities::deserialize<VectorXs>( input_stream ) )
, m_abs_tol( Utilities::deserialize<scalar>( input_stream ) )
, m_max_iters( Utilities::deserialize<unsigned>( input_stream ) )
, m_stabilize( Utilities::deserialize<bool>( input_stream ) )
, m_penetration_threshold( Utilities::deserialize<scalar>( input_stream ) )
, m_impulses_to_cache( Utilities::deserialize<ImpulsesToCache>( input_stream ) )
#ifdef USE_HDF5
, m_write_constraint_forces( false )
, m_constraint_force_stream( nullptr )
#endif
{
  assert( m_abs_tol >= 0.0 );
  assert( m_penetration_threshold >= 0.0 );
}

static void initializeImpulses( const ImpulsesToCache cache_mode, const unsigned ambient_dims, const std::vector<std::unique_ptr<Constraint>>& active_set, ConstrainedSystem& csys, VectorXs& alpha, VectorXs& beta, const VectorXs& q, const VectorXs& v )
{
  switch( cache_mode )
  {
    case ImpulsesToCache::NONE:
    {
      alpha.setZero();
      beta.setZero();
      assert( csys.constraintCacheEmpty() );
      break;
    }
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
        assert( ambient_dims == 3 );
        unsigned col_num{ 0 };
        for( const std::unique_ptr<Constraint>& constraint : active_set )
        {
          VectorXs cached_impulse{ 6 };
          csys.getCachedConstraintImpulse( *constraint, cached_impulse );

          if( ( cached_impulse.array() == 0.0 ).all() )
          {
            alpha( col_num ) = 0.0;
            beta.segment<2>( 2 * col_num ).setZero();
          }
          else
          {
            // Grab the collision basis
            MatrixXXsc basis{3, 3};
            constraint->computeBasis( q, v, basis );
            assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-9 );
            assert( ( basis * basis.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );

            // Compute a rotation from the old normal to the currenet normal
            assert( fabs( basis.col(0).norm() - 1.0 ) <= 1.0e-9 );
            const Quaternions R{ Quaternions::FromTwoVectors( cached_impulse.segment<3>(0), basis.col(0) ) };
            assert( fabs( R.norm() - 1.0 ) <= 1.0e-9 );

            // Transform the previous force by the rotation of the normal
            const Vector3s f{ R * cached_impulse.segment<3>(3) };

            // Project the force onto the current basis
            alpha( col_num ) = f.dot( basis.col(0) );
            beta( 2 * col_num + 0 ) = f.dot( basis.col(1) );
            beta( 2 * col_num + 1 ) = f.dot( basis.col(2) );
          }

          col_num++;
        }
        assert( col_num == active_set.size() );
      }
      csys.clearConstraintCache();
      break;
    }
  }
}

static void cacheImpulses( const ImpulsesToCache cache_mode, const unsigned ambient_dims, const std::vector<std::unique_ptr<Constraint>>& active_set, ConstrainedSystem& csys, const VectorXs& alpha, const VectorXs& beta, const VectorXs& q, const VectorXs& v )
{
  switch( cache_mode )
  {
    case ImpulsesToCache::NONE:
    {
      assert( csys.constraintCacheEmpty() );
      break;
    }
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
    {
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
        assert( ambient_dims == 3 );
        assert( 2 * alpha.size() == beta.size() );
        unsigned col_num = 0;
        for( const std::unique_ptr<Constraint>& constraint : active_set )
        {
          // Compute the contact basis
          MatrixXXsc basis(3, 3);
          constraint->computeBasis( q, v, basis );
          assert( fabs( basis.determinant() - 1.0 ) <= 1.0e-9 );
          assert( ( basis * basis.transpose() - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );

          VectorXs cached_impulse{ 6 };
          // Cache the contact normal
          cached_impulse.segment<3>(0) = basis.col(0);
          // Compute the force in 3D cartesian space
          cached_impulse.segment<3>(3) = alpha( col_num ) * basis.col(0) + beta( 2 * col_num + 0 ) * basis.col(1) + beta( 2 * col_num + 1 ) * basis.col(2);
          csys.cacheConstraint( *constraint, cached_impulse );

          col_num++;
        }
        assert( col_num == active_set.size() );
      }
      break;
    }
  }
}

// TODO: Ignore the unconstrained map, somehow?
void SymplecticEulerImpactFrictionMap::flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 )
{
  assert( dt > 0.0 );
  assert( CoR_default >= 0.0 );
  assert( CoR_default <= 1.0 );
  assert( mu_default >= 0.0 );
  assert( q0.size() == fsys.nqdofs() );
  assert( q1.size() == fsys.nqdofs() );
  assert( v0.size() == fsys.nvdofs() );
  assert( v1.size() == fsys.nvdofs() );

  // TODO: Do something more elegant when the number of bodies changes due to insertions and deletions, like registering the maps with the scripting callbacks so they can update the cache.
  // Initialize the friction impulse cache
  if( m_f.size() != v0.size() )
  {
    m_f = VectorXs::Zero( v0.size() );
  }

  // Compute the force at the start of step and the corresponding change in velocity
  VectorXs vdelta( q0.size() );
  {
    VectorXs F( fsys.Minv().cols() );
    fsys.computeForce( q0, v0, dt, F );
    fsys.zeroOutForcesOnFixedBodies( F );
    vdelta = dt * ( fsys.Minv() * F );
  }

  // Take a predictor step for collision detection
  fsys.linearInertialConfigurationUpdate( q0, v0 + vdelta, dt, q1 );

  // Compute the set of active collisions
  std::vector<std::unique_ptr<Constraint>> active_set;
  csys.computeActiveSet( q0, q1, v0, active_set );
  // If there are no active constraints, there is no need to perform collision response
  if( active_set.empty() )
  {
    v1 = v0 + vdelta;
    fsys.linearInertialConfigurationUpdate( q0, v1, dt, q1 );
    csys.clearConstraintCache();
    #ifdef USE_HDF5
    if( m_write_constraint_forces )
    {
      exportConstraintForcesToBinary( q0, active_set, MatrixXXsc{ fsys.ambientSpaceDimensions(), 0 }, VectorXs::Zero(0), VectorXs::Zero(0), dt );
    }
    m_write_constraint_forces = false;
    m_constraint_force_stream = nullptr;
    #endif
    return;
  }
  const unsigned ncollisions{ static_cast<unsigned>( active_set.size() ) };

  // Set the coefficients of friction to the default
  VectorXs mu{ VectorXs::Constant( ncollisions, mu_default ) };
  // If scripting is enabled, use the scripted version
  call_back.frictionCoefficientCallback( active_set, mu );
  assert( ( mu.array() >= 0.0 ).all() );

  // Coefficients of restitution
  VectorXs CoR{ VectorXs::Constant( ncollisions, CoR_default ) };
  // If scripting is enabled, use the scripted version
  call_back.restitutionCoefficientCallback( active_set, CoR );
  assert( ( CoR.array() >= 0.0 ).all() );
  assert( ( CoR.array() <= 1.0 ).all() );

  // Normal impulse magnitudes
  VectorXs alpha{ ncollisions };
  // Friction impulses magnitudes
  VectorXs beta{ friction_solver.numFrictionImpulsesPerNormal( fsys.ambientSpaceDimensions() ) * ncollisions };

  initializeImpulses( m_impulses_to_cache, fsys.ambientSpaceDimensions(), active_set, csys, alpha, beta, q0, v0 );
  // std::cout << "alpha0: " << alpha.transpose() << std::endl;
  // std::cout << "beta0:  " << beta.transpose() << std::endl;

  // Pre-compute the full contact basis
  MatrixXXsc contact_bases;
  csys.computeContactBases( q0, v0, active_set, contact_bases );
  assert( contact_bases.rows() == fsys.ambientSpaceDimensions() );
  assert( contact_bases.cols() == fsys.ambientSpaceDimensions() * ncollisions );

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
  {
    scalar error;
    bool solve_succeeded;

    // TODO: Pull nrel and drel computation into functions
    VectorXs nrel;
    {
      SparseMatrixsc N{ static_cast<SparseMatrixsc::Index>( v0.size() ), static_cast<SparseMatrixsc::Index>( alpha.size() ) };
      ImpactOperatorUtilities::computeN( fsys, active_set, q0, N );
      nrel = N.transpose() * vdelta;
    }
    VectorXs drel;
    {
      SparseMatrixsc D;
      FrictionOperator::formGeneralizedSmoothFrictionBasis( unsigned( v0.size() ), unsigned( alpha.size() ), q0, active_set, contact_bases, D );
      drel = D.transpose() * vdelta;
    }

    if( m_stabilize )
    {
      VectorXs g0( ncollisions );
      // std::cout << "g0: ";
      for( unsigned col_idx = 0; col_idx < ncollisions; col_idx++ )
      {
        g0( col_idx ) = std::min( 0.0, active_set[col_idx]->evaluateGapFunction( q0 ) );
        // std::cout << g0( col_idx ) << " ";
        using std::fabs;
        if( fabs( g0( col_idx ) ) < m_penetration_threshold )
        {
          g0( col_idx ) = 0.0;
        }
      }
      // std::cout << std::endl;
      assert( ( g0.array() <= 0.0 ).all() );
      nrel += g0 / dt;
    }

    friction_solver.solve( iteration, dt, fsys, fsys.M(), fsys.Minv(), CoR, mu, q0, v0, active_set, contact_bases, nrel, drel, m_max_iters, m_abs_tol, m_f, alpha, beta, v1, solve_succeeded, error );
    assert( error >= 0.0 );
    if( !solve_succeeded )
    {
      std::cerr << "Warning, coupled impact/friction solve exceeded max iterations " << m_max_iters;
      std::cerr << " with absolute error " << error << " stepping to time " << iteration * dt << std::endl;
    }
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

  // For the special case mu == 0, friction should be zero
  #ifndef NDEBUG
  if( ( mu.array() == 0.0 ).all() )
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

  v1 += vdelta;

  // Check the (frictionless) KKT conditions
  #ifndef NDEBUG
  if( ( mu.array() == 0.0 ).all() )
  {
    SparseMatrixsc N{ static_cast<SparseMatrixsc::Index>( v0.size() ), static_cast<SparseMatrixsc::Index>( alpha.size() ) };
    ImpactOperatorUtilities::computeN( fsys, active_set, q0, N );
    assert( ( v0 + vdelta + fsys.Minv() * N * alpha - v1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    const VectorXs CoR_part = CoR(0) * N.transpose() * v0;
    const VectorXs rhs0 = N.transpose() * ( v0 + vdelta + fsys.Minv() * N * alpha ) + CoR_part;
    assert( ( alpha.array() >= 0.0 ).all() );
    assert( ( rhs0.array() >= -1.0e-5 ).all() );
    assert( ( alpha.array() * rhs0.array() ).matrix().lpNorm<Eigen::Infinity>() <= 1.0e-5 );
  }
  // TODO: Check the frictional KKT conditions
  // else
  // {
  //   std::cout << "alpha1: " << alpha.transpose() << std::endl;
  //   std::cout << "beta1:  " << beta.transpose() << std::endl;
  //   std::cout << "v1: " << v1.transpose() << std::endl;
  // }
  #endif

  // Cache the constraints for warm starting
  cacheImpulses( m_impulses_to_cache, fsys.ambientSpaceDimensions(), active_set, csys, alpha, beta, q0, v0 );

  #ifdef USE_HDF5
  // Export constraint forces, if requested
  if( m_write_constraint_forces )
  {
    exportConstraintForcesToBinary( q0, active_set, contact_bases, alpha, beta, dt );
  }
  m_write_constraint_forces = false;
  m_constraint_force_stream = nullptr;
  #endif

  fsys.linearInertialConfigurationUpdate( q0, v1, dt, q1 );
}

void SymplecticEulerImpactFrictionMap::resetCachedData()
{
  m_f = VectorXs::Zero( 0 );
}

#ifdef USE_HDF5
void SymplecticEulerImpactFrictionMap::exportConstraintForcesToBinary( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& contact_bases, const VectorXs& alpha, const VectorXs& beta, const scalar& dt )
{
  assert( m_write_constraint_forces );
  assert( m_constraint_force_stream != nullptr );
  ImpactFrictionMap::exportConstraintForcesToBinaryFile( q, constraints, contact_bases, alpha, beta, dt, *m_constraint_force_stream );
}
#endif

void SymplecticEulerImpactFrictionMap::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  MathUtilities::serialize( m_f, output_stream );
  Utilities::serialize( m_abs_tol, output_stream );
  Utilities::serialize( m_max_iters, output_stream );
  Utilities::serialize( m_stabilize, output_stream );
  Utilities::serialize( m_penetration_threshold, output_stream );
  Utilities::serialize( m_impulses_to_cache, output_stream );
  #ifdef USE_HDF5
  assert( m_write_constraint_forces == false );
  assert( m_constraint_force_stream == nullptr );
  #endif
}

std::string SymplecticEulerImpactFrictionMap::name() const
{
  return "symplectic_euler_impact_friction_map";
}

#ifdef USE_HDF5
void SymplecticEulerImpactFrictionMap::exportForcesNextStep( HDF5File& output_file )
{
  m_write_constraint_forces = true;
  m_constraint_force_stream = &output_file;
}
#endif
