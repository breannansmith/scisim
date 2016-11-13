#include "Ball2DSobogusInterface.h"

#include "Core/Block.impl.hpp"
#include "Core/Block.io.hpp"

#include "Core/BlockSolvers/GaussSeidel.hpp"
#include "Core/BlockSolvers/Coloring.impl.hpp"

#include "Core/BlockSolvers/GaussSeidel.impl.hpp"
#include "Core/Utils/Polynomial.impl.hpp"

#include "FrictionProblem.hpp"

namespace bogus
{

Balls2DSobogusInterface::Balls2DSobogusInterface()
: m_primal( nullptr )
, m_dual( nullptr )
{}

Balls2DSobogusInterface::~Balls2DSobogusInterface()
{}

void Balls2DSobogusInterface::reset()
{
  m_dual.reset( nullptr );
  m_primal.reset( new PrimalFrictionProblem<2u> );
}

void Balls2DSobogusInterface::fromPrimal( const unsigned num_bodies, const Eigen::VectorXd& masses, const Eigen::VectorXd& f_in, const unsigned num_contacts, const Eigen::VectorXd& mu, const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor>& contact_bases, const Eigen::VectorXd& w_in,  const Eigen::VectorXi& obj_a, const Eigen::VectorXi& obj_b, const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>& HA, const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>& HB )
{
  reset();

  // Copy M
  // We don't actually need it after having computed a factorization of M, but we keep it around
  // in case we want to use dumpToFile()
  assert( m_primal != nullptr );
  m_primal->M.reserve( num_bodies );
  m_primal->M.setRows( num_bodies, 2 );
  m_primal->M.setCols( num_bodies, 2 );
  for( unsigned bdy_idx = 0; bdy_idx < num_bodies; ++bdy_idx )
  {
    assert( masses( 4 * bdy_idx + 0 ) == masses( 4 * bdy_idx + 3 ) );
    assert( masses( 4 * bdy_idx + 1 ) == 0.0 ); assert( masses( 4 * bdy_idx + 2 ) == 0.0 );
    m_primal->M.insertBack( bdy_idx, bdy_idx ) = Eigen::MatrixXd::Map( &masses( 4 * bdy_idx ), 2, 2 );
  }
  m_primal->M.finalize();

  // E
  m_primal->E.reserve( num_contacts );
  m_primal->E.setRows( num_contacts );
  m_primal->E.setCols( num_contacts );
  for( unsigned cntct_idx = 0; cntct_idx < num_contacts; ++cntct_idx )
  {
    // TODO: Check properties of the block
    m_primal->E.insertBack( cntct_idx, cntct_idx ) = contact_bases.block<2,2>( 0, 2 * cntct_idx );
  }
  m_primal->E.finalize();
  m_primal->E.cacheTranspose();

  // Build H
  m_primal->H.reserve( 2 * num_contacts );
  m_primal->H.setRows( num_contacts );
  m_primal->H.setCols( num_bodies, 2 );
  #ifndef BOGUS_DONT_PARALLELIZE
  #pragma omp parallel for
  #endif
  for( unsigned cntct_idx = 0; cntct_idx < num_contacts; ++cntct_idx )
  {
    if( obj_a( cntct_idx ) >= 0 && obj_b( cntct_idx ) >= 0 )
    {
      m_primal->H.insert( cntct_idx, obj_a( cntct_idx ) ) =   HA.block<2,2>( 2 * cntct_idx, 0 );
      m_primal->H.insert( cntct_idx, obj_b( cntct_idx ) ) = - HB.block<2,2>( 2 * cntct_idx, 0 );
    }
    else if( obj_a( cntct_idx ) >= 0 && obj_b( cntct_idx ) == -1 )
    {
      m_primal->H.insert( cntct_idx, obj_a( cntct_idx ) ) = HA.block<2,2>( 2 * cntct_idx, 0 );
    }
    #ifndef NDEBUG
    else
    {
      std::cerr << "Error, impossible code path hit in Balls2DSobogusInterface::fromPrimal. This is a bug." << std::endl;
      std::exit( EXIT_FAILURE );
    }
    #endif
  }
  m_primal->H.finalize();

  m_primal->f = f_in.data();
  m_primal->w = w_in.data();
  m_primal->mu = mu.data();
  m_primal->computeMInv();
}

void Balls2DSobogusInterface::computeDual()
{
  m_dual.reset( new DualFrictionProblem<2u> );
  m_dual->computeFrom( *m_primal );
}

double Balls2DSobogusInterface::solve( Eigen::VectorXd& r, Eigen::VectorXd& v, unsigned& num_iterations, const unsigned max_threads, const double& tol, const unsigned max_iters, const unsigned eval_every, const bool use_infinity_norm )
{
  assert( m_primal != nullptr );
  assert( r.size() == 2 * m_primal->H.rowsOfBlocks() );
  assert( v.size() == m_primal->H.cols() );

  // If dual has not been computed yet
  if( !m_dual )
  {
    computeDual();
  }

  // r to local coords
  Eigen::VectorXd r_loc{ m_primal->E.transpose() * r };

  // Setup GS parameters
  bogus::DualFrictionProblem<2u>::GaussSeidelType gs;
  gs.setTol( tol );
  gs.setMaxIters( max_iters );
  gs.setEvalEvery( eval_every );
  gs.setMaxThreads( max_threads );
  gs.setAutoRegularization( 0.0 );
  gs.useInfinityNorm( use_infinity_norm );

  // Compute coloring if multithreading
  gs.coloring().update( max_threads > 1, m_dual->W );

  m_dual->undoPermutation();
  if( max_threads > 1 )
  {
    m_dual->applyPermutation( gs.coloring().permutation );
    gs.coloring().resetPermutation();
  }
  // TODO: cacheTranspose in So-bogus triggers the undefined behavior sanitizer
  m_dual->W.cacheTranspose();

  const bool try_zero{ false };
  const double res{ m_dual->solveWith( gs, r_loc.data(), num_iterations, try_zero ) };

  // Compute the outgoing velocity
  v = m_primal->MInv * ( m_primal->H.transpose() * r_loc - Eigen::VectorXd::Map( m_primal->f, m_primal->H.cols() ) );

  // Convert r to world coords
  r = m_primal->E * r_loc;

  return res;
}

double Balls2DSobogusInterface::evalInfNormError( const Eigen::VectorXd& r )
{
  if( !m_dual )
  {
    computeDual();
  }

  // Convert r to local coords
  assert( m_primal != nullptr );
  const Eigen::VectorXd r_loc{ m_primal->E.transpose() * r };

  // Setup GS parameters
  bogus::DualFrictionProblem<2u>::GaussSeidelType gs;
  gs.useInfinityNorm( true );
  assert( m_dual != nullptr );
  gs.setMatrix( m_dual->W );

  return m_dual->evalWith( gs, r_loc.data() );
}

}
