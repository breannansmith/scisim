// GROperator.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "GROperator.h"

#include <iostream>

#include "SCISim/Math/MathUtilities.h"
#include "SCISim/ConstrainedMaps/ConstrainedMapUtilities.h"
#include "SCISim/Utilities.h"

GROperator::GROperator( const scalar& v_tol, const ImpactOperator& impact_operator )
: m_v_tol( v_tol )
, m_impact_operator( impact_operator.clone() )
{
  assert( m_v_tol >= 0.0 );
  assert( m_impact_operator != nullptr );
}

GROperator::GROperator( std::istream& input_stream )
: m_v_tol( Utilities::deserialize<scalar>( input_stream ) )
, m_impact_operator( ConstrainedMapUtilities::deserializeImpactOperator( input_stream ) )
{
  assert( m_v_tol >= 0.0 );
  assert( m_impact_operator != nullptr );
}

GROperator::~GROperator()
{}

void GROperator::flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha )
{
  // Not intended for use with staggered projections
  assert( ( v0.array() == v0F.array() ).all() );
  // Only works with one CoR
  assert( ( CoR.array() == CoR(0) ).all() );
  if( ( CoR.array() != CoR(0) ).any() )
  {
    std::cerr << "Error, GR only supports a single coefficient of restitution. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  const unsigned ncons = alpha.size();

  // Clear alpha as we will accumulate the generalized impulse below
  alpha.setZero();

  VectorXs v1 = v0F;

  unsigned iteration = 0;
  while( true )
  {
    if( iteration != 0 && iteration % 1000000 == 0 )
    {
      std::cout << "GR Iteration: " << iteration << std::endl;
    }

    // For each constraint k in the active set K, if further violation is imminent, add the constraint to this iteration's active set
    std::vector<unsigned> violated_indices;
    unsigned num_contacts_with_negative_vel = 0;
    {
      // TODO: N is col-major, so just pull out the relavent column to avoid this temporary?
      const VectorXs vrel = N.transpose() * v1 + nrel;
      // Count the number of violated constraints
      for( unsigned con_idx = 0; con_idx < ncons; ++con_idx )
      {
        if( vrel( con_idx ) < - m_v_tol )
        {
          ++num_contacts_with_negative_vel;
        }
      }
      // Reserve space for the sub-problem
      violated_indices.resize( num_contacts_with_negative_vel );
      // Extract the sub-problem
      unsigned current_idx = 0;
      for( unsigned con_idx = 0; con_idx < ncons; ++con_idx )
      {
        if( vrel( con_idx ) < - m_v_tol )
        {
          violated_indices[current_idx++] = con_idx;
        }
      }
    }
    assert( violated_indices.size() == num_contacts_with_negative_vel );

    // If no indices are violated, our work is done
    if( num_contacts_with_negative_vel == 0 )
    {
      break;
    }

    // Form the 'local' problem
    VectorXs nrel_local{ num_contacts_with_negative_vel };
      VectorXs CoR_local{ num_contacts_with_negative_vel };
    for( int local_idx = 0; local_idx < nrel_local.size(); ++local_idx )
    {
      nrel_local( local_idx ) = nrel( violated_indices[local_idx] );
      CoR_local( local_idx ) = CoR( violated_indices[local_idx] );
    }

    VectorXs alpha_local{ VectorXs::Zero( num_contacts_with_negative_vel ) };
    SparseMatrixsc N_local;
    mathutils::extractColumns( N, violated_indices, N_local );
    const SparseMatrixsc Q_local = N_local.transpose() * Minv * N_local;
    // Solve the 'local' problem
    m_impact_operator->flow( cons, M, Minv, q0, v1, v1, N_local, Q_local, nrel_local, CoR_local, alpha_local );

    // Velocity should be feasible along all 'local' constraints
    #ifndef NDEBUG
    {
      const VectorXs vout{ v1 + Minv * N_local * alpha_local };
      const VectorXs vrel{ N_local.transpose() * vout + nrel_local };
      assert( ( vrel.array() >= -m_v_tol ).all() );
    }
    #endif

    // Add the 'local' impulses into the full response
    for( int local_idx = 0; local_idx < nrel_local.size(); ++local_idx )
    {
      alpha( violated_indices[local_idx] ) += alpha_local( local_idx );
    }

    v1 = v0F + Minv * N * alpha;
    ++iteration;
  }

  // All outgoing velocities should be feasible
  assert( ( ( N.transpose() * v1 + nrel ).array() >= -m_v_tol ).all() );
}

std::string GROperator::name() const
{
  return "gr";
}

std::unique_ptr<ImpactOperator> GROperator::clone() const
{
  return std::unique_ptr<ImpactOperator>{ new GROperator{ m_v_tol, *m_impact_operator } };
}

void GROperator::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  Utilities::serializeBuiltInType( m_v_tol, output_stream );
  ConstrainedMapUtilities::serialize( m_impact_operator, output_stream );
}
