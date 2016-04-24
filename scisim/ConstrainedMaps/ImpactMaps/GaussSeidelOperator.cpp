// GaussSeidelOperator.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "GaussSeidelOperator.h"
#include "scisim/Utilities.h"
#include "scisim/Constraints/Constraint.h"

GaussSeidelOperator::GaussSeidelOperator( const scalar& v_tol )
: m_v_tol( v_tol )
{
  assert( m_v_tol >= 0.0 );
}

GaussSeidelOperator::GaussSeidelOperator( std::istream& input_stream )
: m_v_tol( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( m_v_tol >= 0.0 );
}

void GaussSeidelOperator::flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha )
{
  // TODO: Check input sizes, etc
  assert( ( alpha.array() == 0.0 ).all() );
  assert( ( v0.array() == v0F.array() ).all() );

  const unsigned ncons{ unsigned( cons.size() ) };

  VectorXs v1 = v0;

  // Iterate until all constraint violations fall below the threshold
  bool collision_happened = true;
  while( collision_happened )
  {
    collision_happened = false;

    // For each constraint
    for( unsigned current_idx = 0; current_idx < ncons; ++current_idx )
    {
      // If the relative velocity along the constraint is below the threshold
      const scalar ndotv = cons[current_idx]->evalNdotV( q0, v1 );
      if( ndotv < - m_v_tol )
      {
        // Reflect about this constraint
        scalar local_alpha;
        cons[current_idx]->resolveImpact( CoR( current_idx ), M, v0, ndotv, v1, local_alpha );
        assert( local_alpha >= 0.0 ); assert( cons[current_idx]->evalNdotV( q0, v1 ) >= 0.0 );
        alpha( current_idx ) += local_alpha;
        // And remember that a collision happend
        collision_happened = true;
      }
    }
    // TODO: 'rebase' v1 so it doesn't drift from v0 + Minv * N * alpha?
  }
  assert( ( v0 + Minv * N * alpha - v1 ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
}

std::string GaussSeidelOperator::name() const
{
  return "gauss_seidel";
}

std::unique_ptr<ImpactOperator> GaussSeidelOperator::clone() const
{
  return std::unique_ptr<ImpactOperator>{ new GaussSeidelOperator{ m_v_tol } };
}

void GaussSeidelOperator::serialize( std::ostream& output_stream ) const
{
  Utilities::serialize( m_v_tol, output_stream );
}
