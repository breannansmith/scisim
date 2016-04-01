// ImpactOperatorUtilities.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "ImpactOperatorUtilities.h"

#include "scisim/Constraints/Constraint.h"

void ImpactOperatorUtilities::computeN( const FlowableSystem& fsys, const std::vector<std::unique_ptr<Constraint>>& V, const VectorXs& q, SparseMatrixsc& N )
{
  assert( N.cols() == int( V.size() ) );

  // N.reserve( 0 ) seems to segfault for recent versions of Eigen
  if( N.cols() == 0 )
  {
    return;
  }

  VectorXi column_nonzeros( N.cols() );
  {
    auto con_itr = V.cbegin();
    for( int col = 0; col < N.cols(); ++col )
    {
      assert( *con_itr != nullptr );
      column_nonzeros[col] = (*con_itr)->impactStencilSize();
      ++con_itr;
    }
    assert( con_itr == V.cend() );
  }

  N.reserve( column_nonzeros );
  {
    auto con_itr = V.cbegin();
    for( int col = 0; col < N.cols(); ++col )
    {
      assert( *con_itr != nullptr );
      (*con_itr)->evalgradg( q, col, N, fsys );
      ++con_itr;
    }
    assert( con_itr == V.cend() );
  }

  assert( column_nonzeros.sum() == N.nonZeros() );

  N.makeCompressed();
}

void ImpactOperatorUtilities::computeLCPQPLinearTerm( const SparseMatrixsc& N, const VectorXs& nrel, const VectorXs& CoR, const VectorXs& v0, const VectorXs& v0F, VectorXs& linear_term )
{
  assert( v0F.size() == v0.size() ); assert( N.rows() == v0.size() ); assert( N.cols() == nrel.size() );
  assert( CoR.size() == nrel.size() ); assert( ( CoR.array() >= 0.0 ).all() ); assert( ( CoR.array() <= 1.0 ).all() );

  linear_term = N.transpose() * v0F + nrel + ( CoR.array() * ( N.transpose() * v0 + nrel ).array() ).matrix();

  // If the CoR is constant, double check against the simplified solution
  #ifndef NDEBUG
  if( ( CoR.array() == CoR(0) ).all() )
  {
    const VectorXs linear_term_single_cor = N.transpose() * ( CoR(0) * v0 + v0F ) + ( 1.0 + CoR(0) ) * nrel;
    assert( ( linear_term - linear_term_single_cor ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  }
  #endif
}

void ImpactOperatorUtilities::evalKinematicRelativeVelocityN( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& gdotN )
{
  gdotN.resize( active_set.size() );
  int cnstrnt_idx = 0;
  for( const std::unique_ptr<Constraint>& constraint : active_set )
  {
    constraint->evalKinematicNormalRelVel( q, cnstrnt_idx, gdotN );
    ++cnstrnt_idx;
  }
}
