#include "LCPOperatorAPGD.h"

#include "scisim/Math/QPSolvers/ProjectionSolvers.h"
#include "scisim/Math/QPSolvers/SparseMatrixVectorOperators.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperatorUtilities.h"
#include "scisim/Utilities.h"
#include "NonNegativeProjection.h"
#include "MinMapImpact.h"

#include <iostream>

LCPOperatorAPGD::LCPOperatorAPGD( const scalar& tol, const unsigned max_iters )
: m_tol( tol )
, m_max_iters( max_iters )
{
  assert( m_tol >= 0.0 );
}

LCPOperatorAPGD::LCPOperatorAPGD( std::istream& input_stream )
: m_tol( Utilities::deserialize<scalar>( input_stream ) )
, m_max_iters( Utilities::deserialize<unsigned>( input_stream ) )
{
  assert( m_tol >= 0.0 );
}

void LCPOperatorAPGD::flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha )
{
  // b in b^T \alpha
  VectorXs b;
  ImpactOperatorUtilities::computeLCPQPLinearTerm( N, nrel, CoR, v0, v0F, b );

  ProjectionSolveResults results;
  #ifdef MKL_FOUND
  ProjectionSolvers::APGD( NonNegativeProjection{}, MinMapImpact{}, ObjectiveMKLColumnMajor{}, GradientMKLColumnMajor{}, MultiplyMKLColumnMajor{}, m_tol, m_max_iters, Q, b, alpha, results );
  #else
  ProjectionSolvers::APGD( NonNegativeProjection{}, MinMapImpact{}, ObjectiveEigenColumnMajor{}, GradientEigenColumnMajor{}, MultiplyEigenColumnMajor{}, m_tol, m_max_iters, Q, b, alpha, results );
  #endif
  assert( ( alpha.array() >= 0.0 ).all() );

  if( results.status != ProjectionSolveStatus::Success )
  {
    std::cerr << "LCPOperatorAPGD warning, failed to acheive desired tolerance: " << results.achieved_tolerance << std::endl;
  }
}

std::string LCPOperatorAPGD::name() const
{
  return "lcp_apgd";
}

std::unique_ptr<ImpactOperator> LCPOperatorAPGD::clone() const
{
  return std::unique_ptr<ImpactOperator>{ new LCPOperatorAPGD{ m_tol, m_max_iters } };
}

void LCPOperatorAPGD::serialize( std::ostream& output_stream ) const
{
  Utilities::serialize( m_tol, output_stream );
  Utilities::serialize( m_max_iters, output_stream );
}
