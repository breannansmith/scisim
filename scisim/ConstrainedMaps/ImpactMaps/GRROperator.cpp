// GRROperator.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "GRROperator.h"

#include "scisim/ConstrainedMaps/ConstrainedMapUtilities.h"

#include <iostream>

GRROperator::GRROperator( const ImpactOperator& elastic_operator, const ImpactOperator& inelastic_operator )
: m_elastic_operator( elastic_operator.clone() )
, m_inelastic_operator( inelastic_operator.clone() )
{
  assert( m_elastic_operator != nullptr );
  assert( m_inelastic_operator != nullptr );
}

GRROperator::GRROperator( std::istream& input_stream )
: m_elastic_operator( ConstrainedMapUtilities::deserializeImpactOperator( input_stream ) )
, m_inelastic_operator( ConstrainedMapUtilities::deserializeImpactOperator( input_stream ) )
{
  assert( m_elastic_operator != nullptr );
  assert( m_inelastic_operator != nullptr );
}

GRROperator::~GRROperator()
{}

void GRROperator::flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha )
{
  // Not intended for use with staggered projections
  assert( ( v0.array() == v0F.array() ).all() );
  // Only works with one CoR
  assert( ( CoR.array() == CoR(0) ).all() );
  if( ( CoR.array() != CoR(0) ).any() )
  {
    std::cerr << "Error, GRR only supports a single coefficient of restitution. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  // If the CoR is 0 or 1, don't waste time calling the other map
  if( CoR(0) == 0.0 )
  {
    m_inelastic_operator->flow( cons, M, Minv, q0, v0, v0F, N, Q, nrel, CoR, alpha );
    return;
  }
  else if( CoR(0) == 1.0 )
  {
    m_elastic_operator->flow( cons, M, Minv, q0, v0, v0F, N, Q, nrel, CoR, alpha );
    return;
  }

  // Create temporary storage
  VectorXs alpha_out{ alpha.size() };

  alpha_out.setZero();
  assert( m_inelastic_operator != nullptr );
  m_inelastic_operator->flow( cons, M, Minv, q0, v0, v0F, N, Q, nrel, VectorXs::Zero( CoR.size() ), alpha_out );
  //v1 = ( 1.0 - CoR ) * v_out;
  alpha = ( 1.0 - CoR(0) ) * alpha_out;

  alpha_out.setZero();
  assert( m_elastic_operator != nullptr );
  m_elastic_operator->flow( cons, M, Minv, q0, v0, v0F, N, Q, nrel, VectorXs::Ones( CoR.size() ), alpha_out );
  //v1 += CoR * v_out;
  alpha += CoR(0) * alpha_out;
}

std::string GRROperator::name() const
{
  return "grr";
}

std::unique_ptr<ImpactOperator> GRROperator::clone() const
{
  return std::unique_ptr<ImpactOperator>{ new GRROperator{ *m_elastic_operator, *m_inelastic_operator } };
}

void GRROperator::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  ConstrainedMapUtilities::serialize( m_elastic_operator, output_stream );
  ConstrainedMapUtilities::serialize( m_inelastic_operator, output_stream );
}
