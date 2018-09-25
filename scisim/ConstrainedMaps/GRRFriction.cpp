#include "GRRFriction.h"

#include <iostream>

#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"
#include "scisim/ConstrainedMaps/ConstrainedMapUtilities.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperatorUtilities.h"
#include "scisim/Constraints/Constraint.h"

GRRFriction::GRRFriction( const ImpactOperator& impact_operator, const FrictionOperator& friction_operator )
: m_impact_operator( impact_operator.clone() )
, m_friction_operator( friction_operator.clone() )
{}

GRRFriction::GRRFriction( std::istream& input_stream )
: m_impact_operator( ConstrainedMapUtilities::deserializeImpactOperator( input_stream ) )
, m_friction_operator( ConstrainedMapUtilities::deserializeFrictionOperator( input_stream ) )
{}

void GRRFriction::solve( const unsigned iteration, const scalar& dt, const FlowableSystem& fsys, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& CoR, const VectorXs& mu, const VectorXs& q0, const VectorXs& v0, std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const VectorXs& nrel_extra, const VectorXs& drel_extra, const unsigned /*max_iters*/, const scalar& /*tol*/, VectorXs& /*f*/, VectorXs& alpha, VectorXs& beta, VectorXs& vout, bool& solve_succeeded, scalar& error )
{
  if( nrel_extra.size() != 0 )
  {
    std::cerr << "Extra nrel option not supported for GRRFriction::solve yet." << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( drel_extra.size() != 0 )
  {
    std::cerr << "Extra drel option not supported for GRRFriction::solve yet." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  // Impact basis
  SparseMatrixsc N{ v0.size(), alpha.size() };
  ImpactOperatorUtilities::computeN( fsys, active_set, q0, N );

  // Friction basis
  SparseMatrixsc D;

  // Kinematic relative velocities
  VectorXs nrel{ alpha.size() };
  VectorXs drel{ 2 * alpha.size() };
  Constraint::evalKinematicRelVelGivenBases( q0, v0, active_set, contact_bases, nrel, drel );
  // TODO: Temporary workaround until contact_bases supports linearized friction bases
  if( !m_friction_operator->isLinearized() )
  {
    FrictionOperator::formGeneralizedSmoothFrictionBasis( unsigned( v0.size() ), unsigned( alpha.size() ), q0, active_set, contact_bases, D );
  }
  else
  {
    drel.resize( beta.size() );
    D.resize( v0.size(), beta.size() );
    m_friction_operator->formGeneralizedFrictionBasis( q0, v0, active_set, D, drel );
  }

  // Impact solve
  {
    // Quadratic term in LCP QP
    const SparseMatrixsc QN{ N.transpose() * Minv * N };
    assert( ( Eigen::Map<const ArrayXs>{QN.valuePtr(), QN.nonZeros()} != 0.0 ).any() );

    alpha.setZero();
    m_impact_operator->flow( active_set, M, Minv, q0, v0, v0, N, QN, nrel, CoR, alpha );
  }

  // Friction solve
  {
    // Quadratic term in MDP QP
    const SparseMatrixsc QD{ D.transpose() * Minv * D };
    assert( ( Eigen::Map<const ArrayXs>{QD.valuePtr(), QD.nonZeros()} != 0.0 ).any() );

    vout = v0 + Minv * N * alpha;
    beta.setZero();
    // TODO: Get rid of lambda from the friction operator
    VectorXs temp_lambda{ mu.size() };
    m_friction_operator->flow( iteration * dt, Minv, vout, D, QD, drel, mu, alpha, beta, temp_lambda );
  }

  vout = v0 + Minv * ( N * alpha + D * beta );
  solve_succeeded = true;
  error = 0.0;
}

unsigned GRRFriction::numFrictionImpulsesPerNormal( const unsigned /*ambient_space_dimensions*/ ) const
{
  return m_friction_operator->numFrictionImpulsesPerNormal();
}

void GRRFriction::serialize( std::ostream& output_stream ) const
{
  ConstrainedMapUtilities::serialize( m_impact_operator, output_stream );
  ConstrainedMapUtilities::serialize( m_friction_operator, output_stream );
}

std::string GRRFriction::name() const
{
  return "grr_friction";
}
