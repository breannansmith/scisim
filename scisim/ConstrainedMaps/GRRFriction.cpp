// GRRFriction.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "GRRFriction.h"

#include "SCISim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "SCISim/ConstrainedMaps/FrictionMaps/FrictionOperator.h"
#include "SCISim/ConstrainedMaps/ConstrainedMapUtilities.h"
#include "SCISim/ConstrainedMaps/ImpactMaps/ImpactOperatorUtilities.h"
#include "SCISim/UnconstrainedMaps/FlowableSystem.h"
#include "SCISim/Constraints/Constraint.h"

GRRFriction::GRRFriction( const ImpactOperator& impact_operator, const FrictionOperator& friction_operator )
: m_impact_operator( impact_operator.clone() )
, m_friction_operator( friction_operator.clone() )
{}

GRRFriction::GRRFriction( std::istream& input_stream )
: m_impact_operator( ConstrainedMapUtilities::deserializeImpactOperator( input_stream ) )
, m_friction_operator( ConstrainedMapUtilities::deserializeFrictionOperator( input_stream ) )
{}

GRRFriction::~GRRFriction()
{}

void GRRFriction::solve( const unsigned iteration, const scalar& dt, const FlowableSystem& fsys, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& CoR, const VectorXs& mu, const VectorXs& q0, const VectorXs& v0, std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const unsigned max_iters, const scalar& tol, VectorXs& f, VectorXs& alpha, VectorXs& beta, VectorXs& vout, bool& solve_succeeded, scalar& error )
{
  // Impact basis
  SparseMatrixsc N{ static_cast<SparseMatrixsc::Index>( v0.size() ), static_cast<SparseMatrixsc::Index>( alpha.size() ) };
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
    FrictionOperator::formGeneralizedSmoothFrictionBasis( v0.size(), alpha.size(), q0, active_set, contact_bases, D );
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

    alpha.setZero();
    m_impact_operator->flow( active_set, M, Minv, q0, v0, v0, N, QN, nrel, CoR, alpha );
  }

  // Friction solve
  {
    // Quadratic term in MDP QP
    const SparseMatrixsc QD{ D.transpose() * Minv * D };

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

unsigned GRRFriction::numFrictionImpulsesPerNormal( const unsigned ambient_space_dimensions ) const
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
