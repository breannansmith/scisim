// StaggeredProjections.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef STAGGERED_PROJECTIONS_H
#define STAGGERED_PROJECTIONS_H

#include "FrictionSolver.h"

#include <memory>
#include "scisim/Math/MathDefines.h"

class ImpactOperator;
class FrictionOperator;
class Constraint;
class FlowableSystem;

class StaggeredProjections final : public FrictionSolver
{

public:

  StaggeredProjections( const bool warm_start_alpha, const bool warm_start_beta, const ImpactOperator& impact_operator, const FrictionOperator& friction_operator );
  explicit StaggeredProjections( std::istream& input_stream );

  virtual ~StaggeredProjections() override;

  // TODO: Better handling of f
  virtual void solve( const unsigned iteration, const scalar& dt, const FlowableSystem& fsys, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& CoR, const VectorXs& mu, const VectorXs& q0, const VectorXs& v0, std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const VectorXs& nrel_extra, const VectorXs& drel_extra, const unsigned max_iters, const scalar& tol, VectorXs& f, VectorXs& alpha, VectorXs& beta, VectorXs& vout, bool& solve_succeeded, scalar& error ) override;

  virtual unsigned numFrictionImpulsesPerNormal( const unsigned ambient_space_dimensions ) const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual std::string name() const override;

private:

  const bool m_warm_start_alpha;
  const bool m_warm_start_beta;
  const std::unique_ptr<ImpactOperator> m_impact_operator;
  const std::unique_ptr<FrictionOperator> m_friction_operator;

};

#endif
