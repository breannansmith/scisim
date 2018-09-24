#ifndef GRR_FRICTION_H
#define GRR_FRICTION_H

#include "FrictionSolver.h"

class ImpactOperator;
class FrictionOperator;

class GRRFriction final : public FrictionSolver
{

public:

  GRRFriction( const ImpactOperator& impact_operator, const FrictionOperator& friction_operator );
  explicit GRRFriction( std::istream& input_stream );

  GRRFriction( const GRRFriction& ) = delete;
  GRRFriction( GRRFriction&& ) = delete;
  GRRFriction& operator=( const GRRFriction& ) = delete;
  GRRFriction& operator=( GRRFriction&& ) = delete;

  virtual ~GRRFriction() override;

  virtual void solve( const unsigned iteration, const scalar& dt, const FlowableSystem& fsys, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& CoR, const VectorXs& mu, const VectorXs& q0, const VectorXs& v0, std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const VectorXs& nrel_extra, const VectorXs& drel_extra, const unsigned max_iters, const scalar& tol, VectorXs& f, VectorXs& alpha, VectorXs& beta, VectorXs& vout, bool& solve_succeeded, scalar& error ) override;

  virtual unsigned numFrictionImpulsesPerNormal( const unsigned ambient_space_dimensions ) const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual std::string name() const override;

private:

  const std::unique_ptr<ImpactOperator> m_impact_operator;
  const std::unique_ptr<FrictionOperator> m_friction_operator;

};

#endif
