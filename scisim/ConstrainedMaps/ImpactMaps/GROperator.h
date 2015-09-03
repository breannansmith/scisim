// GROperator.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef GR_OPERATOR_H
#define GR_OPERATOR_H

#include "ImpactOperator.h"

#include <memory>

class GROperator : public ImpactOperator
{

public:

  GROperator( const scalar& v_tol, const ImpactOperator& impact_operator );
  GROperator( std::istream& input_stream );
  virtual ~GROperator() override;

  // TODO: Q isn't useful here, revise interface
  virtual void flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha ) override;

  virtual std::string name() const override;

  virtual std::unique_ptr<ImpactOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

private:

  const scalar m_v_tol;
  // TODO: Limit impact_operator to LCP solver, could cause bugs in sub-problem solve
  const std::unique_ptr<ImpactOperator> m_impact_operator;

};

#endif
