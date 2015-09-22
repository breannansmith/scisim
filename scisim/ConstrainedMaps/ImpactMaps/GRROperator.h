// GRROperator.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef GRR_OPERATOR_H
#define GRR_OPERATOR_H

#include "ImpactOperator.h"

#include <memory>

class GRROperator final : public ImpactOperator
{

public:

  GRROperator( const ImpactOperator& elastic_operator, const ImpactOperator& inelastic_operator );
  explicit GRROperator( std::istream& input_stream );
  virtual ~GRROperator() override;

  virtual void flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha ) override;

  virtual std::string name() const override;

  virtual std::unique_ptr<ImpactOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

private:

  const std::unique_ptr<ImpactOperator> m_elastic_operator;
  const std::unique_ptr<ImpactOperator> m_inelastic_operator;

};

#endif
