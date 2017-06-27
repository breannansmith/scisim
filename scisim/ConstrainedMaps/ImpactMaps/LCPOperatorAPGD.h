#ifndef LCP_OPERATOR_APGD_H
#define LCP_OPERATOR_APGD_H

#include "ImpactOperator.h"

class LCPOperatorAPGD final : public ImpactOperator
{

public:

  LCPOperatorAPGD( const scalar& tol, const unsigned max_iters );
  explicit LCPOperatorAPGD( std::istream& input_stream );

  virtual ~LCPOperatorAPGD() override = default;

  virtual void flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha ) override;

  virtual std::string name() const override;

  virtual std::unique_ptr<ImpactOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

private:

  const scalar m_tol;
  const unsigned m_max_iters;

};

#endif
