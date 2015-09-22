// LCPOperatorQL.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef LCP_OPERATOR_QL
#define LCP_OPERATOR_QL

#include "ImpactOperator.h"

class LCPOperatorQL final : public ImpactOperator
{

public:

  explicit LCPOperatorQL( const scalar& eps );
  explicit LCPOperatorQL( std::istream& input_stream );
  LCPOperatorQL( const LCPOperatorQL& other );
  virtual ~LCPOperatorQL() override = default;

  virtual void flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha ) override;

  virtual std::string name() const override;

  virtual std::unique_ptr<ImpactOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

private:

  const scalar m_tol;

};

#endif
