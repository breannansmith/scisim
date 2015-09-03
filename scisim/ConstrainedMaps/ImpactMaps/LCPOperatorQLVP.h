// LCPOperatorQLVP.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef LCP_OPERATOR_QL_VP
#define LCP_OPERATOR_QL_VP

#include "ImpactOperator.h"

class LCPOperatorQLVP final : public ImpactOperator
{

public:

  LCPOperatorQLVP( const scalar& tol );
  LCPOperatorQLVP( const LCPOperatorQLVP& other );
  virtual ~LCPOperatorQLVP() override = default;

  virtual void flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha ) override;

  virtual std::string name() const override;

  virtual std::unique_ptr<ImpactOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

private:

  const scalar m_tol;

};

#endif
