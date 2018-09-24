#ifndef JACOBI_OPERATOR
#define JACOBI_OPERATOR

#include "ImpactOperator.h"

class JacobiOperator final : public ImpactOperator
{

public:

  explicit JacobiOperator( const scalar& v_tol );

  virtual ~JacobiOperator() override = default;

  virtual void flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha ) override;

  virtual std::string name() const override;

  virtual std::unique_ptr<ImpactOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

private:

  const scalar m_v_tol;

};

#endif
