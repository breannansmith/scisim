// GaussSeidelOperator.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef GAUSS_SEIDEL_OPERATOR
#define GAUSS_SEIDEL_OPERATOR

#include "ImpactOperator.h"

class GaussSeidelOperator final : public ImpactOperator
{

public:

  explicit GaussSeidelOperator( const scalar& v_tol );
  explicit GaussSeidelOperator( std::istream& input_stream );

  virtual ~GaussSeidelOperator() override  = default;

  virtual void flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha ) override;

  virtual std::string name() const override;

  virtual std::unique_ptr<ImpactOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

private:

  const scalar m_v_tol;

};

#endif
