// BoundConstrainedMDPOperatorQL.h
//
// Breannan Smith
// Last updated: 09/21/2015

// Solves the problem:
//   1/2 beta^T Q beta + beta^T ( D^T v0 + gdotD  )
//    s.t. - diag(mu) alpha <= beta <= diag(mu) alpha
// Intended for use with 2D frictional contact problems.

#ifndef BOUND_CONSTRAINED_MDP_OPERATOR_QL
#define BOUND_CONSTRAINED_MDP_OPERATOR_QL

#include "FrictionOperator.h"

class BoundConstrainedMDPOperatorQL final : public FrictionOperator
{

public:

  explicit BoundConstrainedMDPOperatorQL( const scalar& tol );
  explicit BoundConstrainedMDPOperatorQL( std::istream& input_stream );
  BoundConstrainedMDPOperatorQL( const BoundConstrainedMDPOperatorQL& other );
  virtual ~BoundConstrainedMDPOperatorQL() override = default;

  virtual void flow( const scalar& t, const SparseMatrixsc& Minv, const VectorXs& v0, const SparseMatrixsc& D, const SparseMatrixsc& Q, const VectorXs& gdotD, const VectorXs& mu, const VectorXs& alpha, VectorXs& beta, VectorXs& lambda ) override;

  virtual int numFrictionImpulsesPerNormal() const override;

  virtual void formGeneralizedFrictionBasis( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D, VectorXs& drel ) override;

  virtual std::string name() const override;

  virtual std::unique_ptr<FrictionOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual bool isLinearized() const override;

private:

  const scalar m_tol;

};

#endif
