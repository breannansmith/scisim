#ifndef LINEAR_MDP_OPERATOR_QL
#define LINEAR_MDP_OPERATOR_QL

#include "FrictionOperator.h"

// TODO: Move to unsigned when possible
class LinearMDPOperatorQL final : public FrictionOperator
{

public:

  LinearMDPOperatorQL( const int disk_samples, const scalar& eps );
  explicit LinearMDPOperatorQL( std::istream& input_stream );

  virtual ~LinearMDPOperatorQL() override = default;

  virtual void flow( const scalar& t, const SparseMatrixsc& Minv, const VectorXs& v0, const SparseMatrixsc& D, const SparseMatrixsc& Q, const VectorXs& gdotD, const VectorXs& mu, const VectorXs& alpha, VectorXs& beta, VectorXs& lambda ) override;

  virtual int numFrictionImpulsesPerNormal() const override;

  virtual void formGeneralizedFrictionBasis( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D, VectorXs& drel ) override;

  virtual std::string name() const override;

  virtual std::unique_ptr<FrictionOperator> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual bool isLinearized() const override;

private:

  const int m_disk_samples;
  const scalar m_tol;

};

#endif
