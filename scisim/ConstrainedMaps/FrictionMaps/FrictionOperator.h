#ifndef FRICTION_OPERATOR_H
#define FRICTION_OPERATOR_H

#include "scisim/Math/MathDefines.h"

#include <memory>

class Constraint;

class FrictionOperator
{

public:

  FrictionOperator( const FrictionOperator& ) = delete;
  FrictionOperator( FrictionOperator&& ) = delete;
  FrictionOperator& operator=( const FrictionOperator& ) = delete;
  FrictionOperator& operator=( FrictionOperator&& ) = delete;

  virtual ~FrictionOperator() = 0;

  virtual void flow( const scalar& t, const SparseMatrixsc& Minv, const VectorXs& v0, const SparseMatrixsc& D, const SparseMatrixsc& Q, const VectorXs& gdotD, const VectorXs& mu, const VectorXs& alpha, VectorXs& beta, VectorXs& lambda ) = 0;

  virtual int numFrictionImpulsesPerNormal() const = 0;

  // Deprecated, do not use.
  virtual void formGeneralizedFrictionBasis( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D, VectorXs& drel );

  virtual std::string name() const = 0;

  virtual std::unique_ptr<FrictionOperator> clone() const = 0;

  virtual void serialize( std::ostream& output_stream ) const = 0;

  virtual bool isLinearized() const = 0;

  // TODO: Move to friction operator utilities
  static void formGeneralizedSmoothFrictionBasis( const unsigned ndofs, const unsigned ncons, const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& K, const MatrixXXsc& bases, SparseMatrixsc& D );

protected:

  FrictionOperator() = default;

};

#endif
