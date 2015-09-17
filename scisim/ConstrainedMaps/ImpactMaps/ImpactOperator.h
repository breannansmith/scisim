// ImpactOperator.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef IMPACT_OPERATOR_H
#define IMPACT_OPERATOR_H

#include "scisim/Math/MathDefines.h"

#include <memory>

class Constraint;

class ImpactOperator
{

public:

  virtual ~ImpactOperator() = 0;

  virtual void flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha ) = 0;

  virtual std::string name() const = 0;

  virtual std::unique_ptr<ImpactOperator> clone() const = 0;

  virtual void serialize( std::ostream& output_stream ) const = 0;

};

#endif
