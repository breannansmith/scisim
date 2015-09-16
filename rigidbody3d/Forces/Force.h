// Force.h
//
// Breannan Smith
// Last updated: 09/15/2015

#ifndef FORCE_H
#define FORCE_H

#include "SCISim/Math/MathDefines.h"
#include <string>
#include <memory>

class Force
{

public:

  virtual ~Force() = 0;

  virtual scalar computePotential( const VectorXs& q, const SparseMatrixsc& M ) const = 0;

  // result += Force
  virtual void computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, VectorXs& result ) const = 0;

  virtual std::string name() const = 0;

  virtual std::unique_ptr<Force> clone() const = 0;

  virtual void serialize( std::ostream& output_stream ) const = 0;

};

#endif
