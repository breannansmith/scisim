// Ball2DForce.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef BALL_2D_FORCE_H
#define BALL_2D_FORCE_H

#include "SCISim/Math/MathDefines.h"

#include <memory>

class Ball2DForce
{

public:

  virtual ~Ball2DForce() = 0;

  virtual scalar computePotential( const VectorXs& q, const SparseMatrixsc& M, const VectorXs& r ) const = 0;

  // result += Force
  virtual void computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, const VectorXs& r, VectorXs& result ) const = 0;

  virtual std::unique_ptr<Ball2DForce> clone() const = 0;

  virtual void serialize( std::ostream& output_stream ) const = 0;

};

#endif
