// Ball2DForce.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef BALL_2D_GRAVITY_FORCE_H
#define BALL_2D_GRAVITY_FORCE_H

#include "Ball2DForce.h"

class Ball2DGravityForce final : public Ball2DForce
{

public:

  explicit Ball2DGravityForce( const Vector2s& g );
  explicit Ball2DGravityForce( std::istream& input_stream );

  virtual ~Ball2DGravityForce() override;

  virtual scalar computePotential( const VectorXs& q, const SparseMatrixsc& M, const VectorXs& r ) const override;

  // result += Force
  virtual void computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, const VectorXs& r, VectorXs& result ) const override;

  virtual std::unique_ptr<Ball2DForce> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

private:

  const Vector2s m_g;

};

#endif
