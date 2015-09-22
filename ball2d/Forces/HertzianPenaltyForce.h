// HertzianPenaltyForce.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef HERTZIAN_PENALTY_FORCE_H
#define HERTZIAN_PENALTY_FORCE_H

#include "Ball2DForce.h"

class HertzianPenaltyForce final : public Ball2DForce
{

public:

  explicit HertzianPenaltyForce( const scalar& k );
  explicit HertzianPenaltyForce( std::istream& input_stream );

  virtual ~HertzianPenaltyForce() override;

  virtual scalar computePotential( const VectorXs& q, const SparseMatrixsc& M, const VectorXs& r ) const override;

  // result += Force
  virtual void computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, const VectorXs& r, VectorXs& result ) const override;

  virtual std::unique_ptr<Ball2DForce> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

private:

  const scalar m_k;

};

#endif
