// PenaltyForce.h
//
// Breannan Smith
// Last updated: 10/11/2015

#ifndef PENALTY_FORCE_H
#define PENALTY_FORCE_H

#include "Ball2DForce.h"

class PenaltyForce final : public Ball2DForce
{

public:

  explicit PenaltyForce( const scalar& k, const scalar& power );
  explicit PenaltyForce( const PenaltyForce& other );
  explicit PenaltyForce( std::istream& input_stream );

  virtual ~PenaltyForce() override;

  virtual scalar computePotential( const VectorXs& q, const SparseMatrixsc& M, const VectorXs& r ) const override;

  // result += Force
  virtual void computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, const VectorXs& r, VectorXs& result ) const override;

  virtual std::unique_ptr<Ball2DForce> clone() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

private:

  const scalar m_k;
  const scalar m_power;

};

#endif
