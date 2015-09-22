// NearEarthGravityForce.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef NEAR_EARTH_GRAVITY_FORCE_H
#define NEAR_EARTH_GRAVITY_FORCE_H

#include "scisim/Math/MathDefines.h"
#include "RigidBody2DForce.h"

class NearEarthGravityForce final : public RigidBody2DForce
{

public:

  explicit NearEarthGravityForce( const Vector2s& g );
  explicit NearEarthGravityForce( std::istream& input_stream );

  virtual ~NearEarthGravityForce() override = default;

  virtual scalar computePotential( const VectorXs& q, const SparseMatrixsc& M ) const override;

  virtual void computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, VectorXs& result ) const override;

  virtual std::unique_ptr<RigidBody2DForce> clone() const override;

private:

  virtual std::string getName() const override;

  virtual void serializeState( std::ostream& output_stream ) const override;

  const Vector2s m_g;

};

#endif
