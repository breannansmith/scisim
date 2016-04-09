// RigidBody2DForce.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef RIGID_BODY_2D_FORCE_H
#define RIGID_BODY_2D_FORCE_H

#include "scisim/Math/MathDefines.h"
#include <memory>

enum class RigidBody2DForceType
{
  NEAR_EARTH_GRAVITY
};

class RigidBody2DForce
{

public:

  RigidBody2DForce( const RigidBody2DForce& ) = delete;
  RigidBody2DForce( RigidBody2DForce&& ) = delete;
  RigidBody2DForce& operator=( const RigidBody2DForce& ) = delete;
  RigidBody2DForce& operator=( RigidBody2DForce&& ) = delete;

  virtual ~RigidBody2DForce() = 0;

  virtual scalar computePotential( const VectorXs& q, const SparseMatrixsc& M ) const = 0;

  // result += Force
  virtual void computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, VectorXs& result ) const = 0;

  virtual std::unique_ptr<RigidBody2DForce> clone() const = 0;

  virtual void serialize( std::ostream& output_stream ) const = 0;

protected:

  RigidBody2DForce() = default;

};

#endif
