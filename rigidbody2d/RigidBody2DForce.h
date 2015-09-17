// RigidBody2DForce.h
//
// Breannan Smith
// Last updated: 09/10/2015

#ifndef RIGID_BODY_2D_FORCE_H
#define RIGID_BODY_2D_FORCE_H

#include <memory>
#include "scisim/Math/MathDefines.h"

class RigidBody2DForce
{

public:

  virtual ~RigidBody2DForce() = 0;

  virtual scalar computePotential( const VectorXs& q, const SparseMatrixsc& M ) const = 0;

  // result += Force
  virtual void computeForce( const VectorXs& q, const VectorXs& v, const SparseMatrixsc& M, VectorXs& result ) const = 0;

  virtual std::unique_ptr<RigidBody2DForce> clone() const = 0;

  std::string name() const;

  void serialize( std::ostream& output_stream ) const;

private:

  virtual std::string getName() const = 0;
  virtual void serializeState( std::ostream& output_stream ) const = 0;

};

#endif
