// RigidBody2DGeometry.h
//
// Breannan Smith
// Last updated: 01/07/2016

#ifndef RIGID_BODY_2D_GEOMETRY_H
#define RIGID_BODY_2D_GEOMETRY_H

#include "scisim/Math/MathDefines.h"

#include <memory>

enum class RigidBody2DGeometryType : std::uint8_t
{
  CIRCLE, BOX
};

class RigidBody2DGeometry
{

public:

  virtual ~RigidBody2DGeometry() = 0;

  virtual RigidBody2DGeometryType type() const = 0;

  virtual std::unique_ptr<RigidBody2DGeometry> clone() const = 0;

  void computeAABB( const Vector2s& x, const scalar& theta, Array2s& min, Array2s& max ) const;

  // density: (in) positive density of the rigid body
  // m:      (out) positive total mass of the rigid body
  // I:      (out) positive moment of inertia of the rigid body
  void computeMassAndInertia( const scalar& density, scalar& m, scalar& I ) const;

  void serialize( std::ostream& output_stream ) const;

private:

  virtual void AABB( const Vector2s& x, const scalar& theta, Array2s& min, Array2s& max ) const = 0;

  virtual void massAndInertia( const scalar& density, scalar& m, scalar& I ) const = 0;

  virtual void serializeState( std::ostream& output_stream ) const = 0;

};

#endif
