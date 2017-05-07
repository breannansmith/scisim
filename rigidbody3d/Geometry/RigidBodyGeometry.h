// RigidBodyGeometry.h
//
// Breannan Smith
// Last updated: 09/15/2015

#ifndef RIGID_BODY_GEOMETRY_H
#define RIGID_BODY_GEOMETRY_H

#include "scisim/Math/MathDefines.h"

#include <memory>

enum class RigidBodyGeometryType : std::uint8_t
{
  BOX,
  SPHERE,
  STAPLE,
  TRIANGLE_MESH
};

class RigidBodyGeometry
{

public:

  RigidBodyGeometry( const RigidBodyGeometry& ) = delete;
  RigidBodyGeometry( RigidBodyGeometry&& ) = delete;
  RigidBodyGeometry& operator=( const RigidBodyGeometry& ) = delete;
  RigidBodyGeometry& operator=( RigidBodyGeometry&& ) = delete;

  virtual ~RigidBodyGeometry();

  virtual RigidBodyGeometryType getType() const = 0;

  virtual std::unique_ptr<RigidBodyGeometry> clone() const = 0;

  virtual void computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const = 0;

  // density: (in) density of the rigid body
  // M:  (out) total mass of the rigid body
  // CM: (out) translation to take this geometry from the the origin to its input orientation
  // I:  (out) moment of inertia measured with respect to the principal axes
  // R:  (out) transformation to take this geometry from the principal axes to its input orientation
  virtual void computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const = 0;

  virtual std::string name() const = 0;

  virtual void serialize( std::ostream& output_stream ) const = 0;

  virtual scalar volume() const = 0;

protected:

  RigidBodyGeometry() = default;

};

#endif
