// RigidBodySphere.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef RIGID_BODY_SPHERE
#define RIGID_BODY_SPHERE

#include "RigidBodyGeometry.h"

class RigidBodySphere final : public RigidBodyGeometry
{

public:

  explicit RigidBodySphere( const scalar& r );
  explicit RigidBodySphere( std::istream& input_stream );
  virtual ~RigidBodySphere() override;

  virtual RigidBodyGeometryType getType() const override;

  virtual std::unique_ptr<RigidBodyGeometry> clone() const override;

  virtual void computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const override;

  virtual void computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const override;

  virtual std::string name() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual scalar volume() const override;

  const scalar& r() const;

private:

  const scalar m_r;

};

#endif
