// RigidBodyBox.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef RIGID_BODY_BOX
#define RIGID_BODY_BOX

#include "RigidBodyGeometry.h"

class RigidBodyBox final : public RigidBodyGeometry
{

public:

  explicit RigidBodyBox( std::istream& input_stream );
  explicit RigidBodyBox( const Vector3s& half_widths );
  virtual ~RigidBodyBox() override;

  virtual RigidBodyGeometryType getType() const override;

  virtual std::unique_ptr<RigidBodyGeometry> clone() const override;

  virtual void computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const override;

  virtual void computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const override;

  virtual std::string name() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual scalar volume() const override;

  const Vector3s& halfWidths() const;

private:

  void computeInertia( const scalar& M, Vector3s& I ) const;

  const Vector3s m_half_widths;

};

#endif
