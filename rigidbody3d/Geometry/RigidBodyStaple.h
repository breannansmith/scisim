// RigidBodyStaple.h
//
// Breannan Smith
// Last updated: 09/15/2015

#ifndef RIGID_BODY_STAPLE_H
#define RIGID_BODY_STAPLE_H

#include "RigidBodyGeometry.h"

class RigidBodyStaple final: public RigidBodyGeometry
{

public:

  virtual void computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const override;

  // w: width along x axis
  // l: length along y axis
  // D: diameter of staple
  RigidBodyStaple( const scalar& w, const scalar& l, const scalar& D );

  virtual ~RigidBodyStaple() override = default;

  virtual RigidBodyGeometryType getType() const override;

  virtual std::unique_ptr<RigidBodyGeometry> clone() const override;

  virtual void computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const override;

  virtual std::string name() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual scalar volume() const override;

  const std::vector<Vector3s>& points() const;

  const scalar& r() const;

private:

  RigidBodyStaple() = default;

  // Extent of the *skeleton* along the x axis
  scalar m_w;
  // Extent of the *skeleton* along the y axis
  scalar m_l;
  // Radius of the staple
  scalar m_r;

  // Four corners of the staple
  std::vector<Vector3s> m_p;

  scalar computeCenterOfMassY() const;

  static bool inertia_warning_printed;

};

#endif
