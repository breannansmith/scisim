#ifndef RIGID_BODY_3D_STATE_H
#define RIGID_BODY_3D_STATE_H

#include <memory>

#include "scisim/Math/MathDefines.h"
#include "Portals/PlanarPortal.h"
#include "StaticGeometry/StaticCylinder.h"
#include "Forces/Force.h"
#include "Geometry/RigidBodyGeometry.h"

class StaticPlane;

enum class SimBoundaryBehavior
{
  NONE,
  EXIT
};

class RigidBody3DState final
{

public:

  RigidBody3DState();

  ~RigidBody3DState() = default;

  RigidBody3DState( const RigidBody3DState& other );
  RigidBody3DState( RigidBody3DState&& ) = default;

  RigidBody3DState& operator=( const RigidBody3DState& other );
  RigidBody3DState& operator=( RigidBody3DState&& ) = default;

  void setState( const std::vector<Vector3s>& X, const std::vector<Vector3s>& V, const std::vector<scalar>& M, const std::vector<VectorXs>& R, const std::vector<Vector3s>& omega, const std::vector<Vector3s>& I0, const std::vector<bool>& fixed, const std::vector<unsigned>& geom_indices, const std::vector<std::unique_ptr<RigidBodyGeometry>>& geometry );

  unsigned nbodies() const;
  unsigned ngeo() const;

  VectorXs& q();
  const VectorXs& q() const;

  VectorXs& v();
  const VectorXs& v() const;

  SparseMatrixsc& M0();
  const SparseMatrixsc& M0() const;

  SparseMatrixsc& M();
  const SparseMatrixsc& M() const;

  SparseMatrixsc& Minv0();
  const SparseMatrixsc& Minv0() const;

  SparseMatrixsc& Minv();
  const SparseMatrixsc& Minv() const;

  const std::vector<bool>& fixed() const;

  bool isKinematicallyScripted( const unsigned bdy_idx ) const;

  const std::vector<std::unique_ptr<RigidBodyGeometry>>& geometry() const;

  const std::vector<unsigned>& indices() const;

  const RigidBodyGeometry& getGeometryOfBody( const unsigned bdy_idx ) const;

  unsigned getGeometryIndexOfBody( const unsigned bdy_idx ) const;

  // Returns the total mass of the given body
  const scalar& getTotalMass( const unsigned body ) const;

  const Eigen::Map<const Matrix33sr> getInertia( const unsigned body ) const;

  void updateMandMinv();

  std::vector<std::unique_ptr<Force>>& forces();

  const std::vector<std::unique_ptr<Force>>& forces() const;

  void addForce( const Force& new_force );

  // Static planes
  void addStaticPlane( const StaticPlane& new_plane );
  StaticPlane& staticPlane( const std::vector<StaticPlane>::size_type plane_idx );
  const StaticPlane& staticPlane( const std::vector<StaticPlane>::size_type plane_idx ) const;
  const std::vector<StaticPlane>& staticPlanes() const;
  std::vector<StaticPlane>::size_type numStaticPlanes() const;
  void deleteStaticPlane( const std::vector<StaticPlane>::size_type plane_index );

  // Static cylinders
  void addStaticCylinder( const StaticCylinder& new_cylinder );
  StaticCylinder& staticCylinder( const std::vector<StaticCylinder>::size_type cylinder_index );
  const StaticCylinder& staticCylinder( const std::vector<StaticCylinder>::size_type cylinder_index ) const;
  const std::vector<StaticCylinder>& staticCylinders() const;
  std::vector<StaticCylinder>::size_type numStaticCylinders() const;

  void addPlanarPortal( const PlanarPortal& planar_portal );
  std::vector<PlanarPortal>::size_type numPlanarPortals() const;
  const PlanarPortal& planarPortal( const std::vector<PlanarPortal>::size_type portal_index ) const;

  void setBoundaryBehavior(const SimBoundaryBehavior& behavior);
  void setBoundaryMin(const Vector3s& min);
  void setBoundaryMax(const Vector3s& max);

  SimBoundaryBehavior boundaryBehavior() const;
  const Vector3s& boundaryMin() const;
  const Vector3s& boundaryMax() const;

  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

private:

  unsigned m_nbodies;
  VectorXs m_q;
  VectorXs m_v;
  SparseMatrixsc m_M0;
  SparseMatrixsc m_Minv0;
  SparseMatrixsc m_M;
  SparseMatrixsc m_Minv;
  std::vector<bool> m_fixed;
  std::vector<std::unique_ptr<RigidBodyGeometry>> m_geometry;
  std::vector<unsigned> m_geometry_indices;
  std::vector<std::unique_ptr<Force>> m_forces;
  std::vector<StaticPlane> m_static_planes;
  std::vector<StaticCylinder> m_static_cylinders;
  std::vector<PlanarPortal> m_planar_portals;

  SimBoundaryBehavior m_boundary_behavior;
  Vector3s m_boundary_min;
  Vector3s m_boundary_max;

};

#endif
