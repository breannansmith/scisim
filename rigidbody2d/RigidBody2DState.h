// RigidBody2DState.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef RIGID_BODY_2D_STATE
#define RIGID_BODY_2D_STATE

#include "scisim/Math/MathDefines.h"
#include "RigidBody2DGeometry.h"
#include "RigidBody2DForce.h"
#include "RigidBody2DStaticPlane.h"
#include "PlanarPortal.h"

class RigidBody2DState final
{

public:

  RigidBody2DState();
  RigidBody2DState( const VectorXs& q, const VectorXs& v, const VectorXs& m, const VectorXu& geometry_indices, const std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry, const std::vector<std::unique_ptr<RigidBody2DForce>>& forces, const std::vector<RigidBody2DStaticPlane>& planes, const std::vector<PlanarPortal>& planar_portals );
  RigidBody2DState( const RigidBody2DState& rhs );
  RigidBody2DState( RigidBody2DState&& rhs ) noexcept;
  ~RigidBody2DState() noexcept;

  RigidBody2DState& operator=( RigidBody2DState rhs );
  RigidBody2DState& operator=( RigidBody2DState&& rhs ) noexcept;

  friend void swap( RigidBody2DState& lhs, RigidBody2DState& rhs ) noexcept;

  VectorXs& q();
  const VectorXs& q() const;

  const VectorXs& v() const;

  const SparseMatrixsc& M() const;

  const SparseMatrixsc& Minv() const;

  const scalar& m( const unsigned bdy_idx ) const;
  const scalar& I( const unsigned bdy_idx ) const;

  // TODO: Remove these
  std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry();
  const std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry() const;
  VectorXu& geometryIndices();
  const VectorXu& geometryIndices() const;

  const std::unique_ptr<RigidBody2DGeometry>& bodyGeometry( const unsigned bdy_idx ) const;

  const std::vector<std::unique_ptr<RigidBody2DForce>>& forces() const;

  const std::vector<RigidBody2DStaticPlane>& planes() const;

  std::vector<PlanarPortal>& planarPortals();
  const std::vector<PlanarPortal>& planarPortals() const;

  // Computes a bounding box around the system
  Array4s computeBoundingBox() const;

  // Serialization and deserialization
  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

private:

  // Format: x0, y0, theta0, x1, y1, theta1, ...
  VectorXs m_q;
  // Format: vx0, vy0, omega0, vx1, vy1, omega1, ...
  VectorXs m_v;
  SparseMatrixsc m_M;
  SparseMatrixsc m_Minv;
  VectorXu m_geometry_indices;
  std::vector<std::unique_ptr<RigidBody2DGeometry>> m_geometry;
  std::vector<std::unique_ptr<RigidBody2DForce>> m_forces;
  std::vector<RigidBody2DStaticPlane> m_planes;
  std::vector<PlanarPortal> m_planar_portals;

};

#endif
