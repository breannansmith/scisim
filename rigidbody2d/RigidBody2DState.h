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

  RigidBody2DState() = default;
  RigidBody2DState( const VectorXs& q, const VectorXs& v, const VectorXs& m, const std::vector<bool>& fixed, const VectorXu& geometry_indices, const std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry, const std::vector<std::unique_ptr<RigidBody2DForce>>& forces, const std::vector<RigidBody2DStaticPlane>& planes, const std::vector<PlanarPortal>& planar_portals );

  RigidBody2DState( const RigidBody2DState& rhs );
  RigidBody2DState( RigidBody2DState&& ) = default;

  RigidBody2DState& operator=( const RigidBody2DState& rhs );
  RigidBody2DState& operator=( RigidBody2DState&& ) = default;

  unsigned nbodies() const;
  unsigned nportals() const;

  VectorXs& q();
  const VectorXs& q() const;

  VectorXs& v();
  const VectorXs& v() const;

  const SparseMatrixsc& M() const;

  const SparseMatrixsc& Minv() const;

  const scalar& m( const unsigned bdy_idx ) const;
  const scalar& I( const unsigned bdy_idx ) const;

  bool fixed( const int idx ) const;

  // Adds a new body at the end of the state vector
  void addBody( const Vector2s& x, const scalar& theta, const Vector2s& v, const scalar& omega, const scalar& rho, const unsigned geo_idx, const bool fixed );

  // Removes bodies at the given indices from the simulation
  void removeBodies( const Eigen::Ref<const VectorXu>& indices );

  // Adds a new circle geometry instance to the back of the geometry vector
  void addCircleGeometry( const scalar& r );

  // Removes geometry instances from the simulation
  void removeGeometry( const Eigen::Ref<const VectorXu>& indices );

  unsigned geometryIndex( const unsigned body_index ) const;

  // TODO: Remove these
  std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry();
  const std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry() const;
  VectorXu& geometryIndices();
  const VectorXu& geometryIndices() const;

  const std::unique_ptr<RigidBody2DGeometry>& bodyGeometry( const unsigned bdy_idx ) const;

  const std::vector<std::unique_ptr<RigidBody2DForce>>& forces() const;

  std::vector<RigidBody2DStaticPlane>& planes();
  const std::vector<RigidBody2DStaticPlane>& planes() const;

  std::vector<PlanarPortal>& planarPortals();
  const std::vector<PlanarPortal>& planarPortals() const;

  // Computes a bounding box around the system
  Array4s computeBoundingBox() const;

  // Serialization and deserialization
  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

private:

  #ifndef NDEBUG
  void checkStateConsistency();
  #endif

  // Format: x0, y0, theta0, x1, y1, theta1, ...
  VectorXs m_q;
  // Format: vx0, vy0, omega0, vx1, vy1, omega1, ...
  VectorXs m_v;
  SparseMatrixsc m_M;
  SparseMatrixsc m_Minv;
  std::vector<bool> m_fixed;
  VectorXu m_geometry_indices;
  std::vector<std::unique_ptr<RigidBody2DGeometry>> m_geometry;
  std::vector<std::unique_ptr<RigidBody2DForce>> m_forces;
  std::vector<RigidBody2DStaticPlane> m_planes;
  std::vector<PlanarPortal> m_planar_portals;

};

#endif
