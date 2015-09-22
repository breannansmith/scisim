// RigidBody2DSim.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef RIGID_BODY_2D_SIM_H
#define RIGID_BODY_2D_SIM_H

#include "scisim/UnconstrainedMaps/FlowableSystem.h"
#include "rigidbody2d/RigidBody2DState.h"

#include "scisim/Constraints/ConstrainedSystem.h"

class UnconstrainedMap;
class ImpactOperator;
class ImpactMap;
class ImpactFrictionMap;
class HDF5File;
class FrictionSolver;

class RigidBody2DSim final : private FlowableSystem, private ConstrainedSystem
{

public:

  RigidBody2DSim();
  RigidBody2DSim( const RigidBody2DSim& other );
  explicit RigidBody2DSim( const RigidBody2DState& state );
  virtual ~RigidBody2DSim() override = default;
  RigidBody2DSim& operator=( RigidBody2DSim other );
  friend void swap( RigidBody2DSim& first, RigidBody2DSim& second );

  RigidBody2DState& state();
  const RigidBody2DState& state() const;

  scalar computeKineticEnergy() const;
  scalar computePotentialEnergy() const;
  scalar computeTotalEnergy() const;
  Vector2s computeTotalMomentum() const;
  scalar computeTotalAngularMomentum() const;

  // Inherited from FlowableSystem

  virtual int nqdofs() const override;
  virtual int nvdofs() const override;
  virtual unsigned numVelDoFsPerBody() const override;
  virtual unsigned ambientSpaceDimensions() const override;

  virtual bool isKinematicallyScripted( const int i ) const override;

  virtual void computeForce( const VectorXs& q, const VectorXs& v, const scalar& t, VectorXs& F ) override;

  virtual const SparseMatrixsc& M() const override;
  virtual const SparseMatrixsc& Minv() const override;
  virtual const SparseMatrixsc& M0() const override;
  virtual const SparseMatrixsc& Minv0() const override;

  virtual void computeMomentum( const VectorXs& v, VectorXs& p ) const override;
  virtual void computeAngularMomentum( const VectorXs& v, VectorXs& L ) const override;

  // Inherited from ConstrainedSystem

  virtual void computeActiveSet( const VectorXs& q0, const VectorXs& qp, std::vector<std::unique_ptr<Constraint>>& active_set ) override;
  virtual void computeImpactBases( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& active_set, MatrixXXsc& impact_bases ) const override;
  virtual void computeContactBases( const VectorXs& q, const VectorXs& v, const std::vector<std::unique_ptr<Constraint>>& active_set, MatrixXXsc& contact_bases ) const override;
  virtual void clearConstraintCache() override;
  virtual void cacheConstraint( const Constraint& constraint, const VectorXs& r ) override;
  virtual void getCachedConstraintImpulse( const Constraint& constraint, VectorXs& r ) const override;

  // Flow using only an unconstrained map
  void flow( const unsigned iteration, const scalar& dt, UnconstrainedMap& umap );

  // Flow using an unconstrained map and an impact map
  void flow( const unsigned iteration, const scalar& dt, UnconstrainedMap& umap, ImpactOperator& iop, const scalar& CoR, ImpactMap& imap );

  // Flow using an unconstrained map, an impact-friction map
  void flow( const unsigned iteration, const scalar& dt, UnconstrainedMap& umap, const scalar& CoR, const scalar& mu, FrictionSolver& solver, ImpactFrictionMap& ifmap );

  void writeBinaryState( HDF5File& output_file ) const;

  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

  void enforcePeriodicBoundaryConditions( VectorXs& q );

private:

  // TODO: Most of these methods don't need to be methods...

  void updatePeriodicBoundaryConditionsStartOfStep( const unsigned next_iteration, const scalar& dt );

  void getTeleportedCollisionCenter( const unsigned portal_index, const bool portal_plane, Vector2s& x ) const;
  void getTeleportedCollisionCenters( const VectorXs& q, const TeleportedCollision& teleported_collision, Vector2s& x0, Vector2s& x1 ) const;
  void dispatchTeleportedNarrowPhaseCollision( const TeleportedCollision& teleported_collision, const std::unique_ptr<RigidBody2DGeometry>& geo0, const std::unique_ptr<RigidBody2DGeometry>& geo1, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  bool teleportedCollisionIsActive( const TeleportedCollision& teleported_collision, const std::unique_ptr<RigidBody2DGeometry>& geo0, const std::unique_ptr<RigidBody2DGeometry>& geo1, const VectorXs& q ) const;

  void computeBodyBodyActiveSetSpatialGrid( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  void computeBodyPlaneActiveSetAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;

  RigidBody2DState m_state;
  //ConstraintCache m_constraint_cache;

};

#endif
