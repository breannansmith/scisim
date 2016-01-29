// RigidBody3DSim.h
//
// Breannan Smith
// Last updated: 10/01/2015

#ifndef RIGID_BODY_3D_SIM_H
#define RIGID_BODY_3D_SIM_H

#ifdef USE_PYTHON
#include <Python.h>
#endif

#include <fstream>

#include "scisim/UnconstrainedMaps/FlowableSystem.h"
#include "scisim/Constraints/ConstrainedSystem.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"

#include "RigidBody3DState.h"
#include "ConstraintCache.h"

class UnconstrainedMap;
class ImpactOperator;
class ImpactFrictionMap;
class RigidBodyBox;
class RigidBodySphere;
class RigidBodyStaple;
class RigidBodyTriangleMesh;
class AABB;
class TeleportedCollision;
class HDF5File;
class FrictionSolver;
class PythonScripting;
template<typename T> class Rational;

class RigidBody3DSim final : private FlowableSystem, private ConstrainedSystem
{

public:

  RigidBody3DSim();
  virtual ~RigidBody3DSim() override = default;

  void setState( const RigidBody3DState& state );
  const RigidBody3DState& getState() const;
  RigidBody3DState& getState();

  // TODO: Move this to arrays
  void computeBoundingBox( scalar& minx, scalar& maxx, scalar& miny, scalar& maxy, scalar& minz, scalar& maxz ) const;
  void computeBoundingSphere( scalar& radius, Vector3s& center ) const;

  Vector3s computeTotalMomentum() const;
  Vector3s computeTotalAngularMomentum() const;

  scalar computeKineticEnergy() const;

  scalar computePotentialEnergy();

  scalar computeTotalEnergy();

  bool empty() const;

  // Inherited from FlowableSystem

  virtual int nqdofs() const override;
  virtual int nvdofs() const override;
  virtual unsigned numVelDoFsPerBody() const override;
  virtual unsigned ambientSpaceDimensions() const override;

  virtual bool isKinematicallyScripted( const int i ) const override;

  virtual void computeForce( const VectorXs& q, const VectorXs& v, const scalar& t, VectorXs& F ) override;

  virtual void linearInertialConfigurationUpdate( const VectorXs& q0, const VectorXs& v0, const scalar& dt, VectorXs& q1 ) const override;

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
  virtual bool constraintCacheEmpty() const override;

  // Computes the number of collisions in the current state and the total amount of penetration
  void computeNumberOfCollisions( std::map<std::string,unsigned>& collision_counts, std::map<std::string,scalar>& collision_depths, std::map<std::string,scalar>& overlap_volumes );

  // Flow using only an unconstrained map
  void flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap );

  // Flow using an unconstrained map and an impact map
  void flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap, ImpactOperator& imap, const scalar& CoR );

  // Flow using an unconstrained map, an impact map, and a friction map
  void flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap, const scalar& CoR, const scalar& mu, FrictionSolver& solver, ImpactFrictionMap& ifmap );

  const RigidBody3DState& state() const;

  RigidBody3DState& state();

  #ifndef USE_HDF5
  [[noreturn]]
  #endif
  void writeBinaryState( HDF5File& output_file ) const;

  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

  ImpactMap& impactMap();

private:

  void enforcePeriodicBoundaryConditions();

  void boxBoxNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const RigidBodyBox& box0, const RigidBodyBox& box1, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  [[noreturn]] void boxSphereNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const RigidBodyBox& box, const RigidBodySphere& sphere, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  void sphereSphereNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const RigidBodySphere& sphere0, const RigidBodySphere& sphere1, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  void stapleStapleNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const RigidBodyStaple& staple0, const RigidBodyStaple& staple1, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  void meshMeshNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const RigidBodyTriangleMesh& mesh0, const RigidBodyTriangleMesh& mesh1, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  void dispatchNarrowPhaseCollision( const unsigned first_body, const unsigned second_body, const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  bool collisionIsActive( const unsigned first_body, const unsigned second_body, const VectorXs& q0, const VectorXs& q1 ) const;

  void generateAABBs( std::vector<AABB>& aabbs, const VectorXs& q );

  bool teleportedCollisionHappens( const VectorXs& q, const TeleportedCollision& teleported_collision ) const;
  void getTeleportedCollisionCenters( const VectorXs& q, const TeleportedCollision& teleported_collision, Vector3s& x0, Vector3s& x1 ) const;
  void generateTeleportedCollision( const VectorXs& q, const TeleportedCollision& teleported_collision, std::vector<std::unique_ptr<Constraint>>& active_set ) const;

  void computeActiveSetBodyBodySpatialGrid( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set );
  //void computeActiveSetBodyBodyAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;

  void computeBodyPlaneActiveSetAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  void computeBodyCylinderActiveSetAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;

  RigidBody3DState m_sim_state;
  ImpactMap m_impact_map;
  ConstraintCache m_constraint_cache;

};

#endif
