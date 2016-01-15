// Ball2DSim.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef BALL_2D_SIM_H
#define BALL_2D_SIM_H

#ifdef USE_PYTHON
#include <Python.h>
#endif

#include "scisim/UnconstrainedMaps/FlowableSystem.h"
#include "scisim/Constraints/ConstrainedSystem.h"
#include "Ball2DState.h"
#include "ConstraintCache.h"

class UnconstrainedMap;
class ImpactOperator;
class TeleportedCollision;
class ImpactMap;
class ImpactFrictionMap;
class PythonScripting;
class HDF5File;
class FrictionSolver;
template<typename T> class Rational;

class Ball2DSim final : private FlowableSystem, private ConstrainedSystem
{

public:

  Ball2DSim() = default;
  Ball2DSim( const Ball2DSim& ) = default;
  Ball2DSim& operator=( const Ball2DSim& ) = default;
  Ball2DSim( Ball2DSim&& ) = default;
  Ball2DSim& operator=( Ball2DSim&& ) = default;
  virtual ~Ball2DSim() override = default;

  Ball2DState& state();
  const Ball2DState& state() const;

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

  // Computes the number of collisions in the current state and the total amount of penetration
  void computeNumberOfCollisions( std::map<std::string,unsigned>& collision_counts, std::map<std::string,scalar>& collision_depths );

  // Flow using only an unconstrained map
  void flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap );

  // Flow using an unconstrained map and an impact map
  void flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap, ImpactOperator& iop, const scalar& CoR, ImpactMap& imap );

  // Flow using an unconstrained map, an impact-friction map
  void flow( PythonScripting& call_back, const unsigned iteration, const Rational<std::intmax_t>& dt, UnconstrainedMap& umap, const scalar& CoR, const scalar& mu, FrictionSolver& solver, ImpactFrictionMap& ifmap );

  void writeBinaryState( HDF5File& output_file ) const;

  void serialize( std::ostream& output_stream ) const;
  void deserialize( std::istream& input_stream );

private:

  // TODO: Most of these methods don't need to be methods...

  void updatePeriodicBoundaryConditionsStartOfStep( const unsigned next_iteration, const scalar& dt );
  void enforcePeriodicBoundaryConditions();

  void getTeleportedBallBallCenters( const VectorXs& q, const TeleportedCollision& teleported_collision, Vector2s& x0, Vector2s& x1 ) const;
  bool teleportedBallBallCollisionHappens( const VectorXs& q, const TeleportedCollision& teleported_collision ) const;
  void generateTeleportedBallBallCollision( const VectorXs& q0, const VectorXs& q1, const VectorXs& r, const TeleportedCollision& teleported_collision, std::vector<std::unique_ptr<Constraint>>& active_set ) const;

  void computeBallBallActiveSetSpatialGrid( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  void computeBallDrumActiveSetAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;
  void computeBallPlaneActiveSetAllPairs( const VectorXs& q0, const VectorXs& q1, std::vector<std::unique_ptr<Constraint>>& active_set ) const;

  Ball2DState m_state;
  ConstraintCache m_constraint_cache;

};

#endif
