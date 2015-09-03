// Sobogus.h
//
// Breannan Smith
// Last updated: 09/03/2015

// TODO: Break this into three classes
// TODO: 2D solver has no need to cache H0 and H1

#ifndef SOBOGUS_H
#define SOBOGUS_H

#include <memory>
#include "scisim/Math/MathDefines.h"
#include "scisim/ConstrainedMaps/bogus/Ball2DSobogusInterface.h"
#include "scisim/ConstrainedMaps/bogus/RigidBody2DSobogusInterface.h"
#include "scisim/ConstrainedMaps/bogus/RigidBody3DSobogusInterface.h"
#include "FrictionSolver.h"

class ImpactOperator;
class FrictionOperator;
class Constraint;
class FlowableSystem;

enum class SobogusSolverType{ Balls2D, RigidBody2D, RigidBodies3D };

// TODO: Have a 2D and 3D version of this
// TODO: Rename m_f_in to m_p0
// TODO: Rename m_H0 to generalized contact basis
class SobogusFrictionProblem final
{

public:

  SobogusFrictionProblem( const SobogusSolverType& solver_type );
  SobogusFrictionProblem( const SobogusSolverType& solver_type, const std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, VectorXs& masses, const VectorXs& q0, const VectorXs& v0, const VectorXs& CoR, const VectorXs& mu, const VectorXs& nrel, const VectorXs& drel );

  void initialize( const std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, VectorXs& masses, const VectorXs& q0, const VectorXs& v0, const VectorXs& CoR, const VectorXs& mu, const VectorXs& nrel, const VectorXs& drel );

  // TODO: Get working with warm starts (setting r correctly)
  void solve( const std::vector<std::unique_ptr<Constraint>>& active_set, const unsigned max_iters, const unsigned eval_every, const scalar& tol, VectorXs& alpha, VectorXs& beta, VectorXs& f, VectorXs& vout, bool& succeeded, scalar& error, unsigned& num_iterations );

  scalar computeError( const VectorXs& r );

  void flattenMass( const SparseMatrixsc& M, VectorXs& masses );

private:

  void initialize2D( const std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const VectorXs& masses, const VectorXs& q0, const VectorXs& v0, const VectorXs& CoR, const VectorXs& mu, const VectorXs& nrel, const VectorXs& drel );
  void solve2D( const std::vector<std::unique_ptr<Constraint>>& active_set, const unsigned max_iters, const unsigned eval_every, const scalar& tol, VectorXs& alpha, VectorXs& beta, VectorXs& f, VectorXs& vout, bool& succeeded, scalar& error, unsigned& num_iterations );

  void initializeRigidBody2D( const std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const VectorXs& masses, const VectorXs& q0, const VectorXs& v0, const VectorXs& CoR, const VectorXs& mu, const VectorXs& nrel, const VectorXs& drel );
  void solveRigidBody2D( const std::vector<std::unique_ptr<Constraint>>& active_set, const unsigned max_iters, const unsigned eval_every, const scalar& tol, VectorXs& alpha, VectorXs& beta, VectorXs& f, VectorXs& vout, bool& succeeded, scalar& error, unsigned& num_iterations );

  void initialize3D( const std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const VectorXs& masses, const VectorXs& q0, const VectorXs& v0, const VectorXs& CoR, const VectorXs& mu, const VectorXs& nrel, const VectorXs& drel );
  void solve3D( const std::vector<std::unique_ptr<Constraint>>& active_set, const unsigned max_iters, const unsigned eval_every, const scalar& tol, VectorXs& alpha, VectorXs& beta, VectorXs& f, VectorXs& vout, bool& succeeded, scalar& error, unsigned& num_iterations );

  const SobogusSolverType m_solver_type;

  // TODO: Can eleminate these by adding a method to MecheFrictionProblem
  unsigned m_num_bodies;
  unsigned m_num_collisions;

  bogus::RigidBodies3DSobogusInterface m_mfp;
  bogus::Balls2DSobogusInterface m_balls_2d;
  bogus::RigidBody2DSobogusInterface m_rigid_body_2d;

  // Needed for the solve (MecheFrictionProblem does not cache these)
  VectorXs m_f_in;
  VectorXs m_w_in;

  // (Could recompute these to save space, if needed)
  // Needed for extracting alpha and beta after solve
  MatrixXXsr m_H_0_store;
  MatrixXXsr m_H_1_store;

};

class Sobogus final : public FrictionSolver
{

public:

  Sobogus( const SobogusSolverType& solver_type, const unsigned eval_every );
  Sobogus( std::istream& input_stream );
  virtual ~Sobogus() override;

  virtual void solve( const unsigned iteration, const scalar& dt, const FlowableSystem& fsys, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& CoR, const VectorXs& mu, const VectorXs& q0, const VectorXs& v0, std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const unsigned max_iters, const scalar& tol, VectorXs& f, VectorXs& alpha, VectorXs& beta, VectorXs& vout, bool& solve_succeeded, scalar& error ) override;

  virtual unsigned numFrictionImpulsesPerNormal( const unsigned ambient_space_dimensions ) const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual std::string name() const override;

private:

  void flattenMass( const SparseMatrixsc& M, VectorXs& masses );

  const SobogusSolverType m_solver_type;
  const unsigned m_eval_every;

};

#endif
