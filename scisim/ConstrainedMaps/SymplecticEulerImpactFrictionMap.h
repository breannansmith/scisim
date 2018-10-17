#ifndef SYMPLECTIC_EULER_IMPACT_FRICTION_MAP_H
#define SYMPLECTIC_EULER_IMPACT_FRICTION_MAP_H

#include "ImpactFrictionMap.h"

#include "ImpulsesToCache.h"

class Constraint;
class FrictionSolver;

#ifdef USE_HDF5
class HDF5File;
#endif

class SymplecticEulerImpactFrictionMap final : public ImpactFrictionMap
{

public:

  SymplecticEulerImpactFrictionMap( const scalar& abs_tol, const unsigned max_iters, const ImpulsesToCache impulses_to_cache, const bool stabilize, const scalar& penetration_threshold );
  SymplecticEulerImpactFrictionMap( const scalar& abs_tol, const unsigned max_iters, const ImpulsesToCache impulses_to_cache, const bool stabilize, const scalar& penetration_threshold, const VectorXs& f );
  explicit SymplecticEulerImpactFrictionMap( std::istream& input_stream );

  SymplecticEulerImpactFrictionMap( const SymplecticEulerImpactFrictionMap& ) = delete;
  SymplecticEulerImpactFrictionMap( SymplecticEulerImpactFrictionMap&& ) = delete;
  SymplecticEulerImpactFrictionMap& operator=( const SymplecticEulerImpactFrictionMap& ) = delete;
  SymplecticEulerImpactFrictionMap& operator=( SymplecticEulerImpactFrictionMap&& ) = delete;

  virtual ~SymplecticEulerImpactFrictionMap() override = default;

  virtual void flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 ) override;

  virtual void resetCachedData() override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual std::string name() const override;

  #ifdef USE_HDF5
  virtual void exportForcesNextStep( HDF5File& output_file ) override;
  #endif

  virtual std::unique_ptr<ImpactFrictionMap> clone() const override;

private:

  #ifdef USE_HDF5
  // For saving out constraint forces
  void exportConstraintForcesToBinary( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& contact_bases, const VectorXs& alpha, const VectorXs& beta, const scalar& dt );
  #endif

  // Cached impulses from last solve
  VectorXs m_f;

  // Solver controls
  scalar m_abs_tol;
  unsigned m_max_iters;
  bool m_stabilize;
  scalar m_penetration_threshold;

  // Controls which portion of the impulse to cache and warm start with
  ImpulsesToCache m_impulses_to_cache;

  #ifdef USE_HDF5
  // Temporary state for writing constraint forces
  bool m_write_constraint_forces;
  HDF5File* m_constraint_force_stream;
  #endif

};

#endif
