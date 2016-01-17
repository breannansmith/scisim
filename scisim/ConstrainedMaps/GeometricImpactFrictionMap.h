// GeometricImpactFrictionMap.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef GEOMETRIC_IMPACT_FRICTION_MAP_H
#define GEOMETRIC_IMPACT_FRICTION_MAP_H

#include "ImpactFrictionMap.h"

class Constraint;
class HDF5File;
class FrictionSolver;

enum class ImpulsesToCache: std::uint8_t
{
  NONE,
  NORMAL,
  NORMAL_AND_FRICTION
};

class GeometricImpactFrictionMap final : public ImpactFrictionMap
{

public:

  GeometricImpactFrictionMap( const scalar& abs_tol, const unsigned max_iters, const ImpulsesToCache impulses_to_cache );
  explicit GeometricImpactFrictionMap( std::istream& input_stream );

  // TODO: Default constructors, etc
  virtual ~GeometricImpactFrictionMap() override = default;

  virtual void flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 ) override;

  virtual void resetCachedData() override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual std::string name() const override;

  virtual void exportForcesNextStep( HDF5File& output_file ) override;

private:

  // For saving out constraint forces
  void exportConstraintForcesToBinary( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& contact_bases, const VectorXs& alpha, const VectorXs& beta, const scalar& dt );

  // Cached friction impulse from last solve
  VectorXs m_f;

  // SP solver controls
  scalar m_abs_tol;
  unsigned m_max_iters;

  // Controls which portion of the impulse to cache and warm start with
  ImpulsesToCache m_impulses_to_cache;

  // Temporary state for writing constraint forces
  bool m_write_constraint_forces;
  HDF5File* m_constraint_force_stream;

};

#endif
