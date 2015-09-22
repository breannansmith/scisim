// StabilizedImpactFrictionMap.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef STABILIZED_IMPACT_FRICTION_MAP_H
#define STABILIZED_IMPACT_FRICTION_MAP_H

#include "ImpactFrictionMap.h"

class Constraint;
class HDF5File;
class FrictionSolver;

class StabilizedImpactFrictionMap : public ImpactFrictionMap
{

public:

  StabilizedImpactFrictionMap( const scalar& abs_tol, const unsigned max_iters );
  explicit StabilizedImpactFrictionMap( std::istream& input_stream );
  virtual ~StabilizedImpactFrictionMap() override;

  virtual void flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 ) override;

  // Resets data used in warm starting to initial setting
  virtual void resetCachedData() override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual std::string name() const override;

  virtual void exportForcesNextStep( HDF5File& output_file ) override;

private:

  // For saving out constraint forces
  [[noreturn]] void exportConstraintForcesToBinary( const unsigned ambient_space_dims, const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const SparseMatrixsc& N, const VectorXs& alpha, const SparseMatrixsc& D, const VectorXs& beta, const scalar& dt );

  // Cached friction impulse from last solve
  VectorXs m_f;

  // SP solver controls
  scalar m_abs_tol;
  unsigned m_max_iters;

  // Temporary state for writing constraint forces
  bool m_write_constraint_forces;
  HDF5File* m_constraint_force_stream;

};

#endif
