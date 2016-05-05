// StabilizedImpactFrictionMap.h
//
// Breannan Smith
// Last updated: 11/16/2015

#ifndef STABILIZED_IMPACT_FRICTION_MAP_H
#define STABILIZED_IMPACT_FRICTION_MAP_H

#include "ImpactFrictionMap.h"

class Constraint;
class FrictionSolver;

#ifdef USE_HDF5
class HDF5File;
#endif

class StabilizedImpactFrictionMap final : public ImpactFrictionMap
{

public:

  StabilizedImpactFrictionMap( const scalar& abs_tol, const unsigned max_iters, const bool external_warm_start_alpha, const bool external_warm_start_beta );
  explicit StabilizedImpactFrictionMap( std::istream& input_stream );

  virtual ~StabilizedImpactFrictionMap() override = default;

  virtual void flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR_default, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 ) override;

  // Resets data used in warm starting to initial setting
  virtual void resetCachedData() override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual std::string name() const override;

  #ifdef USE_HDF5
  virtual void exportForcesNextStep( HDF5File& output_file ) override;
  #endif

private:

  // For saving out constraint forces
  void exportConstraintForcesToBinary( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& contact_bases, const VectorXs& alpha, const VectorXs& beta, const scalar& dt );

  // Cached friction impulse from last solve
  VectorXs m_f;

  // SP solver controls
  scalar m_abs_tol;
  unsigned m_max_iters;

  // TODO: Update warm starting to be consistent with GeometricImpactFrictionMap
  // If true, initialize solve with alpha/beta from last time step, otherwise initialize alpha/beta to zero
  bool m_external_warm_start_alpha;
  bool m_external_warm_start_beta;

  #ifdef USE_HDF5
  // Temporary state for writing constraint forces
  bool m_write_constraint_forces;
  HDF5File* m_constraint_force_stream;
  #endif

};

#endif
