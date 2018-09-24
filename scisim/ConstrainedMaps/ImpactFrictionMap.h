#ifndef IMPACT_FRICTION_MAP_H
#define IMPACT_FRICTION_MAP_H

#include "scisim/Math/MathDefines.h"

#include <memory>

class ScriptingCallback;
class FlowableSystem;
class ConstrainedSystem;
class UnconstrainedMap;
class FrictionSolver;
class Constraint;

#ifdef USE_HDF5
class HDF5File;
#endif

class ImpactFrictionMap
{

public:

  ImpactFrictionMap( const ImpactFrictionMap& ) = delete;
  ImpactFrictionMap( ImpactFrictionMap&& ) = delete;
  ImpactFrictionMap& operator=( const ImpactFrictionMap& ) = delete;
  ImpactFrictionMap& operator=( ImpactFrictionMap&& ) = delete;

  virtual ~ImpactFrictionMap() = 0;

  virtual void flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 ) = 0;

  // Resets data used in warm starting to initial setting
  virtual void resetCachedData() = 0;

  virtual void serialize( std::ostream& output_stream ) const = 0;

  virtual std::string name() const = 0;

  #ifdef USE_HDF5
  virtual void exportForcesNextStep( HDF5File& output_file ) = 0;
  #endif

protected:

  ImpactFrictionMap() = default;

  // TODO: Move these shared routines out of here
  // Support routines shared by various ImpactFrictionMap implementations
  //static bool noImpulsesToKinematicGeometry( const FlowableSystem& fsys, const SparseMatrixsc& N, const VectorXs& alpha, const SparseMatrixsc& D, const VectorXs& beta, const VectorXs& v0 );
  #ifdef USE_HDF5
  static void exportConstraintForcesToBinaryFile( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& contact_bases, const VectorXs& alpha, const VectorXs& beta, const scalar& dt, HDF5File& output_file );
  #endif

  static bool constraintSetShouldConserveMomentum( const std::vector<std::unique_ptr<Constraint>>& cons );
  static bool constraintSetShouldConserveAngularMomentum( const std::vector<std::unique_ptr<Constraint>>& cons );

};

#endif
