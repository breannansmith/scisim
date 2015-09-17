// ImpactFrictionMap.h
//
// Breannan Smith
// Last updated: 09/03/2015

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
class HDF5File;

class ImpactFrictionMap
{

public:

  virtual ~ImpactFrictionMap() = 0;

  virtual void flow( ScriptingCallback& call_back, FlowableSystem& fsys, ConstrainedSystem& csys, UnconstrainedMap& umap, FrictionSolver& friction_solver, const unsigned iteration, const scalar& dt, const scalar& CoR, const scalar& mu_default, const VectorXs& q0, const VectorXs& v0, VectorXs& q1, VectorXs& v1 ) = 0;

  // Resets data used in warm starting to initial setting
  virtual void resetCachedData() = 0;

  virtual void serialize( std::ostream& output_stream ) const = 0;

  virtual std::string name() const = 0;

  virtual void exportForcesNextStep( HDF5File& output_file ) = 0;

protected:

  // TODO: Move these shared routines out of here
  // Support routines shared by various ImpactFrictionMap implementations
  // TODO: replace ...Supported strings with class enum type
  static bool impactOperatorSupported( const std::string& impact_operator_name );
  static bool frictionOperatorSupported( const std::string& friction_operator_name );
  static bool noImpulsesToKinematicGeometry( const FlowableSystem& fsys, const SparseMatrixsc& N, const VectorXs& alpha, const SparseMatrixsc& D, const VectorXs& beta, const VectorXs& v0 );
  static void exportConstraintForcesToBinaryFile( const VectorXs& q, const std::vector<std::unique_ptr<Constraint>>& constraints, const MatrixXXsc& contact_bases, const VectorXs& alpha, const VectorXs& beta, const scalar& dt, HDF5File& output_file );

  static bool constraintSetShouldConserveMomentum( const std::vector<std::unique_ptr<Constraint>>& cons );
  static bool constraintSetShouldConserveAngularMomentum( const std::vector<std::unique_ptr<Constraint>>& cons );

};

#endif
