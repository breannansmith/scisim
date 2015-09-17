// FrictionSolver.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef FRICTION_SOLVER_H
#define FRICTION_SOLVER_H

#include <memory>

#include "scisim/Math/MathDefines.h"

class Constraint;
class FlowableSystem;

class FrictionSolver
{

public:

  virtual ~FrictionSolver() = 0;

  virtual void solve( const unsigned iteration, const scalar& dt, const FlowableSystem& fsys, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& CoR, const VectorXs& mu, const VectorXs& q0, const VectorXs& v0, std::vector<std::unique_ptr<Constraint>>& active_set, const MatrixXXsc& contact_bases, const unsigned max_iters, const scalar& tol, VectorXs& f, VectorXs& alpha, VectorXs& beta, VectorXs& v2, bool& solve_succeeded, scalar& error ) = 0;

  virtual unsigned numFrictionImpulsesPerNormal( const unsigned ambient_space_dimensions ) const = 0;

  virtual void serialize( std::ostream& output_stream ) const = 0;

  virtual std::string name() const = 0;

};

#endif
