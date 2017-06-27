#ifndef PROJECTION_SOLVER_H
#define PROJECTION_SOLVER_H

#include "scisim/Math/MathDefines.h"

#ifndef NDEBUG
#include "scisim/Math/MathUtilities.h"
#endif

enum class ProjectionSolveStatus
{
  Success,
  MaxItersExceeded
};

struct ProjectionSolveResults final
{
  ProjectionSolveStatus status;
  scalar achieved_tolerance;
  unsigned num_iterations;
};

namespace ProjectionSolvers
{

  scalar computeNewTheta( const scalar& theta0 );

  scalar computeNewBeta( const scalar& theta0, const scalar& theta1 );

  template<typename SparseMatrix, typename Projection, typename Termination, typename Objective, typename Gradient, typename Multiplication>
  void APGD( const Projection& project, const Termination& term, const Objective& objective, const Gradient& grad, const Multiplication& mult, const scalar& tol, const unsigned max_iters, const SparseMatrix& A, const VectorXs& b, VectorXs& x0, ProjectionSolveResults& results )
  {
    // Verify that all operations are consistent with the sparse matrix's storage order
    static_assert( !SparseMatrix::IsRowMajor == Objective::columnMajor(), "Error, sparse matrix type and objective evaluation function have inconsistent storage orders." );
    static_assert( !SparseMatrix::IsRowMajor == Gradient::columnMajor(), "Error, sparse matrix type and gradient evaluation function have inconsistent storage orders." );
    static_assert( !SparseMatrix::IsRowMajor == Multiplication::columnMajor(), "Error, sparse matrix type and matrix multiplication function have inconsistent storage orders." );
    // Sanity check input sizes
    assert( A.rows() == A.cols() );
    assert( A.rows() == x0.size() );
    assert( b.size() == x0.size() );
    assert( MathUtilities::isSymmetric( A, 1.0e-6 ) );
    // Must have a non-negative tolerance to terminate
    assert( tol >= 0.0 );

    // Ensure a feasible initial iterate in case the warm start returns immediately
    project( x0 );

    VectorXs g{ b.size() };
    VectorXs y1{ b.size() };
    // Store some iterate in x1
    VectorXs x1{ VectorXs::Ones( b.size() ) };

    VectorXs y0{ x0 };
    scalar theta0{ 1.0 };
    assert( ( x0.array() != x1.array() ).any() );
    g = x0 - x1;
    mult( A, g, y1 );
    // Initial estimate of the Lipschitz constant
    scalar Lk{ y1.norm() / ( x0 - x1 ).norm() };
    assert( Lk != 0.0 );
    scalar tk{ 1.0 / Lk };

    scalar best_residual{ SCALAR_INFINITY };
    VectorXs best_solution;
    results.status = ProjectionSolveStatus::MaxItersExceeded;
    unsigned iteration;
    for( iteration = 0; iteration < max_iters; ++iteration )
    {
      // Determine if we should terminate
      {
        // Store the gradient in x1
        grad( A, b, x0, x1 );
        const scalar current_residual{ term( x0, x1 ) };
        if( current_residual < best_residual )
        {
          best_residual = current_residual;
          best_solution = x0;
        }
        if( best_residual <= tol )
        {
          results.status = ProjectionSolveStatus::Success;
          break;
        }
      }

      // Evaluate the gradient
      grad( A, b, y0, g );
      // Attempt a step in the negative gradient direction
      x1 = y0 - tk * g;
      project( x1 );
      // Backtrack if needed
      const scalar objective_y0{ objective( A, b, y0 ) };
      while( true )
      {
        const scalar lhs{ objective( A, b, x1 ) - objective_y0 };
        y1 = x1 - y0;
        const scalar rhs{ g.dot( y1 ) + 0.5 * Lk * ( y1 ).squaredNorm() };
        if( lhs <= rhs )
        {
          break;
        }
        Lk = 2.0 * Lk;
        assert( Lk != 0.0 );
        tk = 1.0 / Lk;
        x1 = y0 - tk * g;
        project( x1 );
      }
      scalar theta1{ computeNewTheta( theta0 ) };
      const scalar beta1{ computeNewBeta( theta0, theta1 ) };
      y0 = x1 - x0;
      y1 = x1 + beta1 * y0;
      // If momentum is hurting progress, restart
      if( g.dot( y0 ) > 0.0 )
      {
        y1 = x1;
        theta1 = 1.0;
      }
      // Slightly increase the step size
      Lk = 0.9 * Lk;
      tk = 1.0 / Lk;
      // Propagate new values
      x1.swap( x0 );
      y1.swap( y0 );
      using std::swap;
      swap( theta0, theta1 );
    }

    results.achieved_tolerance = best_residual;
    results.num_iterations = iteration;
    x0.swap( best_solution );
  }

}

#endif
