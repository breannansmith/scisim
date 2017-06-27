#include "ProjectionSolvers.h"

scalar ProjectionSolvers::computeNewTheta( const scalar& theta0 )
{
  return 0.5 * ( - theta0 * theta0 + theta0 * sqrt( theta0 * theta0 + 4.0 ) );
}

scalar ProjectionSolvers::computeNewBeta( const scalar& theta0, const scalar& theta1 )
{
  assert( theta0 != 0.0 || theta1 != 0.0 );
  return theta0 * ( 1.0 - theta0 ) / ( theta0 * theta0 + theta1 );
}
