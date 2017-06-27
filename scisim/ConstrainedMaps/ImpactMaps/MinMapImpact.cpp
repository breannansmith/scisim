#include "MinMapImpact.h"

static scalar minMapInfinityNorm( const VectorXs& x, const VectorXs& y )
{
  assert( x.size() == y.size() );
  scalar inf_norm{ 0.0 };
  for( int i = 0; i < x.size(); ++i )
  {
    using std::min;
    using std::max;
    inf_norm = max( inf_norm, fabs( min( x(i), y(i) ) ) );
  }
  return inf_norm;
}

scalar MinMapImpact::operator()( const VectorXs& alpha, const VectorXs& grad_objective ) const
{
  return minMapInfinityNorm( alpha, grad_objective );
}
