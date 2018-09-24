#include "VerletMap.h"

#include "scisim/UnconstrainedMaps/FlowableSystem.h"

VerletMap::VerletMap( std::istream& /*input_stream*/ )
{
  // No state to read in
}

void VerletMap::flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 )
{
  // Sanity checks on size of inputs/outputs
  assert( fsys.nqdofs() == fsys.nvdofs() );
  const int N{ fsys.nqdofs() };
  assert( N == q0.size() ); assert( N == v0.size() );
  assert( N == q1.size() ); assert( N == v1.size() );

  assert( iteration > 0 );
  const scalar next_time{ iteration * dt };

  // F( q0 )
  VectorXs F{ N };
  fsys.computeForce( q0, v0, next_time, F );

  // v1/2 = v0 + (1/2) h a0
  v1 = v0 + 0.5 * dt * fsys.Minv() * F;

  // q1 = q0 + h v0 + (1/2) h^2 a0
  q1 = q0 + dt * v1;

  // F( q1 )
  fsys.computeForce( q1, v0, next_time, F );

  // v1 = v0 + (1/2) h ( a0 + a1 )
  v1 += 0.5 * dt * fsys.Minv() * F;
}

std::string VerletMap::name() const
{
  return "verlet";
}

void VerletMap::serialize( std::ostream& /*output_stream*/ ) const
{
  // No state to serialize
}
