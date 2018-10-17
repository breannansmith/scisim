#include "VerletMap.h"

#include "scisim/UnconstrainedMaps/FlowableSystem.h"

VerletMap::VerletMap( std::istream& /*input_stream*/ )
{
  // No state to read in
}

static void zeroForcesOnKinematicBodies( const FlowableSystem& fsys, VectorXs& F )
{
  const unsigned nbodies{ fsys.numBodies() };
  for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
  {
    if( fsys.isKinematicallyScripted( bdy_idx ) )
    {
      F.segment<3>( 3 * bdy_idx ).setZero();
    }
  }
}

void VerletMap::flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 )
{
  // Sanity checks on size of inputs/outputs
  assert( fsys.nqdofs() == fsys.nvdofs() );
  const int N{ fsys.nqdofs() };
  assert( N == q0.size() ); assert( N == v0.size() );
  assert( N == q1.size() ); assert( N == v1.size() );

  assert( iteration > 0 );
  const scalar start_time{ ( iteration - 1 ) * dt };

  // F( q0 )
  VectorXs F{ N };
  fsys.computeForce( q0, v0, start_time, F );
  zeroForcesOnKinematicBodies( fsys, F );

  // v1/2 = v0 + (1/2) h a0
  v1 = v0 + 0.5 * dt * fsys.Minv() * F;

  // q1 = q0 + h v0 + (1/2) h^2 a0
  q1 = q0 + dt * v1;

  // F( q1 )
  fsys.computeForce( q1, v0, start_time, F );
  zeroForcesOnKinematicBodies( fsys, F );

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

std::unique_ptr<UnconstrainedMap> VerletMap::clone() const
{
  // No state to copy
  return std::make_unique<VerletMap>();
}
