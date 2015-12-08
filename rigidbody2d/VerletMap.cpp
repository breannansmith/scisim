// VerletMap.cpp
//
// Breannan Smith
// Last updated: 12/08/2015

#include "VerletMap.h"

#include "scisim/UnconstrainedMaps/FlowableSystem.h"

VerletMap::VerletMap( std::istream& input_stream )
{
  // No state to read in
}

static void zeroForcesOnKinematicBodies( const FlowableSystem& fsys, VectorXs& F )
{
  // TODO: Probably faster to store an explicit list of fixed bodies
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
  // Ensure that kinematic bodies are not expecting to get integrated
  #ifndef NDEBUG
  {
    const unsigned nbodies{ fsys.numBodies() };
    for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
    {
      if( fsys.isKinematicallyScripted( bdy_idx ) )
      {
        assert( ( v0.segment<3>( 3 * bdy_idx ).array() == 0.0 ).all() );
      }
    }
  }
  #endif

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
  zeroForcesOnKinematicBodies( fsys, F );

  // v1/2 = v0 + (1/2) h a0
  v1 = v0 + 0.5 * dt * fsys.Minv() * F;

  // q1 = q0 + h v0 + (1/2) h^2 a0
  q1 = q0 + dt * v1;

  // F( q1 )
  fsys.computeForce( q1, v0, next_time, F );
  zeroForcesOnKinematicBodies( fsys, F );

  // v1 = v0 + (1/2) h ( a0 + a1 )
  v1 += 0.5 * dt * fsys.Minv() * F;

  // Ensure that kinematic bodies were not integrated
  #ifndef NDEBUG
  {
    const unsigned nbodies{ fsys.numBodies() };
    for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
    {
      if( fsys.isKinematicallyScripted( bdy_idx ) )
      {
        assert( ( q0.segment<3>( 3 * bdy_idx ).array() == q1.segment<3>( 3 * bdy_idx ).array() ).all() );
        assert( ( v1.segment<3>( 3 * bdy_idx ).array() == 0.0 ).all() );
      }
    }
  }
  #endif
}

std::string VerletMap::name() const
{
  return "verlet";
}

void VerletMap::serialize( std::ostream& output_stream ) const
{
  // No state to serialize
}
