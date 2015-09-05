// VerletMap.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "VerletMap.h"

#include "SCISim/UnconstrainedMaps/FlowableSystem.h"

VerletMap::VerletMap()
{}

VerletMap::VerletMap( std::istream& input_stream )
{
  // No state to read in
}

VerletMap::~VerletMap()
{}

void VerletMap::flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 )
{
  // Ugh, aliasing problems :(.
  // TODO: Rework computaitons to avoid aliasing
  assert( &q0 != &q1 );
  assert( &v0 != &v1 );

  // Sanity checks on size of inputs/outputs
  assert( fsys.nqdofs() == fsys.nvdofs() );
  const int N{ fsys.nqdofs() };
  assert( N == q0.size() ); assert( N == v0.size() );
  assert( N == q1.size() ); assert( N == v1.size() );

  assert( iteration > 0 );
  const scalar next_time{ iteration * dt };

  // If system changed size since last call, resize all storage
  VectorXs F{ N };
  //VectorXs m_tmp1(N);

  // F = (1/2) h a0 = (1/2) h M^{-1} F(q0)
  fsys.computeForce( q0, v0, next_time, F );
  F = fsys.Minv() * F;
  F *= 0.5 * dt;

  // v1 = (1/2) h a0
  v1 = F;

  // q1 = (1/2) h^2 a0
  q1 = dt * F;

  // q1 = q0 + h v0 + (1/2) h^2 a0
  q1 += q0 + dt * v0;

  //F = (1/2) h a1 = (1/2) h M^{-1} F(q1)
  fsys.computeForce( q1, v0, next_time, F );
  F = fsys.Minv() * F;
  F *= 0.5 * dt;

  // v1 = (1/2) h ( a0 + a1 )
  v1 += F;

  // v1 = v0 + (1/2) h ( a0 + a1 )
  v1 += v0;
}

std::string VerletMap::name() const
{
  return "verlet";
}

void VerletMap::serialize( std::ostream& output_stream ) const
{
  // No state to serialize
}
