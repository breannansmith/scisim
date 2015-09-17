// SymplecticEulerMap.cpp
//
// Breannan Smith
// Last updated: 09/11/2015

#include "SymplecticEulerMap.h"

#include "scisim/UnconstrainedMaps/FlowableSystem.h"

SymplecticEulerMap::SymplecticEulerMap( std::istream& input_stream )
{
  // No state to read in
}

void SymplecticEulerMap::flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 )
{
  assert( iteration > 0 );
  const scalar next_time{ iteration * dt };

  // Use q1 as temporary storage for the force
  fsys.computeForce( q0, v0, next_time, q1 );
  // Velocity update
  v1 = v0 + dt * fsys.Minv() * q1;
  // Position update
  q1 = q0 + dt * v1;
}

std::string SymplecticEulerMap::name() const
{
  return "symplectic_euler";
}

void SymplecticEulerMap::serialize( std::ostream& output_stream ) const
{
  // No state to serialize
}
