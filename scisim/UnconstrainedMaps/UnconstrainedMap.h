#ifndef UNCONSTRAINED_MAP_H
#define UNCONSTRAINED_MAP_H

#include <memory>

#include "scisim/Math/MathDefines.h"

class FlowableSystem;

class UnconstrainedMap
{

public:

  UnconstrainedMap( const UnconstrainedMap& ) = delete;
  UnconstrainedMap( UnconstrainedMap&& ) = delete;
  UnconstrainedMap& operator=( const UnconstrainedMap& ) = delete;
  UnconstrainedMap& operator=( UnconstrainedMap&& ) = delete;

  virtual ~UnconstrainedMap() = 0;

  // q0: Initial configuration
  // v0: Initial velocity
  // fsys: Computes forces, multiplies by mass matrix, etc
  // TODO: Verify whether iteration is start or next across implementations.
  // iteration: Iteration we are stepping to (for example, 3 means this step is from time 3 * dt to 4 * dt)
  // dt: Timestep
  // q1: Result of map acting on (q0,v0)
  // v1: Result of map acting on (q0,v0)
  virtual void flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 ) = 0;

  virtual std::string name() const = 0;

  virtual void serialize( std::ostream& output_stream ) const = 0;

  virtual std::unique_ptr<UnconstrainedMap> clone() const = 0;

protected:

  UnconstrainedMap() = default;

};

#endif
