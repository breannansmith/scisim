// ScriptingCallback.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

// TODO: Don't check if the name is empty, instead have a subclass that simply does nothing

#include "ScriptingCallback.h"

ScriptingCallback::~ScriptingCallback() = default;

void ScriptingCallback::restitutionCoefficientCallback( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& cor )
{
  if( name().empty() )
  {
    return;
  }
  restitutionCoefficient( active_set, cor );
  assert( ( cor.array() >= 0.0 ).all() );
  assert( ( cor.array() <= 1.0 ).all() );
}

void ScriptingCallback::frictionCoefficientCallback( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& mu )
{
  if( name().empty() )
  {
    return;
  }
  frictionCoefficient( active_set, mu );
  assert( ( mu.array() >= 0.0 ).all() );
}

void ScriptingCallback::startOfSimCallback()
{
  if( name().empty() )
  {
    return;
  }
  startOfSim();
}

void ScriptingCallback::endOfSimCallback()
{
  if( name().empty() )
  {
    return;
  }
  endOfSim();
}

void ScriptingCallback::startOfStepCallback( const unsigned next_iteration, const Rational<std::intmax_t>& dt )
{
  if( name().empty() )
  {
    return;
  }
  startOfStep( next_iteration, dt );
}

void ScriptingCallback::endOfStepCallback( const unsigned next_iteration, const Rational<std::intmax_t>& dt )
{
  if( name().empty() )
  {
    return;
  }
  endOfStep( next_iteration, dt );
}
