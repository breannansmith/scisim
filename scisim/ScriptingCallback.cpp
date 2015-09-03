// ScriptingCallback.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "ScriptingCallback.h"

ScriptingCallback::~ScriptingCallback() = default;

void ScriptingCallback::restitutionCoefficientCallback( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& cor )
{
  if( name() == "" )
  {
    return;
  }
  restitutionCoefficient( active_set, cor );
  assert( ( cor.array() >= 0.0 ).all() );
  assert( ( cor.array() <= 1.0 ).all() );
}

void ScriptingCallback::frictionCoefficientCallback( const std::vector<std::unique_ptr<Constraint>>& active_set, VectorXs& mu )
{
  if( name() == "" )
  {
    return;
  }
  frictionCoefficient( active_set, mu );
  assert( ( mu.array() >= 0.0 ).all() );
}

void ScriptingCallback::startOfSimCallback()
{
  if( name() == "" )
  {
    return;
  }
  startOfSim();
}

void ScriptingCallback::endOfSimCallback()
{
  if( name() == "" )
  {
    return;
  }
  endOfSim();
}

void ScriptingCallback::startOfStepCallback( const unsigned next_iteration, const Rational<std::intmax_t>& dt )
{
  if( name() == "" )
  {
    return;
  }
  startOfStep( next_iteration, dt );
}

void ScriptingCallback::endOfStepCallback( const unsigned next_iteration, const Rational<std::intmax_t>& dt )
{
  if( name() == "" )
  {
    return;
  }
  endOfStep( next_iteration, dt );
}
