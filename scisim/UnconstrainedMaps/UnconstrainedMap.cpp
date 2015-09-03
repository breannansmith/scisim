// UnconstrainedMap.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "UnconstrainedMap.h"

#include <iostream>

UnconstrainedMap::~UnconstrainedMap()
{}

void UnconstrainedMap::linearInertialConfigurationUpdate( const VectorXs& q0, const VectorXs& v0, const scalar& dt, VectorXs& q1 )
{
  std::cerr << "UnconstrainedMap::linearInertialConfigurationUpdate not implemented for " << name() << std::endl;
  std::exit( EXIT_FAILURE );
}
