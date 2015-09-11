// RigidBody2DUtilities.cpp
//
// Breannan Smith
// Last updated: 09/10/2015

#include "RigidBody2DUtilities.h"

#include <cassert>
#include <ostream>
#include <iostream>

#include "SCISim/StringUtilities.h"
#include "SCISim/UnconstrainedMaps/UnconstrainedMap.h"

#include "SymplecticEulerMap.h"

void RigidBody2DUtilities::serialize( const std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::ostream& output_stream )
{
  assert( output_stream.good() );

  if( unconstrained_map != nullptr )
  {
    StringUtilities::serializeString( unconstrained_map->name(), output_stream );
    unconstrained_map->serialize( output_stream );
  }
  else
  {
    StringUtilities::serializeString( "NULL", output_stream );
  }
}

std::unique_ptr<UnconstrainedMap> RigidBody2DUtilities::deserializeUnconstrainedMap( std::istream& input_stream )
{
  assert( input_stream.good() );

  std::unique_ptr<UnconstrainedMap> unconstrained_map{ nullptr };

  const std::string integrator_name{ StringUtilities::deserializeString( input_stream ) };
  if( "symplectic_euler" == integrator_name )
  {
    unconstrained_map.reset( new SymplecticEulerMap{ input_stream } );
  }
  else
  {
    std::cerr << "Deserialization not supported for: " << integrator_name << std::endl;
    std::exit( EXIT_FAILURE );
  }

  return unconstrained_map;
}
