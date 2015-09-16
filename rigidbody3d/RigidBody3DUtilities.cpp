// RigidBody3DUtilities.cpp
//
// Breannan Smith
// Last updated: 09/14/2015

#include "RigidBody3DUtilities.h"

#include <cassert>
#include "scisim/StringUtilities.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"

#include "rigidbody3d/UnconstrainedMaps/DMVMap.h"
#include "rigidbody3d/UnconstrainedMaps/SplitHamMap.h"

#include <iostream>

void RigidBody3DUtilities::serialize( const std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::ostream& output_stream )
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

std::unique_ptr<UnconstrainedMap> RigidBody3DUtilities::deserializeUnconstrainedMap( std::istream& input_stream )
{
  assert( input_stream.good() );

  std::unique_ptr<UnconstrainedMap> unconstrained_map;

  const std::string integrator_name{ StringUtilities::deserializeString( input_stream ) };
  // TODO: Create an integrator enum to simplify this code
  if( "dmv" == integrator_name )
  {
    unconstrained_map.reset( new DMVMap );
  }
  else if( "split_ham" == integrator_name )
  {
    unconstrained_map.reset( new SplitHamMap );
  }
  else if( "NULL" == integrator_name )
  {
    unconstrained_map.reset( nullptr );
  }
  else
  {
    std::cerr << "Deserialization not supported for: " << integrator_name << std::endl;
    std::exit( EXIT_FAILURE );
  }

  return unconstrained_map;
}
