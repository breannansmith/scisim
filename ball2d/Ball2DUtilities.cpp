// Ball2DUtilities.cpp
//
// Breannan Smith
// Last updated: 09/04/2015

#include "Ball2DUtilities.h"

#include <cassert>
#include "scisim/StringUtilities.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "ball2d/VerletMap.h"

#include <iostream>

void Ball2DUtilities::serialize( const std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::ostream& output_stream )
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

std::unique_ptr<UnconstrainedMap> Ball2DUtilities::deserializeUnconstrainedMap( std::istream& input_stream )
{
  assert( input_stream.good() );

  std::unique_ptr<UnconstrainedMap> unconstrained_map{ nullptr };

  const std::string integrator_name{ StringUtilities::deserializeString( input_stream ) };
  if( "verlet" == integrator_name )
  {
    unconstrained_map.reset( new VerletMap{ input_stream } );
  }
  else
  {
    std::cerr << "Deserialization not supported for: " << integrator_name << std::endl;
    std::exit( EXIT_FAILURE );
  }

  return unconstrained_map;
}
