// RigidBody2DUtilities.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef RIGID_BODY_2D_UTILITIES_H
#define RIGID_BODY_2D_UTILITIES_H

#include <memory>

class UnconstrainedMap;

namespace RigidBody2DUtilities
{

  void serialize( const std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::ostream& output_stream );
  std::unique_ptr<UnconstrainedMap> deserializeUnconstrainedMap( std::istream& input_stream );

}

#endif
