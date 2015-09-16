// RigidBody3DUtilities.h
//
// Breannan Smith
// Last updated: 09/14/2015

#ifndef RIGID_BODY_3D_UTILITIES_H
#define RIGID_BODY_3D_UTILITIES_H

#include <memory>

class UnconstrainedMap;

namespace RigidBody3DUtilities
{

  void serialize( const std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::ostream& output_stream );
  std::unique_ptr<UnconstrainedMap> deserializeUnconstrainedMap( std::istream& input_stream );

}

#endif
