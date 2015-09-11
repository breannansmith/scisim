// RigidBody2DUtilities.h
//
// Breannan Smith
// Last updated: 09/10/2015

// TODO: Merge serialize with the equiavlents in ThreeDRigidBodySim and TwoDBallSim and pull into SCISim Utilities
//       ... can't really do deserialize, as that depends on stuff local to rigid body2d and things

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
