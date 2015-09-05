// Ball2DUtilities.h
//
// Breannan Smith
// Last updated: 09/04/2015

#ifndef BALL_2D_UTILITIES_H
#define BALL_2D_UTILITIES_H

#include <memory>

class UnconstrainedMap;

namespace Ball2DUtilities
{

  void serialize( const std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::ostream& output_stream );
  std::unique_ptr<UnconstrainedMap> deserializeUnconstrainedMap( std::istream& input_stream );

}

#endif
