// ConstrainedMapUtilities.h
//
// Breannan Smith
// Last updated: 09/03/2015

// TODO: Move to exceptions instead of std::exit
// TODO: Replace serialization of strings with serialization of enum classes

#ifndef CONSTRAINED_MAP_UTILITIES_H
#define CONSTRAINED_MAP_UTILITIES_H

#include <memory>

#include "scisim/Math/MathDefines.h"

class ImpactOperator;
class FrictionOperator;
class FrictionSolver;
class ImpactMap;
class ImpactFrictionMap;

namespace ConstrainedMapUtilities
{

  void serialize( const std::unique_ptr<ImpactOperator>& impact_operator, std::ostream& output_stream );
  void serialize( const std::unique_ptr<FrictionOperator>& friction_operator, std::ostream& output_stream );
  void serialize( const std::unique_ptr<FrictionSolver>& friction_solver, std::ostream& output_stream );
  void serialize( const std::unique_ptr<ImpactMap>& impact_map, std::ostream& output_stream );
  void serialize( const std::unique_ptr<ImpactFrictionMap>& impact_friction_map, std::ostream& output_stream );

  std::unique_ptr<ImpactOperator> deserializeImpactOperator( std::istream& input_stream );
  std::unique_ptr<FrictionOperator> deserializeFrictionOperator( std::istream& input_stream );
  std::unique_ptr<FrictionSolver> deserializeFrictionSolver( std::istream& input_stream );
  std::unique_ptr<ImpactMap> deserializeImpactMap( std::istream& input_stream );
  std::unique_ptr<ImpactFrictionMap> deserializeImpactFrictionMap( std::istream& input_stream );

}

#endif
