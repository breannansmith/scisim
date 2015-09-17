// RigidBody3DSceneParser.h
//
// Breannan Smith
// Last updated: 09/16/2015

#ifndef RIGID_BODY_3D_SCENE_PARSER_H
#define RIGID_BODY_3D_SCENE_PARSER_H

#include "scisim/Math/MathDefines.h"
#include <string>
#include <memory>
#include <cstdint>

class RigidBody3DState;
class UnconstrainedMap;
class ImpactOperator;
class FrictionSolver;
class ImpactFrictionMap;
class RenderingState;
template<typename T> class Rational;

namespace RigidBody3DSceneParser
{

  bool parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback, RigidBody3DState& sim_state, std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map, RenderingState& rendering_state );

}

#endif
