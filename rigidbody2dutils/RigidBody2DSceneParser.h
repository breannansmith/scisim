// RigidBody2DSceneParser.h
//
// Breannan Smith
// Last updated: 09/11/2015

#ifndef RIGID_BODY_2D_SCENE_PARSER_H
#define RIGID_BODY_2D_SCENE_PARSER_H

#include "scisim/Math/MathDefines.h"
#include <string>
#include <memory>
#include <cstdint>

class RigidBody2DState;
class UnconstrainedMap;
class ImpactOperator;
class FrictionSolver;
class ImpactFrictionMap;
class ImpactMap;
class CameraSettings2D;
template<typename T> class Rational;

namespace RigidBody2DSceneParser
{

  bool parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback, RigidBody2DState& sim_state, std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator, std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map, CameraSettings2D& camera_settings );

}

#endif
