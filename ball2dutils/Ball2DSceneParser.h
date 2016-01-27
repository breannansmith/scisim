// Ball2DSceneParser.h
//
// Breannan Smith
// Last updated: 09/21/2015

#ifndef BALL_2D_SCENE_PARSER_H
#define BALL_2D_SCENE_PARSER_H

#include "scisim/Math/MathDefines.h"
#include <iosfwd>
#include <memory>
#include <cstdint>

class Ball2DState;
class UnconstrainedMap;
class ImpactOperator;
class ImpactMap;
class FrictionSolver;
class ImpactFrictionMap;
template<typename T> class Rational;

namespace Ball2DSceneParser
{

  bool parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback_name, Ball2DState& state, std::unique_ptr<UnconstrainedMap>& integrator, std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator, std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map );

  bool parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback_name, Ball2DState& state, std::unique_ptr<UnconstrainedMap>& integrator, std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator, std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map, bool& camera_set, Eigen::Vector2d& camera_center, double& camera_scale_factor, unsigned& fps, bool& render_at_fps, bool& lock_camera );

}

#endif
