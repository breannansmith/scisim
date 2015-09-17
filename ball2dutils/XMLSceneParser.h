// XMLSceneParser.h
//
// Breannan Smith
// Last updated: 09/05/2015

// TODO: Createa container class for the state that is parsed in here

#ifndef XML_SCENE_PARSER_H
#define XML_SCENE_PARSER_H

#include "scisim/Math/MathDefines.h"
#include <string>
#include <memory>
#include <cstdint>

class Ball2D;
class StaticDrum;
class StaticPlane;
class PlanarPortal;
class UnconstrainedMap;
class ImpactOperator;
class ImpactMap;
class FrictionSolver;
class ImpactFrictionMap;
class Ball2DForce;
template<typename T> class Rational;

namespace XMLSceneParser
{

  bool parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback_name, std::vector<Ball2D>& balls, std::vector<StaticDrum>& drums, std::vector<StaticPlane>& planes, std::vector<PlanarPortal>& planar_portals, std::unique_ptr<UnconstrainedMap>& integrator, std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator, std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map, std::vector<std::unique_ptr<Ball2DForce>>& forces, bool& camera_set, Eigen::Vector2d& camera_center, double& camera_scale_factor, unsigned& fps, bool& render_at_fps, bool& lock_camera );

}

#endif
