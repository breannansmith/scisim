#ifndef RIGID_BODY_2D_SCENE_PARSER_H
#define RIGID_BODY_2D_SCENE_PARSER_H

#include "scisim/Math/MathDefines.h"

#include <iosfwd>
#include <memory>

class RigidBody2DState;
class UnconstrainedMap;
class ImpactOperator;
class FrictionSolver;
class ImpactFrictionMap;
class ImpactMap;
class CameraSettings2D;
template<typename T> class Rational;

struct RenderSettings final
{
  // bool camera_set;
  // Eigen::Vector2d camera_center;
  // double camera_scale_factor;
  unsigned fps;
  bool render_at_fps;
  bool lock_camera;

  // int num_ball_subdivs = 64;
  // int num_drum_subdivs = 64;
  int num_aa_samples = 4;

  // std::vector<PlaneRenderSettings> plane_render_settings;
  // std::vector<DrumRenderSettings> drum_render_settings;
  // std::vector<PortalRenderSettings> portal_render_settings;
};

struct SimSettings final
{};

namespace RigidBody2DSceneParser
{

  bool parseXMLSceneFile( const std::string& file_name, SimSettings& sim_settings, RenderSettings& render_settings );

  bool parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback, RigidBody2DState& sim_state, std::unique_ptr<UnconstrainedMap>& unconstrained_map, std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator, std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map, CameraSettings2D& camera_settings );

}

#endif
