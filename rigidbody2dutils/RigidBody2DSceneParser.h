#ifndef RIGID_BODY_2D_SCENE_PARSER_H
#define RIGID_BODY_2D_SCENE_PARSER_H

#include "scisim/Math/MathDefines.h"

#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/Math/Rational.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"

#include "rigidbody2d/RigidBody2DState.h"

#include <iosfwd>
#include <memory>

struct SimSettings final
{
  std::string scripting_callback_name;

  RigidBody2DState state;

  std::string dt_string;
  Rational<std::intmax_t> dt;
  scalar end_time = SCALAR_INFINITY;

  scalar CoR = 1.0;
  scalar mu = 0.0;

  std::unique_ptr<UnconstrainedMap> unconstrained_map = nullptr;
  std::unique_ptr<ImpactOperator> impact_operator = nullptr;
  std::unique_ptr<ImpactMap> impact_map = nullptr;
  std::unique_ptr<FrictionSolver> friction_solver = nullptr;
  std::unique_ptr<ImpactFrictionMap> if_map = nullptr;
};

struct RenderSettings final
{
  bool camera_set = false;
  Eigen::Vector2d camera_center = Eigen::Vector2d::Zero();
  double camera_scale_factor = 1.0;
  unsigned fps = 30;
  bool render_at_fps = false;
  bool lock_camera = false;

  // int num_ball_subdivs = 64;
  // int num_drum_subdivs = 64;
  int num_aa_samples = 4;

  // std::vector<PlaneRenderSettings> plane_render_settings;
  // std::vector<DrumRenderSettings> drum_render_settings;
  // std::vector<PortalRenderSettings> portal_render_settings;
};

namespace RigidBody2DSceneParser
{

  bool parseXMLSceneFile( const std::string& file_name, SimSettings& sim_settings );

  bool parseXMLSceneFile( const std::string& file_name, SimSettings& sim_settings, RenderSettings& render_settings );

}

#endif
