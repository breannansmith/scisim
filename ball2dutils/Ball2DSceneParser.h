#ifndef BALL_2D_SCENE_PARSER_H
#define BALL_2D_SCENE_PARSER_H

#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/Math/MathDefines.h"
#include "scisim/Math/Rational.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"

#include "ball2d/Ball2DState.h"
#include "ball2d/Integrator.h"

#include <iosfwd>
#include <memory>

struct PlaneRenderSettings final
{
  PlaneRenderSettings( const int idx_in, const Eigen::Vector2f& r_in, const Eigen::Vector3f& color_in )
  : idx( idx_in )
  , r( r_in )
  , color( color_in )
  {}

  int idx;
  Eigen::Vector2f r;
  Eigen::Vector3f color;
};

struct DrumRenderSettings final
{
  DrumRenderSettings( const int idx_in, const float& r_in, const Eigen::Vector3f& color_in )
  : idx( idx_in )
  , r( r_in )
  , color( color_in )
  {}

  int idx;
  float r;
  Eigen::Vector3f color;
};

struct PortalRenderSettings final
{
  PortalRenderSettings( const int idx_in, const float& thickness_in, const float& width_in, const float& indicator_width_in, const Eigen::Vector3f& color_in )
  : idx( idx_in )
  , thickness( 0.5f * thickness_in )
  , half_width( 0.5f * width_in )
  , indicator_half_width( 0.5f * indicator_width_in )
  , color( color_in )
  {}

  int idx;
  float thickness;
  float half_width;
  float indicator_half_width;
  Eigen::Vector3f color;
};

struct SimSettings final
{
  Ball2DState state;
  Integrator integrator;
  std::string scripting_callback_name;
  std::string dt_string;
  scalar end_time = SCALAR_INFINITY;
};

struct RenderSettings final
{
  bool camera_set = false;
  Eigen::Vector2d camera_center = Eigen::Vector2d::Zero();
  double camera_scale_factor = 1.0;
  unsigned fps = 30;
  bool render_at_fps = false;
  bool lock_camera = false;

  int num_ball_subdivs = 64;
  int num_drum_subdivs = 64;
  int num_aa_samples = 4;

  std::vector<PlaneRenderSettings> plane_render_settings;
  std::vector<DrumRenderSettings> drum_render_settings;
  std::vector<PortalRenderSettings> portal_render_settings;
};

namespace Ball2DSceneParser
{
  bool parseXMLSceneFile( const std::string& file_name, SimSettings& sim_settings );
  bool parseXMLSceneFile( const std::string& file_name, SimSettings& sim_settings, RenderSettings& render_settings );
}

#endif
