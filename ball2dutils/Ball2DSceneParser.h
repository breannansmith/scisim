#ifndef BALL_2D_SCENE_PARSER_H
#define BALL_2D_SCENE_PARSER_H

#include "scisim/Math/MathDefines.h"
#include <iosfwd>
#include <memory>

class Ball2DState;
class UnconstrainedMap;
class ImpactOperator;
class ImpactMap;
class FrictionSolver;
class ImpactFrictionMap;
template<typename T> class Rational;

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

struct RenderSettings final
{
  bool camera_set;
  Eigen::Vector2d camera_center;
  double camera_scale_factor;
  unsigned fps;
  bool render_at_fps;
  bool lock_camera;

  std::vector<PlaneRenderSettings> plane_render_settings;
  std::vector<DrumRenderSettings> drum_render_settings;
  std::vector<PortalRenderSettings> portal_render_settings;
};

namespace Ball2DSceneParser
{

  bool parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback_name, Ball2DState& state, std::unique_ptr<UnconstrainedMap>& integrator,
                          std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator,
                          std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map );

  bool parseXMLSceneFile( const std::string& file_name, std::string& scripting_callback_name, Ball2DState& state, std::unique_ptr<UnconstrainedMap>& integrator,
                          std::string& dt_string, Rational<std::intmax_t>& dt, scalar& end_time, std::unique_ptr<ImpactOperator>& impact_operator,
                          std::unique_ptr<ImpactMap>& impact_map, scalar& CoR, std::unique_ptr<FrictionSolver>& friction_solver, scalar& mu, std::unique_ptr<ImpactFrictionMap>& if_map,
                          RenderSettings& render_settings );

}

#endif
