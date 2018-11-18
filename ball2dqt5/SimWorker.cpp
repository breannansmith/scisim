#include "SimWorker.h"

#include <algorithm>
#include <array>

#include <QApplication>
#include <QDebug>

SimWorker::SimWorker()
: m_sim0()
, m_sim()
, m_integrator0()
, m_integrator()
, m_scripting()
, m_iteration( 0 )
, m_end_time( std::numeric_limits<scalar>::max() )
, m_H0( 0.0 )
, m_p0( Vector2s::Zero() )
, m_L0( 0.0 )
, m_delta_H0( 0.0 )
, m_delta_p0( Vector2s::Zero() )
, m_delta_L0( 0.0 )
, m_ball_colors()
, m_color_gen( 0.0, 1.0 )
, m_ball_color_gen( 1337 )
, m_plane_render_settings0()
, m_drum_render_settings0()
, m_portal_render_settings0()
, m_plane_render_settings()
, m_drum_render_settings()
, m_portal_render_settings()
, m_steps_per_frame( 1 )
{}

Vector3s SimWorker::generateColor()
{
  Vector3s color( 1.0, 1.0, 1.0 );
  // Generate colors until we get one with a luminance in [0.1, 0.9]
  while( ( 0.2126 * color.x() + 0.7152 * color.y() + 0.0722 * color.z() ) > 0.9 ||
        ( 0.2126 * color.x() + 0.7152 * color.y() + 0.0722 * color.z() ) < 0.1 )
  {
    color.x() = m_color_gen( m_ball_color_gen );
    color.y() = m_color_gen( m_ball_color_gen );
    color.z() = m_color_gen( m_ball_color_gen );
  }
  return color;
}

static void ballInsertCallback( void* context, int num_balls )
{
  static_cast<SimWorker*>(context)->insertBallCallback(num_balls);
}

void SimWorker::insertBallCallback( const int num_balls )
{
  m_ball_colors.conservativeResize( 3 * num_balls );
  m_ball_colors.segment<3>( 3 * num_balls - 3) = generateColor();
}

static void planeDeleteCallback( void* context, int plane_idx )
{
  static_cast<SimWorker*>(context)->deletePlaneCallback(plane_idx);
}

void SimWorker::deletePlaneCallback( const int plane_idx )
{
  // For each plane renderer
  for( int rndr_idx = 0; rndr_idx < int(m_plane_render_settings.size()); rndr_idx++ )
  {
    PlaneRenderSettings& settings = m_plane_render_settings[rndr_idx];
    // Flag the renderer for deletion if it matches the marked plane
    if( settings.idx == plane_idx )
    {
      settings.idx = -1;
    }
    // Otherwise, if the index is greater than the marked plane, decrement the index
    else if( settings.idx > plane_idx )
    {
      settings.idx--;
    }
  }
  // Delete any flagged renderers
  m_plane_render_settings.erase(
    std::remove_if( m_plane_render_settings.begin(), m_plane_render_settings.end(),
                    []( const PlaneRenderSettings& settings ){ return settings.idx == -1; } ),
                    m_plane_render_settings.end() );
}

static std::string xmlFilePath( const std::string& xml_file_name )
{
  std::string path;
  std::string file_name;
  StringUtilities::splitAtLastCharacterOccurence( xml_file_name, path, file_name, '/' );
  if( file_name.empty() )
  {
    using std::swap;
    swap( path, file_name );
  }
  return path;
}

SimWorker::SimWorker( const QString& xml_scene_file_name, SimSettings& sim_settings, RenderSettings& render_settings )
: SimWorker()
{
  // Push the initial state and cache it to allow resets
  m_sim.state() = std::move( sim_settings.state );
  m_sim.clearConstraintCache();
  m_sim0 = m_sim;

  // Push the initial integrator state and cache it to allow resets
  m_integrator0 = sim_settings.integrator;
  m_integrator = m_integrator0;

  // Save the time and iteration related quantities
  m_iteration = 0;
  m_end_time = sim_settings.end_time;
  assert( m_end_time > 0.0 );

  // Compute the initial energy, momentum, and angular momentum
  m_H0 = m_sim.state().computeTotalEnergy();
  m_p0 = m_sim.state().computeMomentum();
  m_L0 = m_sim.state().computeAngularMomentum();
  // Trivially there is no change in energy, momentum, and angular momentum until we take a timestep
  m_delta_H0 = 0.0;
  m_delta_p0 = Vector2s::Zero();
  m_delta_L0 = 0.0;

  // Generate a random color for each ball
  m_ball_color_gen = std::mt19937_64( 1337 );
  m_ball_colors.resize( 3 * m_sim.state().nballs() );
  for( int i = 0; i < m_ball_colors.size(); i += 3 )
  {
    m_ball_colors.segment<3>( i ) = generateColor();
  }

  m_plane_render_settings = std::move( render_settings.plane_render_settings );
  m_plane_render_settings0 = m_plane_render_settings;
  m_drum_render_settings = std::move( render_settings.drum_render_settings );
  m_drum_render_settings0 = m_drum_render_settings;
  m_portal_render_settings = std::move( render_settings.portal_render_settings );
  m_portal_render_settings0 = m_portal_render_settings;

  // Initialize the scripting callback
  {
    PythonScripting new_scripting{ xmlFilePath( xml_scene_file_name.toStdString() ), sim_settings.scripting_callback_name };
    swap( m_scripting, new_scripting );
  }

  // User-provided start of simulation python callback
  m_scripting.setState( m_sim.state() );
  m_scripting.startOfSimCallback();
  m_scripting.forgetState();

  // Register UI callbacks for Python scripting
  m_scripting.registerBallInsertCallback( this, &ballInsertCallback );
  m_scripting.registerPlaneDeleteCallback( this, &planeDeleteCallback );
}

void SimWorker::reset()
{
  m_sim = m_sim0;

  m_integrator = m_integrator0;

  m_iteration = 0;

  m_H0 = m_sim.state().computeTotalEnergy();
  m_p0 = m_sim.state().computeMomentum();
  m_L0 = m_sim.state().computeAngularMomentum();
  m_delta_H0 = 0.0;
  m_delta_p0.setZero();
  m_delta_L0 = 0.0;

  // Reset ball colors, in case the number of balls changed
  m_ball_color_gen = std::mt19937_64( 1337 );
  m_ball_colors.resize( 3 * m_sim.state().nballs() );
  for( int i = 0; i < m_ball_colors.size(); i += 3 )
  {
    m_ball_colors.segment<3>( i ) = generateColor();
  }

  m_plane_render_settings = m_plane_render_settings0;
  m_drum_render_settings = m_drum_render_settings0;
  m_portal_render_settings = m_portal_render_settings0;

  // User-provided start of simulation python callback
  m_scripting.setState( m_sim.state() );
  m_scripting.startOfSimCallback();
  m_scripting.forgetState();

  // Register UI callbacks for Python scripting
  m_scripting.registerBallInsertCallback( this, &ballInsertCallback );
  m_scripting.registerPlaneDeleteCallback( this, &planeDeleteCallback );

  constexpr bool was_reset = true;
  const bool fps_multiple = true;
  const int output_num = 0;
  emit postStep( was_reset, fps_multiple, output_num );
}

void SimWorker::takeStep()
{
  if( m_iteration * scalar( m_integrator.dt() ) >= m_end_time )
  {
    // User-provided end of simulation python callback
    m_scripting.setState( m_sim.state() );
    m_scripting.endOfSimCallback();
    m_scripting.forgetState();
    qInfo( "Simulation complete. Exiting." );
    QApplication::quit();
  }

  const unsigned next_iter{ m_iteration + 1 };

  m_integrator.step( next_iter, m_scripting, m_sim );

  m_iteration++;

  {
    const Ball2DState& state{ m_sim.state() };
    m_delta_H0 = std::max( m_delta_H0, fabs( m_H0 - state.computeTotalEnergy() ) );
    const Vector2s p{ state.computeMomentum() };
    m_delta_p0.x() = std::max( m_delta_p0.x(), fabs( m_p0.x() - p.x() ) );
    m_delta_p0.y() = std::max( m_delta_p0.y(), fabs( m_p0.y() - p.y() ) );
    m_delta_L0 = std::max( m_delta_L0, fabs( m_L0 - state.computeAngularMomentum() ) );
  }

  constexpr bool was_reset = false;
  const bool fps_multiple = m_iteration % m_steps_per_frame == 0;
  const int output_num = m_iteration / m_steps_per_frame;
  emit postStep( was_reset, fps_multiple, output_num );
}

void SimWorker::exportMovieInit()
{
  constexpr bool was_reset = false;
  const bool fps_multiple = m_iteration == 0;
  const int output_num = m_iteration / m_steps_per_frame;
  emit postStep( was_reset, fps_multiple, output_num );
}

void SimWorker::setOutputFPS( const int fps )
{
  if( 1.0 < scalar( m_integrator.dt() * std::intmax_t( fps ) ) )
  {
    emit errorMessage( tr("Requested movie frame rate faster than timestep. Dumping at timestep rate.") );
    m_steps_per_frame = 1;
  }
  else
  {
    const Rational<std::intmax_t> potential_steps_per_frame{ std::intmax_t( 1 ) / ( m_integrator.dt() * std::intmax_t( fps ) ) };
    if( !potential_steps_per_frame.isInteger() )
    {
      if( m_integrator.dt() != Rational<std::intmax_t>{ 0 } )
      {
        emit errorMessage( tr("Warning, timestep and output frequency do not yield an integer number of timesteps for data output. Dumping at timestep rate.") );
      }
      m_steps_per_frame = 1;
    }
    else
    {
      m_steps_per_frame = unsigned( potential_steps_per_frame.numerator() );
    }
  }
}

unsigned SimWorker::iteration() const
{
  return m_iteration;
}

const scalar& SimWorker::endTime() const
{
  return m_end_time;
}

const Integrator& SimWorker::integrator() const
{
  return m_integrator;
}

const Ball2DSim& SimWorker::sim() const
{
  return m_sim;
}

const scalar& SimWorker::deltaH0() const
{
  return m_delta_H0;
}

const Vector2s& SimWorker::deltap0() const
{
  return m_delta_p0;
}

const scalar& SimWorker::deltaL0() const
{
  return m_delta_L0;
}

const VectorXs& SimWorker::ballColors() const
{
  return m_ball_colors;
}

const std::vector<PlaneRenderSettings>& SimWorker::planeRenderSettings() const
{
  return m_plane_render_settings;
}

const std::vector<DrumRenderSettings>& SimWorker::drumRenderSettings() const
{
  return m_drum_render_settings;
}

const std::vector<PortalRenderSettings>& SimWorker::portalRenderSettings() const
{
  return m_portal_render_settings;
}

void SimWorker::computeCameraCenter( bool& sim_empty, Vector4s& bbox ) const
{
  sim_empty = m_sim.state().empty();
  bbox = m_sim.state().computeSimulatedBoundingBox();

  // Expand the bounding box to account for planes
  {
    static constexpr std::array<std::pair<scalar,scalar>, 4> ref_points{
      {{-1.0, 0.5}, {0.0, 0.5}, {0.0, -0.5}, {-1.0, -0.5}}
    };

    for( const PlaneRenderSettings& plane_renderer : m_plane_render_settings )
    {
      const StaticPlane& plane = m_sim.state().staticPlanes()[plane_renderer.idx];
      const Vector2s& n = plane.n();
      const Vector2s& cm = plane.x();

      const Vector2s r = plane_renderer.r.cast<scalar>();

      for( const std::pair<scalar,scalar>& val : ref_points )
      {
        const scalar sx = r.x() * val.first;
        const scalar sy = r.y() * val.second;

        const scalar x = (n.x() * sx - n.y() * sy) + cm.x();
        const scalar y = (n.y() * sx + n.x() * sy) + cm.y();

        bbox(0) = std::min( bbox(0), x );
        bbox(1) = std::max( bbox(1), x );
        bbox(2) = std::min( bbox(2), y );
        bbox(3) = std::max( bbox(3), y );
      }
    }
  }

  // Expand the bounding box to account for all drums
  for( const DrumRenderSettings& drum_renderer : m_drum_render_settings )
  {
    const StaticDrum& drum = m_sim.state().staticDrums()[drum_renderer.idx];
    const scalar r = scalar(drum_renderer.r) + drum.r();
    const scalar& x = drum.x().x();
    const scalar& y = drum.x().y();

    bbox(0) = std::min( bbox(0), x - r );
    bbox(1) = std::max( bbox(1), x + r );
    bbox(2) = std::min( bbox(2), y - r );
    bbox(3) = std::max( bbox(3), y + r );
  }

  // Expand the boudning box to account for all portals
  {
    static constexpr std::array<std::pair<scalar,scalar>, 4> ref_points{
      {{-1.0, -1.0}, {1.0, -1.0}, {1.0, 1.0}, {-1.0, 1.0}}
    };

    for( const PortalRenderSettings& portal_renderer : m_portal_render_settings )
    {
      const scalar r0 = scalar(portal_renderer.thickness);
      const scalar r1 = scalar(portal_renderer.half_width);
      const scalar iw = scalar(portal_renderer.indicator_half_width);
      const PlanarPortal& portal = m_sim.state().planarPortals()[portal_renderer.idx];

      // Center line, plane A
      for( const std::pair<scalar,scalar>& val : ref_points )
      {
        const scalar sx = r0 * val.first;
        const scalar sy = r1 * val.second;
        const scalar c = portal.planeA().n().x();
        const scalar s = portal.planeA().n().y();

        const Vector2s cm = portal.planeA().x() - r0 * portal.planeA().n();

        const scalar x = (c * sx - s * sy) + cm.x();
        const scalar y = (s * sx + c * sy) + cm.y();

        bbox(0) = std::min( bbox(0), x );
        bbox(1) = std::max( bbox(1), x );
        bbox(2) = std::min( bbox(2), y );
        bbox(3) = std::max( bbox(3), y );
      }
      // Indicator, plane A
      for( const std::pair<scalar,scalar>& val : ref_points )
      {
        const scalar sx = iw * val.first;
        const scalar sy = r0 * val.second;
        const scalar c = portal.planeA().n().x();
        const scalar s = portal.planeA().n().y();

        const Vector2s cm = portal.transformedAx() - (2.0 * r0 + iw) * portal.planeA().n();

        const scalar x = (c * sx - s * sy) + cm.x();
        const scalar y = (s * sx + c * sy) + cm.y();

        bbox(0) = std::min( bbox(0), x );
        bbox(1) = std::max( bbox(1), x );
        bbox(2) = std::min( bbox(2), y );
        bbox(3) = std::max( bbox(3), y );
      }
      // Center line, plane B
      for( const std::pair<scalar,scalar>& val : ref_points )
      {
        const scalar sx = r0 * val.first;
        const scalar sy = r1 * val.second;
        const scalar c = portal.planeB().n().x();
        const scalar s = portal.planeB().n().y();

        const Vector2s cm = portal.planeB().x() - r0 * portal.planeB().n();

        const scalar x = (c * sx - s * sy) + cm.x();
        const scalar y = (s * sx + c * sy) + cm.y();

        bbox(0) = std::min( bbox(0), x );
        bbox(1) = std::max( bbox(1), x );
        bbox(2) = std::min( bbox(2), y );
        bbox(3) = std::max( bbox(3), y );
      }
      // Indicator, plane B
      for( const std::pair<scalar,scalar>& val : ref_points )
      {
        const scalar sx = iw * val.first;
        const scalar sy = r0 * val.second;
        const scalar c = portal.planeB().n().x();
        const scalar s = portal.planeB().n().y();

        const Vector2s cm = portal.transformedBx() - (2.0 * r0 + iw) * portal.planeB().n();

        const scalar x = (c * sx - s * sy) + cm.x();
        const scalar y = (s * sx + c * sy) + cm.y();

        bbox(0) = std::min( bbox(0), x );
        bbox(1) = std::max( bbox(1), x );
        bbox(2) = std::min( bbox(2), y );
        bbox(3) = std::max( bbox(3), y );
      }
    }
  }
}
