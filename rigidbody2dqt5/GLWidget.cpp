#include "GLWidget.h"

#include <QApplication>
#include <QFontDatabase>
#include <QMatrix4x4>
#include <QOpenGLFunctions_3_3_Core>
#include <QPainter>
#include <QWheelEvent>

#include <cassert>
#include <iomanip>

#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"

#include "rigidbody2d/BoxGeometry.h"
#include "rigidbody2d/CircleGeometry.h"

GLWidget::GLWidget( QWidget* parent, const QSurfaceFormat& format )
: QOpenGLWidget( parent )
, m_f( nullptr )
, m_axis_shader()
, m_circle_shader()
, m_plane_shader()
// , m_annulus_shader()
, m_rectangle_shader()
, m_w( 1280 )
, m_h( 720 )
, m_display_scale( 1.0 )
, m_center_x( 0.0 )
, m_center_y( 0.0 )
, m_render_at_fps( false )
, m_lock_camera( false )
, m_last_pos()
, m_left_mouse_button_pressed( false )
, m_right_mouse_button_pressed( false )
, m_body_colors()
, m_color_gen( 0.0, 1.0 )
, m_body_color_gen( 1337 )
, m_display_precision( 0 )
, m_display_HUD( true )
, m_movie_dir_name()
, m_movie_dir()
, m_output_frame( 0 )
, m_output_fps()
, m_steps_per_frame()
, m_unconstrained_map( nullptr )
, m_impact_operator( nullptr )
, m_friction_solver( nullptr )
, m_if_map( nullptr )
, m_imap( nullptr )
, m_scripting()
, m_iteration( 0 )
, m_dt( 0, 1 )
, m_end_time( SCALAR_INFINITY )
, m_CoR( 1.0 )
, m_mu( 0.0 )
, m_sim0()
, m_sim()
, m_H0( 0.0 )
, m_p0( Vector2s::Zero() )
, m_L0( 0.0 )
, m_delta_H0( 0.0 )
, m_delta_p0( Vector2s::Zero() )
, m_delta_L0( 0.0 )
, m_plane_render_settings()
// , m_drum_render_settings()
// , m_portal_render_settings()
, m_half_num_circle_subdivs( 32 )
// , m_num_drum_subdivs( 32 )
, m_num_aa_samples( format.samples() )
{
  this->setFormat( format );
}

GLWidget::~GLWidget()
{
  if( m_f != nullptr )
  {
    // makeCurrent();
    m_axis_shader.cleanup();
    m_circle_shader.cleanup();
    m_plane_shader.cleanup();
    // m_annulus_shader.cleanup();
    m_rectangle_shader.cleanup();
    // doneCurrent();
    assert( checkGLErrors() );
  }
}

QSize GLWidget::minimumSizeHint() const
{
  return QSize{ 50, 50 };
}

QSize GLWidget::sizeHint() const
{
  return QSize{ m_w, m_h };
}

int GLWidget::sampleCount() const
{
  return m_num_aa_samples;
}

static int computeTimestepDisplayPrecision( const Rational<std::intmax_t>& dt, const std::string& dt_string )
{
  if( dt_string.find( '.' ) != std::string::npos )
  {
    return int( StringUtilities::computeNumCharactersToRight( dt_string, '.' ) );
  }
  else
  {
    std::string converted_dt_string;
    std::stringstream ss;
    ss << std::fixed << scalar( dt );
    ss >> converted_dt_string;
    return int( StringUtilities::computeNumCharactersToRight( converted_dt_string, '.' ) );
  }
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

Vector3s GLWidget::generateColor()
{
  Vector3s color( 1.0, 1.0, 1.0 );
  // Generate colors until we get one with a luminance in [0.1, 0.9]
  while( ( 0.2126 * color.x() + 0.7152 * color.y() + 0.0722 * color.z() ) > 0.9 ||
         ( 0.2126 * color.x() + 0.7152 * color.y() + 0.0722 * color.z() ) < 0.1 )
  {
    color.x() = m_color_gen( m_body_color_gen );
    color.y() = m_color_gen( m_body_color_gen );
    color.z() = m_color_gen( m_body_color_gen );
  }
  return color;
}

// void GLWidget::insertBallCallback( const int num_balls )
// {
//   m_body_colors.conservativeResize( 3 * num_balls );
//   m_body_colors.segment<3>( 3 * num_balls - 3) = generateColor();
// }

// static void ballInsertCallback( void* context, int num_balls )
// {
//   static_cast<GLWidget*>(context)->insertBallCallback(num_balls);
// }

// void GLWidget::deletePlaneCallback( const int plane_idx )
// {
//   // For each plane renderer
//   for( int rndr_idx = 0; rndr_idx < int(m_plane_render_settings.size()); rndr_idx++ )
//   {
//     PlaneRenderSettings& settings = m_plane_render_settings[rndr_idx];
//     // Flag the renderer for deletion if it matches the marked plane
//     if( settings.idx == plane_idx )
//     {
//       settings.idx = -1;
//     }
//     // Otherwise, if the index is greater than the marked plane, decrement the index
//     else if( settings.idx > plane_idx )
//     {
//       settings.idx--;
//     }
//   }
//   // Delete any flagged renderers
//   m_plane_render_settings.erase(
//     std::remove_if( m_plane_render_settings.begin(), m_plane_render_settings.end(),
//       []( const PlaneRenderSettings& settings ){ return settings.idx == -1; } ),
//     m_plane_render_settings.end() );
// }

// static void planeDeleteCallback( void* context, int plane_idx )
// {
//   static_cast<GLWidget*>(context)->deletePlaneCallback(plane_idx);
// }

void GLWidget::initializeSimulation( const QString& xml_scene_file_name, const bool& render_on_load, SimSettings& sim_settings, RenderSettings& render_settings )
{
  // Ensure we have a correct combination of maps.
  assert( ( sim_settings.unconstrained_map != nullptr && sim_settings.impact_operator == nullptr &&
            sim_settings.friction_solver == nullptr && sim_settings.if_map == nullptr && sim_settings.impact_map == nullptr ) ||
          ( sim_settings.unconstrained_map != nullptr && sim_settings.impact_operator != nullptr &&
            sim_settings.friction_solver == nullptr && sim_settings.if_map == nullptr && sim_settings.impact_map != nullptr ) ||
          ( sim_settings.unconstrained_map != nullptr && sim_settings.impact_operator == nullptr &&
            sim_settings.friction_solver != nullptr && sim_settings.if_map != nullptr && sim_settings.impact_map == nullptr ) );

  // Set the new maps
  m_unconstrained_map = std::move( sim_settings.unconstrained_map );
  m_impact_operator = std::move( sim_settings.impact_operator );
  m_friction_solver = std::move( sim_settings.friction_solver );
  m_if_map = std::move( sim_settings.if_map );
  m_imap = std::move( sim_settings.impact_map );

  // Initialize the scripting callback
  {
    PythonScripting new_scripting{ xmlFilePath( xml_scene_file_name.toStdString() ), sim_settings.scripting_callback_name };
    swap( m_scripting, new_scripting );
  }

  // Save the coefficient of restitution and the coefficient of friction
  m_CoR = sim_settings.CoR;
  m_mu = sim_settings.mu;

  // Push the new state to the simulation
  m_sim.state() = std::move( sim_settings.state );
  m_sim.clearConstraintCache();

  // Cache the new state locally to allow one to reset a simulation
  m_sim0 = m_sim;

  // Save the timestep and compute related quantities
  m_dt = sim_settings.dt;
  assert( m_dt.positive() );
  m_iteration = 0;
  m_end_time = sim_settings.end_time;
  assert( m_end_time > 0.0 );

  // Update the FPS setting
  if( render_settings.camera_set )
  {
    m_render_at_fps = render_settings.render_at_fps;
    m_output_fps = render_settings.fps;
    m_lock_camera = render_settings.lock_camera;
  }
  assert( m_output_fps > 0 );
  setMovieFPS( m_output_fps );

  // Compute the initial energy, momentum, and angular momentum
  m_H0 = m_sim.computeTotalEnergy();
  m_p0 = m_sim.computeTotalMomentum();
  m_L0 = m_sim.computeTotalAngularMomentum();
  // Trivially there is no change in energy, momentum, and angular momentum until we take a timestep
  m_delta_H0 = 0.0;
  m_delta_p0 = Vector2s::Zero();
  m_delta_L0 = 0.0;

  // Compute the number of characters after the decimal point in the timestep string
  m_display_precision = computeTimestepDisplayPrecision( m_dt, sim_settings.dt_string );

  // Generate a random color for each body
  m_body_color_gen = std::mt19937_64( 1337 );
  m_body_colors.resize( 3 * m_sim.state().nbodies() );
  for( int bdy_idx = 0; bdy_idx < int(m_sim.state().nbodies()); bdy_idx++ )
  {
    if( !m_sim.state().fixed( bdy_idx ) )
    {
      m_body_colors.segment<3>( 3 * bdy_idx ) = generateColor();
    }
    else
    {
      m_body_colors.segment<3>( 3 * bdy_idx ) << 0.2, 0.2, 0.2;
    }
  }

  // Reset the output movie option
  m_movie_dir_name = QString{};
  m_movie_dir = QDir{};

  const bool lock_backup{ m_lock_camera };
  m_lock_camera = false;

  if( !render_settings.camera_set )
  {
    centerCamera( false );
  }
  else
  {
    m_center_x = float(render_settings.camera_center.x());
    m_center_y = float(render_settings.camera_center.y());
    m_display_scale = float(render_settings.camera_scale_factor);
  }

  m_lock_camera = lock_backup;

  // User-provided start of simulation python callback
  m_scripting.setState( m_sim.state() );
  m_scripting.startOfSimCallback();
  m_scripting.forgetState();

  // Register UI callbacks for Python scripting
  // m_scripting.registerBallInsertCallback( this, &ballInsertCallback );
  // m_scripting.registerPlaneDeleteCallback( this, &planeDeleteCallback );

  m_plane_render_settings = std::move( render_settings.plane_render_settings );
  // m_drum_render_settings = std::move( render_settings.drum_render_settings );
  // m_portal_render_settings = std::move( render_settings.portal_render_settings );

  m_half_num_circle_subdivs = render_settings.half_num_circle_subdivs;
  // m_num_drum_subdivs = render_settings.num_drum_subdivs;

  if( m_f != nullptr && render_on_load )
  {
    // Update the global render settings
    m_circle_shader.cleanup();
    m_circle_shader.initialize( m_half_num_circle_subdivs, m_f );

  //   m_annulus_shader.cleanup();
  //   m_annulus_shader.initialize( m_num_drum_subdivs, m_f );

    // Draw the scene
    resizeGL( m_w, m_h );
    update();
  }

  copyRenderState();
}

void GLWidget::copyRenderState()
{
  // Copy the body state over
  {
    Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& circle_data{ m_circle_shader.circleData() };
    circle_data.resize( 7 * m_sim.state().ncircles() );

    Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& rectangle_data{ m_rectangle_shader.data() };
    rectangle_data.resize( 8 * m_sim.state().nboxes() );

    int curCircle = 0;
    int curBox = 0;
    for( int bdyIdx = 0; bdyIdx < int(m_sim.state().nbodies()); bdyIdx++ )
    {
      const std::unique_ptr<RigidBody2DGeometry>& geo = m_sim.state().bodyGeometry( bdyIdx );
      switch( geo->type() )
      {
        case RigidBody2DGeometryType::CIRCLE:
        {
          const CircleGeometry& circle = static_cast<const CircleGeometry&>( *geo.get() );
          // center_of_mass theta radius color ...
          circle_data.segment<3>( 7 * curCircle ) = m_sim.state().q().segment<3>( 3 * bdyIdx ).cast<GLfloat>();
          circle_data( 7 * curCircle + 3 ) = GLfloat( circle.r() );
          circle_data.segment<3>( 7 * curCircle + 4 ) = m_body_colors.segment<3>( 3 * bdyIdx ).cast<GLfloat>();
          curCircle++;
          break;
        }
        case RigidBody2DGeometryType::BOX:
        {
          const BoxGeometry& box = static_cast<const BoxGeometry&>( *geo.get() );
          // center_of_mass theta r0 r1 color ...
          rectangle_data.segment<3>( 8 * curBox ) = m_sim.state().q().segment<3>( 3 * bdyIdx ).cast<GLfloat>();
          rectangle_data.segment<2>( 8 * curBox + 3 ) = box.r().cast<GLfloat>();
          rectangle_data.segment<3>( 8 * curBox + 5 ) = m_body_colors.segment<3>( 3 * bdyIdx ).cast<GLfloat>();
          curBox++;
          break;
        }
      }
    }
  }

  // Boxes
  // {
  //   const std::vector<PlanarPortal>& planar_portals{ m_sim.state().planarPortals() };

  //   for( int render_num = 0; render_num < int(m_portal_render_settings.size()); render_num++ )
  //   {
  //     const int portal_idx = m_portal_render_settings[render_num].idx;
  //     const float& r0 = m_portal_render_settings[render_num].thickness;
  //     const float& r1 = m_portal_render_settings[render_num].half_width;
  //     const float& iw = m_portal_render_settings[render_num].indicator_half_width;
  //     const Eigen::Vector3f& portal_color = m_portal_render_settings[render_num].color;

  //     // Top portal
  //     // Center of mass
  //     rectangle_data(32 * render_num + 0) = float(planar_portals[portal_idx].planeA().x().x())
  //                                           - r0 * float(planar_portals[portal_idx].planeA().n().x());
  //     rectangle_data(32 * render_num + 1) = float(planar_portals[portal_idx].planeA().x().y())
  //                                           - r0 * float(planar_portals[portal_idx].planeA().n().y());
  //     // Orientation
  //     rectangle_data(32 * render_num + 2) = float(std::atan2(planar_portals[portal_idx].planeA().n().y(),
  //                                                 planar_portals[portal_idx].planeA().n().x()));
  //     // Radii
  //     rectangle_data(32 * render_num + 3) = r0;
  //     rectangle_data(32 * render_num + 4) = r1;
  //     // Color
  //     rectangle_data(32 * render_num + 5) = portal_color.x();
  //     rectangle_data(32 * render_num + 6) = portal_color.y();
  //     rectangle_data(32 * render_num + 7) = portal_color.z();

  //     // Top portal center
  //     // Center of mass
  //     rectangle_data(32 * render_num + 8) = float(planar_portals[portal_idx].transformedAx().x())
  //                                           - (r0 + iw) * float(planar_portals[portal_idx].planeA().n().x());
  //     rectangle_data(32 * render_num + 9) = float(planar_portals[portal_idx].transformedAx().y())
  //                                           - (r0 + iw) * float(planar_portals[portal_idx].planeA().n().y());
  //     // Orientation
  //     rectangle_data(32 * render_num + 10) = float(std::atan2(planar_portals[portal_idx].planeA().n().y(),
  //                                                  planar_portals[portal_idx].planeA().n().x()));
  //     // Radii
  //     rectangle_data(32 * render_num + 11) = iw;
  //     rectangle_data(32 * render_num + 12) = r0;
  //     // Color
  //     rectangle_data(32 * render_num + 13) = portal_color.x();
  //     rectangle_data(32 * render_num + 14) = portal_color.y();
  //     rectangle_data(32 * render_num + 15) = portal_color.z();

  //     // Bottom portal
  //     // Center of mass
  //     rectangle_data(32 * render_num + 16) = float(planar_portals[portal_idx].planeB().x().x())
  //                                            - r0 * float(planar_portals[portal_idx].planeB().n().x());
  //     rectangle_data(32 * render_num + 17) = float(planar_portals[portal_idx].planeB().x().y())
  //                                            - r0 * float(planar_portals[portal_idx].planeB().n().y());
  //     // Orientation
  //     rectangle_data(32 * render_num + 18) = float(std::atan2(planar_portals[portal_idx].planeB().n().y(),
  //                                                  planar_portals[portal_idx].planeB().n().x()));
  //     // Radii
  //     rectangle_data(32 * render_num + 19) = r0;
  //     rectangle_data(32 * render_num + 20) = r1;
  //     // Color
  //     rectangle_data(32 * render_num + 21) = portal_color.x();
  //     rectangle_data(32 * render_num + 22) = portal_color.y();
  //     rectangle_data(32 * render_num + 23) = portal_color.z();

  //     // Bottom portal center
  //     // Center of mass
  //     rectangle_data(32 * render_num + 24) = float(planar_portals[portal_idx].transformedBx().x())
  //                                            - (r0 + iw) * float(planar_portals[portal_idx].planeB().n().x());
  //     rectangle_data(32 * render_num + 25) = float(planar_portals[portal_idx].transformedBx().y())
  //                                            - (r0 + iw) * float(planar_portals[portal_idx].planeB().n().y());
  //     // Orientation
  //     rectangle_data(32 * render_num + 26) = float(std::atan2(planar_portals[portal_idx].planeB().n().y(),
  //                                                  planar_portals[portal_idx].planeB().n().x()));
  //     // Radii
  //     rectangle_data(32 * render_num + 27) = iw;
  //     rectangle_data(32 * render_num + 28) = r0;
  //     // Color
  //     rectangle_data(32 * render_num + 29) = portal_color.x();
  //     rectangle_data(32 * render_num + 30) = portal_color.y();
  //     rectangle_data(32 * render_num + 31) = portal_color.z();
  //   }
  // }

  // {
  //   const VectorXs& q{ m_sim.state().q() };
  //   const VectorXs& r{ m_sim.state().r() };

  //   // Find teleported versions of each ball
  //   std::vector<Vector2s> teleported_centers;
  //   std::vector<int> teleported_indices;
  //   // For each periodic boundary
  //   const std::vector<PlanarPortal>& planar_portals{ m_sim.state().planarPortals() };
  //   for( const PlanarPortal& planar_portal : planar_portals )
  //   {
  //     // For each ball
  //     for( unsigned ball_idx = 0; ball_idx < r.size(); ball_idx++ )
  //     {
  //         const Vector2s pos{ q.segment<2>( 2 * ball_idx ) };
  //         const scalar rad{ r( ball_idx ) };
  //         // If the current ball intersect a periodic boudnary
  //         bool intersecting_index;
  //         if( planar_portal.ballTouchesPortal( pos, rad, intersecting_index ) )
  //         {
  //           Vector2s teleported_pos;
  //           planar_portal.teleportBall( pos, rad, teleported_pos );
  //           teleported_centers.emplace_back( teleported_pos );
  //           teleported_indices.emplace_back( ball_idx );
  //         }
  //     }
  //   }

  //   Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& cd{ m_circle_shader.circleData() };
  //   cd.resize( 6 * ( m_sim.state().nballs() + teleported_centers.size() ) );

  //   // Copy over the non-teleported balls
  //   for( unsigned ball_idx = 0; ball_idx < m_sim.state().nballs(); ball_idx++ )
  //   {
  //     // Center of mass, radius, and color
  //     cd.segment<2>( 6 * ball_idx ) = q.segment<2>( 2 * ball_idx ).cast<GLfloat>();
  //     cd( 6 * ball_idx + 2 ) = GLfloat( r( ball_idx ) );
  //     cd.segment<3>( 6 * ball_idx + 3 ) = m_body_colors.segment<3>( 3 * ball_idx ).cast<GLfloat>();
  //   }

  //   // Copy over the teleported balls
  //   for( int tlprtd_idx = 0; tlprtd_idx < int(teleported_centers.size()); tlprtd_idx++ )
  //   {
  //     // Center of mass, radius, and color
  //     cd.segment<2>( 6 * m_sim.state().nballs() + 6 * tlprtd_idx ) = teleported_centers[tlprtd_idx].cast<GLfloat>();
  //     cd( 6 * m_sim.state().nballs() + 6 * tlprtd_idx + 2 ) = GLfloat( r( teleported_indices[tlprtd_idx] ) );
  //     cd.segment<3>( 6 * m_sim.state().nballs() + 6 * tlprtd_idx + 3 ) = m_body_colors.segment<3>( 3 * teleported_indices[tlprtd_idx] ).cast<GLfloat>();
  //   }
  // }

  // Planes
  {
    Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& plane_data{ m_plane_shader.planeData() };
    plane_data.resize( 6 * m_plane_render_settings.size() );
    for( int renderer_num = 0; renderer_num < int(m_plane_render_settings.size()); renderer_num++ )
    {
      const int plane_idx = m_plane_render_settings[renderer_num].idx;
      const RigidBody2DStaticPlane& plane = m_sim.state().planes()[plane_idx];

      plane_data.segment<2>( 6 * plane_idx ) = plane.x().cast<GLfloat>();
      plane_data.segment<2>( 6 * plane_idx + 2 ) = plane.n().cast<GLfloat>();
      plane_data( 6 * plane_idx + 4 ) = m_plane_render_settings[renderer_num].r(0);
      plane_data( 6 * plane_idx + 5 ) = m_plane_render_settings[renderer_num].r(1);
    }
  }

  // // Annuli
  // {
  //   Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& annulus_data{ m_annulus_shader.annulusData() };
  //   annulus_data.resize( 4 * m_sim.state().staticDrums().size() );
  //   for( int renderer_num = 0; renderer_num < int(m_drum_render_settings.size()); renderer_num++ )
  //   {
  //     const int drum_idx = m_drum_render_settings[renderer_num].idx;
  //     const StaticDrum& drum = m_sim.state().staticDrums()[drum_idx];

  //     annulus_data.segment<2>( 4 * drum_idx ) = drum.x().cast<GLfloat>();
  //     annulus_data( 4 * drum_idx + 2 ) = GLfloat(drum.r());
  //     annulus_data( 4 * drum_idx + 3 ) = GLfloat(drum.r()) + m_drum_render_settings[renderer_num].r;
  //   }
  // }
}

void GLWidget::stepSystem()
{
  if( m_iteration * scalar( m_dt ) >= m_end_time )
  {
    // User-provided end of simulation python callback
    m_scripting.setState( m_sim.state() );
    m_scripting.endOfSimCallback();
    m_scripting.forgetState();
    qInfo( "Simulation complete. Exiting." );
    QApplication::quit();
  }

  const unsigned next_iter{ m_iteration + 1 };

  if( m_unconstrained_map == nullptr && m_impact_operator == nullptr && m_imap == nullptr && m_friction_solver == nullptr && m_if_map == nullptr )
  {
    return;
  }
  else if( m_unconstrained_map != nullptr && m_impact_operator == nullptr && m_imap == nullptr && m_friction_solver == nullptr && m_if_map == nullptr )
  {
    m_sim.flow( m_scripting, next_iter, m_dt, *m_unconstrained_map );
  }
  else if( m_unconstrained_map != nullptr && m_impact_operator != nullptr && m_imap != nullptr && m_friction_solver == nullptr && m_if_map == nullptr )
  {
    m_sim.flow( m_scripting, next_iter, m_dt, *m_unconstrained_map, *m_impact_operator, m_CoR, *m_imap );
  }
  else if( m_unconstrained_map != nullptr && m_impact_operator == nullptr && m_imap == nullptr && m_friction_solver != nullptr && m_if_map != nullptr )
  {
    m_sim.flow( m_scripting, next_iter, m_dt, *m_unconstrained_map, m_CoR, m_mu, *m_friction_solver, *m_if_map );
  }
  else
  {
    qFatal( "Impossible code path hit. Exiting." );
  }

  m_iteration++;

  m_delta_H0 = std::max( m_delta_H0, fabs( m_H0 - m_sim.computeTotalEnergy() ) );
  {
    const Vector2s p{ m_sim.computeTotalMomentum() };
    m_delta_p0.x() = std::max( m_delta_p0.x(), fabs( m_p0.x() - p.x() ) );
    m_delta_p0.y() = std::max( m_delta_p0.y(), fabs( m_p0.y() - p.y() ) );
  }
  m_delta_L0 = std::max( m_delta_L0, fabs( m_L0 - m_sim.computeTotalAngularMomentum() ) );

  copyRenderState();

  if( !m_render_at_fps || m_iteration % m_steps_per_frame == 0 )
  {
    update();
  }

  if( !m_movie_dir_name.isEmpty() )
  {
    assert( m_steps_per_frame > 0 );
    if( m_iteration % m_steps_per_frame == 0 )
    {
      // Save a screenshot of the current state
      QString output_image_name{ QString{ tr( "frame%1.png" ) }.arg( m_output_frame, 10, 10, QLatin1Char('0') ) };
      saveScreenshot( m_movie_dir.filePath( output_image_name ) );
      ++m_output_frame;
    }
  }
}

void GLWidget::resetSystem()
{
  if( m_if_map != nullptr )
  {
    m_if_map->resetCachedData();
  }

  m_sim = m_sim0;

  m_iteration = 0;

  m_H0 = m_sim.computeTotalEnergy();
  m_p0 = m_sim.computeTotalMomentum();
  m_L0 = m_sim.computeTotalAngularMomentum();
  m_delta_H0 = 0.0;
  m_delta_p0.setZero();
  m_delta_L0 = 0.0;

  // Reset the output movie option
  m_movie_dir_name = QString{};
  m_movie_dir = QDir{};
  m_output_frame = 0;

  // Reset body colors, in case the number of bodies changed
  m_body_color_gen = std::mt19937_64( 1337 );
  m_body_colors.resize( 3 * m_sim.state().nbodies() );
  for( int bdy_idx = 0; bdy_idx < int(m_sim.state().nbodies()); bdy_idx++ )
  {
    if( !m_sim.state().fixed( bdy_idx ) )
    {
      m_body_colors.segment<3>( 3 * bdy_idx ) = generateColor();
    }
    else
    {
      m_body_colors.segment<3>( 3 * bdy_idx ) << 0.2, 0.2, 0.2;
    }
  }

  // User-provided start of simulation python callback
  m_scripting.setState( m_sim.state() );
  m_scripting.startOfSimCallback();
  m_scripting.forgetState();

  // Register UI callbacks for Python scripting
  // m_scripting.registerBallInsertCallback( this, &ballInsertCallback );
  // m_scripting.registerPlaneDeleteCallback( this, &planeDeleteCallback );

  copyRenderState();
  update();
}

void GLWidget::initializeGL()
{
  assert( m_f == nullptr );
  m_f = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_3_3_Core>();
  if( m_f == nullptr )
  {
    qFatal( "Error, failed to obtain correct OpenGL functions." );
  }
  m_f->initializeOpenGLFunctions();
  m_f->glClearColor( 1.0, 1.0, 1.0, 1.0 );

  m_axis_shader.initialize( m_f );
  m_circle_shader.initialize( m_half_num_circle_subdivs, m_f );
  m_plane_shader.initialize( m_f );
  // m_annulus_shader.initialize( m_num_drum_subdivs, m_f );
  m_rectangle_shader.initialize( m_f );
}

void GLWidget::resizeGL( int width, int height )
{
  m_w = width;
  m_h = height;

  QMatrix4x4 pv;
  {
    assert( height > 0 );
    const float ratio{ float( width ) / float( height ) };
    const float left{ m_center_x - m_display_scale * ratio };
    const float right{ m_center_x + m_display_scale * ratio };
    const float bottom{ m_center_y - m_display_scale };
    const float top{ m_center_y + m_display_scale };
    constexpr float nearVal{ -1.0 };
    constexpr float farVal{ 1.0 };
    pv.ortho( left, right, bottom, top, nearVal, farVal );
  }

  m_circle_shader.setTransform( pv );
  m_plane_shader.setTransform( pv );
  // m_annulus_shader.setTransform( pv );
  m_rectangle_shader.setTransform( pv );

  pv.scale( m_display_scale, m_display_scale );
  m_axis_shader.setTransform( pv );
}

void GLWidget::paintGL()
{
  assert( m_f != nullptr );

  m_f->glClear( GL_COLOR_BUFFER_BIT );

  if( m_left_mouse_button_pressed )
  {
    m_axis_shader.draw();
  }

  m_plane_shader.draw();
  // m_annulus_shader.draw();
  m_rectangle_shader.draw();
  m_circle_shader.draw();

  if( m_display_HUD )
  {
    paintHUD();
  }

  assert( checkGLErrors() );
}

void GLWidget::renderAtFPS( const bool render_at_fps )
{
  m_render_at_fps = render_at_fps;
}

void GLWidget::lockCamera( const bool lock_camera )
{
  m_lock_camera = lock_camera;
}

void GLWidget::toggleHUD()
{
  m_display_HUD = !m_display_HUD;
  update();
}

void GLWidget::centerCamera( const bool update_gl )
{
  if( m_lock_camera )
  {
    return;
  }

  if( m_sim.state().q().size() == 0 )
  {
    m_display_scale = 1.0;
    m_center_x = 0.0;
    m_center_y = 0.0;
    return;
  }

  const Array4s bbox{ m_sim.state().computeBoundingBox() };
  const scalar& minx{ bbox( 0 ) };
  const scalar& maxx{ bbox( 2 ) };
  assert( minx < maxx );
  const scalar& miny{ bbox( 1 ) };
  const scalar& maxy{ bbox( 3 ) };
  assert( miny < maxy );

  const scalar cx{ minx + 0.5 * ( maxx - minx ) };
  const scalar rx{ maxx - cx };
  const scalar cy{ miny + 0.5 * ( maxy - miny ) };
  const scalar ry{ maxy - cy };

  const scalar ratio{ scalar( m_h ) / scalar( m_w ) };
  const scalar size{ 1.2 * std::max( ratio * rx, ry ) };

  m_center_x = float(cx);
  m_center_y = float(cy);
  m_display_scale = float(size);

  if( update_gl )
  {
    resizeGL( m_w, m_h );
    update();
  }
}

void GLWidget::saveScreenshot( const QString& file_name )
{
  std::stringstream ss;
  ss << "Saving screenshot of time " << std::fixed << std::setprecision( m_display_precision )
     << m_iteration * scalar( m_dt ) << " to " << file_name.toStdString();
  qInfo( "%s", ss.str().c_str() );
  const QImage frame_buffer{ grabFramebuffer() };
  frame_buffer.save( file_name );
}

void GLWidget::setMovieDir( const QString& dir_name )
{
  m_movie_dir_name = dir_name;
  m_output_frame = 0;

  // Save a screenshot of the current state
  if( !m_movie_dir_name.isEmpty() )
  {
    m_movie_dir.setPath( m_movie_dir_name );
    assert( m_movie_dir.exists() );

    const QString output_image_name{ QString{ tr( "frame%1.png" ) }.arg( m_output_frame, 10, 10, QLatin1Char{ '0' } ) };
    saveScreenshot( m_movie_dir.filePath( output_image_name ) );
    m_output_frame++;
  }
}

void GLWidget::setMovieFPS( const unsigned fps )
{
  assert( fps > 0 );
  m_output_fps = fps;
  m_output_frame = 0;
  if( 1.0 < scalar( m_dt * std::intmax_t( m_output_fps ) ) )
  {
    qWarning() << "Warning, requested movie frame rate faster than timestep. Dumping at timestep rate.";
    m_steps_per_frame = 1;
  }
  else
  {
    const Rational<std::intmax_t> potential_steps_per_frame{ std::intmax_t( 1 ) / ( m_dt * std::intmax_t( m_output_fps ) ) };
    if( !potential_steps_per_frame.isInteger() )
    {
      if( m_dt != Rational<std::intmax_t>{ 0 } )
      {
        qWarning() << "Warning, timestep and output frequency do not yield an integer number of timesteps for data output. Dumping at timestep rate.";
      }
      m_steps_per_frame = 1;
    }
    else
    {
      m_steps_per_frame = unsigned( potential_steps_per_frame.numerator() );
    }
  }
}

void GLWidget::exportCameraSettings()
{
  std::stringstream ss;
  ss << "<camera cx=\"" << m_center_x << "\" cy=\"" << m_center_y << "\" scale_factor=\"" << m_display_scale << "\" fps=\"" << m_output_fps
     << "\" render_at_fps=\"" << m_render_at_fps << "\" locked=\"" << m_lock_camera << "\"/>";
  qInfo( "%s", ss.str().c_str() );
}

static QString generateTimeString( const unsigned iteration, const Rational<std::intmax_t>& dt, const int display_precision, const scalar& end_time )
{
  QString time_string{ QObject::tr( "  t: " ) };
  time_string += QString::number( iteration * scalar( dt ), 'f', display_precision );
  if( end_time != SCALAR_INFINITY )
  {
    time_string += QString{ QObject::tr( " / " ) };
    time_string += QString::number( end_time );
  }
  return time_string;
}

static QString generateNumericString( const std::string& label, const scalar& number )
{
  return QString{ label.c_str() } + QString::number( number );
}

void GLWidget::paintHUD()
{
  const QString time_string{ generateTimeString( m_iteration, m_dt, m_display_precision, m_end_time ) };
  const QString delta_H{ generateNumericString( " dH: ", m_delta_H0 ) };
  const QString delta_px{ generateNumericString( "dpx: ", m_delta_p0.x() ) };
  const QString delta_py{ generateNumericString( "dpy: ", m_delta_p0.y() ) };
  const QString delta_L{ generateNumericString( " dL: ", m_delta_L0 ) };

  QFont fixedFont{ QFontDatabase::systemFont(QFontDatabase::FixedFont) };
  fixedFont.setPointSize( 12 );

  int text_width{ 0 };
  {
    const QFontMetrics font_metrics{ fixedFont };
    text_width = std::max( text_width, font_metrics.boundingRect( time_string ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_H ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_px ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_py ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_L ).width() );
  }

  const int xextent{ text_width + 2 + 4 };
  constexpr int yextent{ 5 * 12 + 4 };

  {
    QPainter painter{ this };
    painter.setPen( QColor{ 0, 0, 0, 125 } );
    {
      const QRect rect{ 0, 0, xextent, yextent };
      painter.fillRect( rect, QBrush{ QColor{ 0, 0, 0, 128 } } );
    }
    painter.setPen( QColor{ 255, 255, 255 } );
    painter.setFont( fixedFont );
    painter.drawText( 2, fixedFont.pointSize(), time_string );
    painter.drawText( 2, 2 * fixedFont.pointSize(), delta_H );
    painter.drawText( 2, 3 * fixedFont.pointSize(), delta_px );
    painter.drawText( 2, 4 * fixedFont.pointSize(), delta_py );
    painter.drawText( 2, 5 * fixedFont.pointSize(), delta_L );
  }

  // QPainter disables multi-sampling, so turn it back on
  m_f->glEnable( GL_MULTISAMPLE );
}

void GLWidget::mousePressEvent( QMouseEvent* event )
{
  if( m_lock_camera )
  {
    return;
  }

  bool repaint_needed{ false };

  if( event->buttons() & Qt::LeftButton )
  {
    m_left_mouse_button_pressed = true;
    repaint_needed = true;
  }
  if( event->buttons() & Qt::RightButton )
  {
    m_right_mouse_button_pressed = true;
  }

  if( repaint_needed )
  {
    update();
  }

  m_last_pos = event->pos();
}

void GLWidget::mouseReleaseEvent( QMouseEvent* event )
{
  if( m_lock_camera )
  {
    return;
  }

  bool repaint_needed{ false };

  if( !( event->buttons() & Qt::LeftButton ) && m_left_mouse_button_pressed )
  {
    m_left_mouse_button_pressed = false;
    repaint_needed = true;
  }
  if( !( event->buttons() & Qt::RightButton ) && m_right_mouse_button_pressed )
  {
    m_right_mouse_button_pressed = false;
  }

  if( repaint_needed )
  {
    update();
  }
}

void GLWidget::mouseMoveEvent( QMouseEvent* event )
{
  if( m_lock_camera )
  {
    return;
  }

  const int dx{ event->x() - m_last_pos.x() };
  const int dy{ event->y() - m_last_pos.y() };
  m_last_pos = event->pos();

  bool repaint_needed{ false };

  if( event->buttons() & Qt::LeftButton )
  {
    assert( m_left_mouse_button_pressed );
    const float scale{ 2.0f * m_display_scale / float( m_h ) };
    const float translate_x{ scale * float(dx) };
    const float translate_y{ scale * float(dy) };
    m_center_x -= translate_x;
    m_center_y += translate_y;

    repaint_needed = true;
    resizeGL( m_w, m_h );
  }

  if( event->buttons() & Qt::RightButton )
  {
    assert( m_right_mouse_button_pressed );
    // makeCurrent();
    const float new_val{ 0.02f * m_display_scale * float(dy) };
    m_display_scale = std::max( 0.1f, m_display_scale + new_val );
    repaint_needed = true;
    resizeGL( m_w, m_h );
  }

  if( repaint_needed )
  {
    update();
  }
}

void GLWidget::wheelEvent( QWheelEvent* event )
{
  if( m_lock_camera )
  {
    return;
  }

  // makeCurrent();
  const float new_val{ -0.002f * m_display_scale * float(event->delta()) };
  m_display_scale = std::max( 0.1f, m_display_scale + new_val );
  resizeGL( m_w, m_h );
  update();
}

static std::string glErrorToString( const GLenum error_code )
{
  switch( error_code )
  {
    case GL_NO_ERROR:
      return "GL_NO_ERROR";
    case GL_INVALID_ENUM:
      return "GL_INVALID_ENUM";
    case GL_INVALID_VALUE:
      return "GL_INVALID_VALUE";
    case GL_INVALID_OPERATION:
      return "GL_INVALID_OPERATION";
    case GL_INVALID_FRAMEBUFFER_OPERATION:
      return "GL_INVALID_FRAMEBUFFER_OPERATION";
    case GL_OUT_OF_MEMORY:
      return "GL_OUT_OF_MEMORY";
    case GL_STACK_UNDERFLOW:
      return "GL_STACK_UNDERFLOW";
    case GL_STACK_OVERFLOW:
      return "GL_STACK_OVERFLOW";
    default:
      return "Unknown error. Please contact the maintainer of this code.";
  }
}

bool GLWidget::checkGLErrors() const
{
  const GLenum error_code{ m_f->glGetError() };
  if( error_code != GL_NO_ERROR )
  {
    const std::string error{ std::string{"OpenGL error: "} + glErrorToString( error_code ) };
    qWarning( "%s", error.c_str() );
    return false;
  }
  return true;
}
