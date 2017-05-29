#include "GLWidget.h"

#include <QOpenGLFunctions>
#include <QMatrix4x4>

// #include <QDir>
// #include <QtGui>
// #include <QtOpenGL>

#include <QWheelEvent>
#include <QOpenGLFunctions_3_3_Core>

#include <cassert>
// #include <cmath>
#include <iostream>
// #include <fstream>
// #include <iomanip>

#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"

// #include "scisim/StringUtilities.h"
// #include "scisim/UnconstrainedMaps/UnconstrainedMap.h"

#include "ball2d/Ball2DState.h"
// #include "ball2d/StaticGeometry/StaticPlane.h"
// #include "ball2d/StaticGeometry/StaticDrum.h"
// #include "ball2d/Portals/PlanarPortal.h"

#include "ball2dutils/Ball2DSceneParser.h"


GLWidget::GLWidget( QWidget* parent )
: QOpenGLWidget( parent )
, m_f( nullptr )
, m_ball_shader()
, m_w( 512 )
, m_h( 512 )
, m_display_scale( 1.0 )
, m_center_x( 0.0 )
, m_center_y( 0.0 )
// , m_camera_controller()
, m_render_at_fps( false )
, m_lock_camera( false )
, m_last_pos()
, m_left_mouse_button_pressed( false )
, m_right_mouse_button_pressed( false )
, m_ball_colors()
, m_ball_color_gen( 1337 )
, m_display_precision( 0 )
// , m_display_HUD( true )
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
, m_CoR( SCALAR_NAN )
, m_mu( SCALAR_NAN )
, m_sim0()
, m_sim()
, m_H0( 0.0 )
, m_p0( Vector2s::Zero() )
, m_L0( 0.0 )
, m_delta_H0( 0.0 )
, m_delta_p0( Vector2s::Zero() )
, m_delta_L0( 0.0 )
{}

GLWidget::~GLWidget()
{
  // makeCurrent();
  m_ball_shader.cleanup();
  // doneCurrent();
  assert( checkGLErrors() );
}

QSize GLWidget::minimumSizeHint() const
{
  return QSize{ 50, 50 };
}

QSize GLWidget::sizeHint() const
{
  return QSize{ m_w, m_h };
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

bool GLWidget::openScene( const QString& xml_scene_file_name, const bool& render_on_load, unsigned& fps, bool& render_at_fps, bool& lock_camera )
{
  // State provided by config files
  std::string new_scripting_callback_name;
  Ball2DState new_simulation_state;
  std::string new_dt_string;
  Rational<std::intmax_t> new_dt;
  scalar new_end_time = SCALAR_NAN;
  std::unique_ptr<UnconstrainedMap> new_unconstrained_map{ nullptr };
  std::unique_ptr<ImpactOperator> new_impact_operator{ nullptr };
  scalar new_CoR = SCALAR_NAN;
  std::unique_ptr<ImpactMap> new_imap{ nullptr };
  std::unique_ptr<FrictionSolver> new_friction_solver{ nullptr };
  scalar new_mu = SCALAR_NAN;
  std::unique_ptr<ImpactFrictionMap> new_if_map{ nullptr };
  bool camera_set{ false };
  Eigen::Vector2d camera_center{ Eigen::Vector2d::Constant( std::numeric_limits<double>::signaling_NaN() ) };
  double camera_scale_factor{ std::numeric_limits<double>::signaling_NaN() };
  unsigned new_fps;
  bool new_render_at_fps;
  bool new_lock_camera;

  // TODO: Instead of std::string as input, just take PythonScripting directly
  const bool loaded_successfully{ Ball2DSceneParser::parseXMLSceneFile( xml_scene_file_name.toStdString(), new_scripting_callback_name, new_simulation_state, new_unconstrained_map, new_dt_string, new_dt, new_end_time, new_impact_operator, new_imap, new_CoR, new_friction_solver, new_mu, new_if_map, camera_set, camera_center, camera_scale_factor, new_fps, new_render_at_fps, new_lock_camera ) };

  if( !loaded_successfully )
  {
    qWarning() << "Failed to load file: " << xml_scene_file_name;
    return false;
  }

  // Ensure we have a correct combination of maps.
  assert( ( new_unconstrained_map != nullptr && new_impact_operator == nullptr && new_friction_solver == nullptr && new_if_map == nullptr && new_imap == nullptr ) ||
          ( new_unconstrained_map != nullptr && new_impact_operator != nullptr && new_friction_solver == nullptr && new_if_map == nullptr && new_imap != nullptr ) ||
          ( new_unconstrained_map != nullptr && new_impact_operator == nullptr && new_friction_solver != nullptr && new_if_map != nullptr && new_imap == nullptr ) );

  // Set the new maps
  m_unconstrained_map.swap( new_unconstrained_map );
  m_impact_operator.swap( new_impact_operator );
  m_friction_solver.swap( new_friction_solver );
  m_if_map.swap( new_if_map );
  m_imap.swap( new_imap );

  // Initialize the scripting callback
  {
    PythonScripting new_scripting{ xmlFilePath( xml_scene_file_name.toStdString() ), new_scripting_callback_name };
    swap( m_scripting, new_scripting );
  }

  // Save the coefficient of restitution and the coefficient of friction
  m_CoR = new_CoR;
  m_mu = new_mu;

  // Push the new state to the simulation
  using std::swap;
  swap( new_simulation_state, m_sim.state() );
  m_sim.clearConstraintCache();

  // Cache the new state locally to allow one to reset a simulation
  m_sim0 = m_sim;

  // Save the timestep and compute related quantities
  m_dt = new_dt;
  assert( m_dt.positive() );
  m_iteration = 0;
  m_end_time = new_end_time;
  assert( m_end_time > 0.0 );

  // Update the FPS setting
  if( camera_set )
  {
    m_render_at_fps = new_render_at_fps;
    m_output_fps = new_fps;
    m_lock_camera = new_lock_camera;
  }
  assert( m_output_fps > 0 );
  setMovieFPS( m_output_fps );

  // For the parent to update the UI
  fps = m_output_fps;
  render_at_fps = m_render_at_fps;
  lock_camera = m_lock_camera;

  // Compute the initial energy, momentum, and angular momentum
  m_H0 = m_sim.state().computeTotalEnergy();
  m_p0 = m_sim.state().computeMomentum();
  m_L0 = m_sim.state().computeAngularMomentum();
  // Trivially there is no change in energy, momentum, and angular momentum until we take a timestep
  m_delta_H0 = 0.0;
  m_delta_p0 = Vector2s::Zero();
  m_delta_L0 = 0.0;

  // Compute the number of characters after the decimal point in the timestep string
  m_display_precision = computeTimestepDisplayPrecision( m_dt, new_dt_string );

  // Generate a random color for each ball
  // TODO: Initialize these directly in HSV of CMYK and save as QColor objects
  m_ball_colors.resize( 3 * m_sim.state().nballs() );
  {
    m_ball_color_gen = std::mt19937_64( 1337 );
    std::uniform_real_distribution<scalar> color_gen( 0.0, 1.0 );
    for( int i = 0; i < m_ball_colors.size(); i += 3 )
    {
      scalar r = 1.0;
      scalar g = 1.0;
      scalar b = 1.0;
      // Generate colors until we get one with a luminance within [0.1,0.9]
      while( ( 0.2126 * r + 0.7152 * g + 0.0722 * b ) > 0.9 || ( 0.2126 * r + 0.7152 * g + 0.0722 * b ) < 0.1 )
      {
        r = color_gen( m_ball_color_gen );
        g = color_gen( m_ball_color_gen );
        b = color_gen( m_ball_color_gen );
      }
      m_ball_colors.segment<3>( i ) << r, g, b;
    }
  }

  // Reset the output movie option
  m_movie_dir_name = QString{};
  m_movie_dir = QDir{};

  const bool lock_backup{ m_lock_camera };
  m_lock_camera = false;

  if( !camera_set )
  {
    centerCamera( false );
  }
  else
  {
    m_center_x = camera_center.x();
    m_center_y = camera_center.y();
    m_display_scale = camera_scale_factor;
  }

  m_lock_camera = lock_backup;

  // User-provided start of simulation python callback
  m_scripting.setState( m_sim.state() );
  m_scripting.startOfSimCallback();
  m_scripting.forgetState();

  if( render_on_load )
  {
    resizeGL( m_w, m_h );
    update();
  }

  copyBallRenderState();

  return true;
}

void GLWidget::copyBallRenderState()
{
  Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& circle_data{ m_ball_shader.circleData() };
  circle_data.resize( 6 * m_sim.state().nballs() );
  for( unsigned ball_idx = 0; ball_idx < m_sim.state().nballs(); ball_idx++ )
  {
    // Center of mass, radius, and color
    circle_data.segment<2>( 6 * ball_idx ) = m_sim.state().q().segment<2>( 2 * ball_idx ).cast<GLfloat>();
    circle_data( 6 * ball_idx + 2 ) = GLfloat(m_sim.state().r()( ball_idx ));
    circle_data.segment<3>( 6 * ball_idx + 3 ) = m_ball_colors.segment<3>( 3 * ball_idx ).cast<GLfloat>();
  }
}

void GLWidget::stepSystem()
{
  if( m_iteration * scalar( m_dt ) >= m_end_time )
  {
    // User-provided end of simulation python callback
    m_scripting.setState( m_sim.state() );
    m_scripting.endOfSimCallback();
    m_scripting.forgetState();
    // TODO: Display a message! std::cout << "Simulation complete. Exiting." << std::endl;
    std::exit( EXIT_SUCCESS );
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

  {
    const Ball2DState& state{ m_sim.state() };
    m_delta_H0 = std::max( m_delta_H0, fabs( m_H0 - state.computeTotalEnergy() ) );
    const Vector2s p{ state.computeMomentum() };
    m_delta_p0.x() = std::max( m_delta_p0.x(), fabs( m_p0.x() - p.x() ) );
    m_delta_p0.y() = std::max( m_delta_p0.y(), fabs( m_p0.y() - p.y() ) );
    m_delta_L0 = std::max( m_delta_L0, fabs( m_L0 - state.computeAngularMomentum() ) );
  }

  // If the number of particles changed
  if( m_ball_colors.size() != 3 * m_sim.state().r().size() )
  {
    qFatal( "Error, changing ball count not supported in the new front-end, yet!" );
//     const unsigned original_size{ static_cast<unsigned>( m_ball_colors.size() ) };
//     m_ball_colors.conservativeResize( 3 * m_sim.state().r().size() );
//     // If the size grew
//     if( m_ball_colors.size() > original_size )
//     {
//       const unsigned num_new_balls = ( unsigned( m_ball_colors.size() ) - original_size ) / 3;
//       // Add new colors
//       for( unsigned new_ball_num = 0; new_ball_num < num_new_balls; ++new_ball_num )
//       {
//         std::uniform_real_distribution<scalar> color_gen( 0.0, 1.0 );
//         scalar r = 1.0;
//         scalar g = 1.0;
//         scalar b = 1.0;
//         // Generate colors until we get one with a luminance within [0.1,0.9]
//         while( ( 0.2126 * r + 0.7152 * g + 0.0722 * b ) > 0.9 || ( 0.2126 * r + 0.7152 * g + 0.0722 * b ) < 0.1 )
//         {
//           r = color_gen( m_ball_color_gen );
//           g = color_gen( m_ball_color_gen );
//           b = color_gen( m_ball_color_gen );
//         }
//         m_ball_colors.segment<3>( original_size + 3 * new_ball_num ) << r, g, b;
//       }
//     }
  }

  copyBallRenderState();

  if( !m_render_at_fps || m_iteration % m_steps_per_frame == 0 )
  {
    update();
  }

  if( m_movie_dir_name.size() != 0 )
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

  m_H0 = m_sim.state().computeTotalEnergy();
  m_p0 = m_sim.state().computeMomentum();
  m_L0 = m_sim.state().computeAngularMomentum();
  m_delta_H0 = 0.0;
  m_delta_p0.setZero();
  m_delta_L0 = 0.0;

  // Reset the output movie option
  m_movie_dir_name = QString{};
  m_movie_dir = QDir{};
  m_output_frame = 0;

  // Reset ball colors, in case the number of balls changed
  m_ball_colors.resize( 3 * m_sim.state().nballs() );
  {
    m_ball_color_gen = std::mt19937_64( 1337 );
    std::uniform_real_distribution<scalar> color_gen( 0.0, 1.0 );
    for( int i = 0; i < m_ball_colors.size(); i += 3 )
    {
      scalar r = 1.0; scalar g = 1.0; scalar b = 1.0;
      // Generate colors until we get one with a luminance within [0.1,0.9]
      while( ( 0.2126 * r + 0.7152 * g + 0.0722 * b ) > 0.9 || ( 0.2126 * r + 0.7152 * g + 0.0722 * b ) < 0.1 )
      {
        r = color_gen( m_ball_color_gen );
        g = color_gen( m_ball_color_gen );
        b = color_gen( m_ball_color_gen );
      }
      m_ball_colors.segment<3>( i ) << r, g, b;
    }
  }

  // User-provided start of simulation python callback
  m_scripting.setState( m_sim.state() );
  m_scripting.startOfSimCallback();
  m_scripting.forgetState();

  copyBallRenderState();
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

  m_ball_shader.initialize( m_f );
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

  // for (int row = 0; row < 4; row++)
  // {
  //   for (int col = 0; col < 4; col++)
  //   {
  //     std::cout << pv(row, col) << " ";
  //   }
  //   std::cout << std::endl;
  // }

  m_ball_shader.setTransform( pv );
}

void GLWidget::paintGL()
{
  assert( m_f != nullptr );

  m_f->glClear( GL_COLOR_BUFFER_BIT );

  m_ball_shader.draw();

  assert( checkGLErrors() );
}
// {
//   assert( glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE );

//   glClear( GL_COLOR_BUFFER_BIT );

//   glMatrixMode( GL_MODELVIEW );

//   if( axesDrawingIsEnabled() )
//   {
//     paintAxes();
//   }

//   paintSystem();

//   if( m_display_HUD )
//   {
//     paintHUD();
//   }

//   assert( checkGLErrors() );
// }

// bool GLWidget::axesDrawingIsEnabled() const
// {
//   return m_left_mouse_button_pressed;
// }

// void GLWidget::paintAxes()
// {
//   // Draw the positive x axis
//   glColor3d( 1.0, 0.0, 0.0 );
//   glLineWidth( 2.0 );
//   glBegin( GL_LINES );
//   glVertex4f( 0.0, 0.0, 0.0, 1.0 );
//   glVertex4f( 1.0, 0.0, 0.0, 0.0 );
//   glEnd();

//   // Draw the negative x axis
//   glColor3d( 1.0, 0.0, 0.0 );
//   glLineWidth( 2.0 );
//   glLineStipple( 8, 0xAAAA );
//   glEnable( GL_LINE_STIPPLE );
//   glBegin( GL_LINES );
//   glVertex4f( 0.0, 0.0, 0.0, 1.0 );
//   glVertex4f( -1.0, 0.0, 0.0, 0.0 );
//   glEnd();
//   glDisable( GL_LINE_STIPPLE );

//   // Draw the positive y axis
//   glColor3d( 0, 1.0, 0 );
//   glLineWidth( 2.0 );
//   glBegin( GL_LINES );
//   glVertex4f( 0.0, 0.0, 0.0, 1.0 );
//   glVertex4f( 0.0, 1.0, 0.0, 0.0 );
//   glEnd();

//   // Draw the negative y axis
//   glColor3d( 0, 1.0, 0 );
//   glLineWidth( 2.0 );
//   glLineStipple( 8, 0xAAAA );
//   glEnable( GL_LINE_STIPPLE );
//   glBegin( GL_LINES );
//   glVertex4f( 0.0, 0.0, 0.0, 1.0 );
//   glVertex4f( 0.0, -1.0, 0.0, 0.0 );
//   glEnd();
//   glDisable( GL_LINE_STIPPLE );
// }

void GLWidget::renderAtFPS( const bool render_at_fps )
{
  m_render_at_fps = render_at_fps;
}

void GLWidget::lockCamera( const bool lock_camera )
{
  m_lock_camera = lock_camera;
}

// void GLWidget::toggleHUD()
// {
//   m_display_HUD = !m_display_HUD;

//   update();
// }

void GLWidget::centerCamera( const bool update_gl )
{
  if( m_lock_camera )
  {
    return;
  }

  if( m_sim.empty() )
  {
    m_display_scale = 1.0;
    m_center_x = 0.0;
    m_center_y = 0.0;
    return;
  }

  const Vector4s bbox{ m_sim.state().computeBoundingBox() };
  const scalar& minx{ bbox( 0 ) };
  const scalar& maxx{ bbox( 1 ) };
  const scalar& miny{ bbox( 2 ) };
  const scalar& maxy{ bbox( 3 ) };

  const scalar cx{ minx + 0.5 * ( maxx - minx ) };
  const scalar rx{ maxx - cx };
  const scalar cy{ miny + 0.5 * ( maxy - miny ) };
  const scalar ry{ maxy - cy };

  const scalar ratio{ scalar( m_h ) / scalar( m_w ) };
  const scalar size{ 1.2 * std::max( ratio * rx, ry ) };

  m_center_x = cx;
  m_center_y = cy;
  m_display_scale = size;

  if( update_gl )
  {
    resizeGL( m_w, m_h );
    update();
  }
}

void GLWidget::saveScreenshot( const QString& file_name )
{
  // std::cout << "Saving screenshot of time " << std::fixed << std::setprecision( m_display_precision ) << m_iteration * scalar( m_dt ) << " to " << file_name.toStdString() << std::endl;
  const QImage frame_buffer{ grabFramebuffer() };
  frame_buffer.save( file_name );
}

void GLWidget::setMovieDir( const QString& dir_name )
{
  m_movie_dir_name = dir_name;
  m_output_frame = 0;

  // Save a screenshot of the current state
  if( m_movie_dir_name.size() != 0 )
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
  std::cout << "<camera cx=\"" << m_center_x << "\" cy=\"" << m_center_y << "\" scale_factor=\"" << m_display_scale << "\" fps=\"" << m_output_fps << "\" render_at_fps=\"" << m_render_at_fps << "\" locked=\"" << m_lock_camera << "\"/>" << std::endl;
}

// static void paintInfiniteLine( const Vector2s& x, const Vector2s& n )
// {
//   const scalar theta{ -180.0 * atan2( n.x(), n.y() ) / PI<scalar> };

//   glPushMatrix();

//   glTranslated( GLdouble( x.x() ), GLdouble( x.y() ), GLdouble( 0.0 ) );
//   glRotated( GLdouble( theta ), GLdouble( 0.0 ), GLdouble( 0.0 ), GLdouble( 1.0 ) );

//   glBegin( GL_LINES );
//   glVertex4d(  0.0,  0.0, 0.0, 1.0 );
//   glVertex4d(  1.0,  0.0, 0.0, 0.0 );
//   glVertex4d(  0.0,  0.0, 0.0, 1.0 );
//   glVertex4d( -1.0,  0.0, 0.0, 0.0 );
//   glEnd();

//   glPopMatrix();
// }

// static void paintSolidHalfPlane( const Vector2s& x, const Vector2s& n )
// {
//   paintInfiniteLine( x, n );

//   //const scalar theta = -180.0 * atan2( n.x(), n.y() ) / 3.14159265359;
//   //
//   //glPushMatrix();
//   //
//   //glTranslated( (GLdouble) x.x(), (GLdouble) x.y(), (GLdouble) 0.0 );
//   //glRotated( (GLdouble) theta, 0.0, (GLdouble) 0.0, (GLdouble) 1.0 );
//   //
//   //glBegin( GL_TRIANGLES );
//   //glVertex4d(  0.0,  0.0, 0.0, 1.0 );
//   //glVertex4d(  1.0,  0.0, 0.0, 0.0 );
//   //glVertex4d(  0.0, -1.0, 0.0, 0.0 );
//   //glVertex4d(  0.0,  0.0, 0.0, 1.0 );
//   //glVertex4d( -1.0,  0.0, 0.0, 0.0 );
//   //glVertex4d(  0.0, -1.0, 0.0, 0.0 );
//   //glEnd();
//   //
//   //glPopMatrix();
// }

// // TODO: Abstract the shared code in here into its own function
// static void paintPlanarPortal( const PlanarPortal& planar_portal )
// {
//   // Draw the first plane of the portal
//   {
//     const scalar theta{ -180.0 * atan2( planar_portal.planeA().n().x(), planar_portal.planeA().n().y() ) / PI<scalar> };

//     glPushMatrix();
//     glTranslated( GLdouble( planar_portal.planeA().x().x() ), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
//     glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

//     glLineStipple( 8, 0xAAAA );
//     glEnable( GL_LINE_STIPPLE );
//     glBegin( GL_LINES );
//     glVertex4d( 0.0, 0.0, 0.0, 1.0 );
//     glVertex4d( -1.0, 0.0, 0.0, 0.0 );
//     glEnd();
//     glDisable( GL_LINE_STIPPLE );

//     glLineStipple( 8, 0x5555 );
//     glEnable( GL_LINE_STIPPLE );
//     glBegin( GL_LINES );
//     glVertex4d( 0.0, 0.0, 0.0, 1.0 );
//     glVertex4d( 1.0, 0.0, 0.0, 0.0 );
//     glEnd();
//     glDisable( GL_LINE_STIPPLE );

//     glPopMatrix();

//     if( planar_portal.isLeesEdwards() )
//     {
//       // Draw the lower bound
//       glPushMatrix();
//       glTranslated( GLdouble( planar_portal.planeA().x().x() ), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
//       glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );
//       glTranslated( -planar_portal.bounds(), 0.0, 0.0 );
//       glLineStipple( 8, 0x5555 );
//       glEnable( GL_LINE_STIPPLE );
//       glBegin( GL_LINES );
//       glVertex4d( 0.0, 0.0, 0.0, 1.0 );
//       glVertex4d( 0.0, -1.0, 0.0, 0.0 );
//       glEnd();
//       glDisable( GL_LINE_STIPPLE );
//       glPopMatrix();

//       // Draw the upper bound
//       glPushMatrix();
//       glTranslated( GLdouble( planar_portal.planeA().x().x() ), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
//       glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );
//       glTranslated( planar_portal.bounds(), 0.0, 0.0 );
//       glLineStipple( 8, 0x5555 );
//       glEnable( GL_LINE_STIPPLE );
//       glBegin( GL_LINES );
//       glVertex4d( 0.0, 0.0, 0.0, 1.0 );
//       glVertex4d( 0.0, -1.0, 0.0, 0.0 );
//       glEnd();
//       glDisable( GL_LINE_STIPPLE );
//       glPopMatrix();

//       // Draw a short line to indicate the rest position of the center of the portal
//       glPushMatrix();
//       glTranslated( planar_portal.planeA().x().x(), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
//       glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

//       glLineStipple( 8, 0x5555 );
//       glEnable( GL_LINE_STIPPLE );
//       glBegin( GL_LINES );
//       glVertex2d( 0.0, 0.0 );
//       glVertex2d( 0.0, - 0.2 * planar_portal.bounds() );
//       glEnd();
//       glDisable( GL_LINE_STIPPLE );

//       glPopMatrix();

//       // Draw a short line to indicate the current position of the center of the portal
//       const Vector2s plane_a_x{ planar_portal.transformedAx() };
//       glPushMatrix();
//       glTranslated( GLdouble( plane_a_x.x() ), GLdouble( plane_a_x.y() ), 0.0 );
//       glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

//       glLineStipple( 8, 0x5555 );
//       glEnable( GL_LINE_STIPPLE );
//       glBegin( GL_LINES );
//       glVertex2d( 0.0, 0.0 );
//       glVertex2d( 0.0, - 0.2 * planar_portal.bounds() );
//       glEnd();
//       glDisable( GL_LINE_STIPPLE );

//       glPopMatrix();
//     }
//     else
//     {
//       glPushMatrix();
//       glTranslated( GLdouble( planar_portal.planeA().x().x() ), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
//       glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

//       // Draw an infinite line to show what half of portal is free
//       glLineStipple( 8, 0x5555 );
//       glEnable( GL_LINE_STIPPLE );
//       glBegin( GL_LINES );
//       glVertex4d( 0.0, 0.0, 0.0, 1.0 );
//       glVertex4d( 0.0, -1.0, 0.0, 0.0 );
//       glEnd();
//       glDisable( GL_LINE_STIPPLE );

//       glPopMatrix();
//     }
//   }

//   // Draw the second plane of the portal
//   {
//     const scalar theta{ -180.0 * atan2( planar_portal.planeB().n().x(), planar_portal.planeB().n().y() ) / PI<scalar> };

//     glPushMatrix();
//     glTranslated( GLdouble( planar_portal.planeB().x().x() ), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
//     glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

//     glLineStipple( 8, 0xAAAA );
//     glEnable( GL_LINE_STIPPLE );
//     glBegin( GL_LINES );
//     glVertex4d( 0.0, 0.0, 0.0, 1.0 );
//     glVertex4d( -1.0, 0.0, 0.0, 0.0 );
//     glEnd();
//     glDisable( GL_LINE_STIPPLE );

//     glLineStipple( 8, 0x5555 );
//     glEnable( GL_LINE_STIPPLE );
//     glBegin( GL_LINES );
//     glVertex4d( 0.0, 0.0, 0.0, 1.0 );
//     glVertex4d( 1.0, 0.0, 0.0, 0.0 );
//     glEnd();
//     glDisable( GL_LINE_STIPPLE );

//     glPopMatrix();

//     if( planar_portal.isLeesEdwards() )
//     {
//       // Draw the lower bound
//       glPushMatrix();
//       glTranslated( GLdouble( planar_portal.planeB().x().x() ), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
//       glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );
//       glTranslated( - planar_portal.bounds(), 0.0, 0.0 );
//       glLineStipple( 8, 0x5555 );
//       glEnable( GL_LINE_STIPPLE );
//       glBegin( GL_LINES );
//       glVertex4d( 0.0, 0.0, 0.0, 1.0 );
//       glVertex4d( 0.0, -1.0, 0.0, 0.0 );
//       glEnd();
//       glDisable( GL_LINE_STIPPLE );
//       glPopMatrix();

//       // Draw the upper bound
//       glPushMatrix();
//       glTranslated( GLdouble( planar_portal.planeB().x().x() ), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
//       glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );
//       glTranslated( planar_portal.bounds(), 0.0, 0.0 );
//       glLineStipple( 8, 0x5555 );
//       glEnable( GL_LINE_STIPPLE );
//       glBegin( GL_LINES );
//       glVertex4d( 0.0, 0.0, 0.0, 1.0 );
//       glVertex4d( 0.0, -1.0, 0.0, 0.0 );
//       glEnd();
//       glDisable( GL_LINE_STIPPLE );
//       glPopMatrix();

//       // Draw a short line to indicate the rest position of the center of the portal
//       glPushMatrix();
//       glTranslated( GLdouble( planar_portal.planeB().x().x() ), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
//       glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

//       glLineStipple( 8, 0x5555 );
//       glEnable( GL_LINE_STIPPLE );
//       glBegin( GL_LINES );
//       glVertex2d( 0.0, 0.0 );
//       glVertex2d( 0.0, - 0.2 * planar_portal.bounds() );
//       glEnd();
//       glDisable( GL_LINE_STIPPLE );

//       glPopMatrix();

//       // Draw a short line to indicate the current position of the center of the portal
//       const Vector2s plane_b_x{ planar_portal.transformedBx() };
//       glPushMatrix();
//       glTranslated( GLdouble( plane_b_x.x() ), GLdouble( plane_b_x.y() ), 0.0 );
//       glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

//       glLineStipple( 8, 0x5555 );
//       glEnable( GL_LINE_STIPPLE );
//       glBegin( GL_LINES );
//       glVertex2d( 0.0, 0.0 );
//       glVertex2d( 0.0, - 0.2 * planar_portal.bounds() );
//       glEnd();
//       glDisable( GL_LINE_STIPPLE );

//       glPopMatrix();
//     }
//     else
//     {
//       glPushMatrix();
//       glTranslated( GLdouble( planar_portal.planeB().x().x() ), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
//       glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

//       // Draw an infinite line to show what half of portal is free
//       glLineStipple( 8, 0x5555 );
//       glEnable( GL_LINE_STIPPLE );
//       glBegin( GL_LINES );
//       glVertex4d( 0.0, 0.0, 0.0, 1.0 );
//       glVertex4d( 0.0, -1.0, 0.0, 0.0 );
//       glEnd();
//       glDisable( GL_LINE_STIPPLE );

//       glPopMatrix();
//     }
//   }
// }

// void GLWidget::paintSystem()
// {
//   const Ball2DState& state{ m_sim.state() };

//   // Draw each ball
//   glPushAttrib( GL_COLOR );
//   {
//     const VectorXs& q{ state.q() };
//     const VectorXs& r{ state.r() };
//     assert( q.size() == 2 * r.size() );
//     assert( m_ball_colors.size() == 3 * r.size() );
//     for( int i = 0; i < r.size(); i++ )
//     {
//       glColor3d( m_ball_colors( 3 * i + 0 ), m_ball_colors( 3 * i + 1 ), m_ball_colors( 3 * i + 2 ) );
//       m_circle_renderer.renderSolidCircle( q.segment<2>( 2 * i ), r( i ) );
//     }
//   }
//   glPopAttrib();

//   // Draw teleported versions of each ball
//   glPushAttrib( GL_COLOR );
//   {
//     const VectorXs& q{ state.q() };
//     const VectorXs& r{ state.r() };
//     assert( q.size() == 2 * r.size() );
//     assert( m_ball_colors.size() == 3 * r.size() );
//     const std::vector<PlanarPortal>& planar_portals{ state.planarPortals() };
//     // For each periodic boundary
//     for( const PlanarPortal& planar_portal : planar_portals )
//     {
//       // For each ball
//       for( unsigned ball_idx = 0; ball_idx < r.size(); ball_idx++ )
//       {
//         const Vector2s pos{ q.segment<2>( 2 * ball_idx ) };
//         const scalar rad{ r( ball_idx ) };
//         // If the current ball intersect a periodic boudnary
//         bool intersecting_index;
//         if( planar_portal.ballTouchesPortal( pos, rad, intersecting_index ) )
//         {
//           Vector2s teleported_pos;
//           planar_portal.teleportBall( pos, rad, teleported_pos );
//           glColor3d( m_ball_colors( 3 * ball_idx + 0 ), m_ball_colors( 3 * ball_idx + 1 ), m_ball_colors( 3 * ball_idx + 2 ) );
//           m_circle_renderer.renderCircle( teleported_pos, rad );
//         }
//       }
//     }
//   }
//   glPopAttrib();

//   // Draw each static drum
//   glPushAttrib( GL_COLOR );
//   glColor3d( 0.0, 0.0, 0.0 );
//   {
//     const std::vector<StaticDrum>& drums = state.staticDrums();
//     for( std::vector<StaticDrum>::size_type i = 0; i < drums.size(); i++ )
//     {
//       m_circle_renderer.renderSolidCircle( drums[i].x(), drums[i].r() );
//     }
//   }
//   glPopAttrib();

//   // Draw each planar portal
//   glPushAttrib( GL_COLOR );
//   glPushAttrib( GL_LINE_WIDTH );
//   glLineWidth( 2.0 );
//   {
//     // TODO: Create a set number of nice looking colors for the portal
//     std::mt19937_64 mt{ 123456 };
//     std::uniform_int_distribution<int> color_gen{ 0, 255 };
//     const std::vector<PlanarPortal>& planar_portals{ state.planarPortals() };
//     for( const PlanarPortal& planar_portal : planar_portals )
//     {
//       const int r = color_gen( mt );
//       const int g = color_gen( mt );
//       const int b = color_gen( mt );
//       glColor3d( GLdouble(r) / 255.0, GLdouble(g) / 255.0, GLdouble(b) / 255.0 );
//       paintPlanarPortal( planar_portal );
//     }
//   }
//   glPopAttrib();
//   glPopAttrib();

//   // Draw each static plane
//   glPushAttrib( GL_COLOR );
//   glColor3d( 0.0, 0.0, 0.0 );
//   {
//     const std::vector<StaticPlane>& planes = state.staticPlanes();
//     for( const StaticPlane& plane : planes )
//     {
//       paintSolidHalfPlane( plane.x(), plane.n() );
//     }
//   }
//   glPopAttrib();
// }

// static QString generateTimeString( const unsigned iteration, const Rational<std::intmax_t>& dt, const int display_precision, const scalar& end_time )
// {
//   QString time_string{ QObject::tr( "  t: " ) };
//   time_string += QString::number( iteration * scalar( dt ), 'f', display_precision );
//   if( end_time != SCALAR_INFINITY )
//   {
//     time_string += QString{ QObject::tr( " / " ) };
//     time_string += QString::number( end_time );
//   }
//   return time_string;
// }

// static QString generateNumericString( const std::string& label, const scalar& number )
// {
//   return QString{ label.c_str() } + QString::number( number );
// }

// void GLWidget::paintHUD()
// {
//   static int text_width{ 0 };

//   // String to display in upper left corner
//   const QString time_string{ generateTimeString( m_iteration, m_dt, m_display_precision, m_end_time ) };
//   const QString delta_H{ generateNumericString( " dH: ", m_delta_H0 ) };
//   const QString delta_px{ generateNumericString( "dpx: ", m_delta_p0.x() ) };
//   const QString delta_py{ generateNumericString( "dpy: ", m_delta_p0.y() ) };
//   const QString delta_L{ generateNumericString( " dL: ", m_delta_L0 ) };

//   QFont fixedFont = QFontDatabase::systemFont(QFontDatabase::FixedFont);
//   fixedFont.setPointSize(12);

//   {
//     const QFontMetrics font_metrics{ fixedFont };
//     text_width = std::max( text_width, font_metrics.boundingRect( time_string ).width() );
//     text_width = std::max( text_width, font_metrics.boundingRect( delta_H ).width() );
//     text_width = std::max( text_width, font_metrics.boundingRect( delta_px ).width() );
//     text_width = std::max( text_width, font_metrics.boundingRect( delta_py ).width() );
//     text_width = std::max( text_width, font_metrics.boundingRect( delta_L ).width() );
//   }

//   glPushAttrib( GL_MATRIX_MODE );
//   glMatrixMode( GL_PROJECTION );
//   glPushMatrix();
//   glMatrixMode( GL_MODELVIEW );
//   glPushMatrix();

//   // Set an orthographic projection with height and width equal to window height and width
//   glMatrixMode( GL_PROJECTION );
//   glLoadIdentity();
//   GLint width;
//   GLint height;
//   getViewportDimensions( width, height );
//   assert( width > 0 );
//   assert( height > 0 );
//   glOrtho( 0, width, 0, height, -1, 1 );
//   glMatrixMode( GL_MODELVIEW );
//   glLoadIdentity();

//   // Enable blending for transparent HUD elements
//   glPushAttrib( GL_BLEND );
//   glEnable( GL_BLEND );
//   glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

//   // Draw a semi-transparent overlay so text is visible regardless of background color
//   const int pixelRatio = devicePixelRatio();
//   const Eigen::Matrix<GLdouble,2,1> overlay_start{ 0, (height + (- 5 * 12 - 4) * pixelRatio ) };
//   const Eigen::Matrix<GLdouble,2,1> overlay_extnt{ (text_width + 2 + 4) * pixelRatio, height * pixelRatio };

//   glColor4d( 0.0, 0.0, 0.0, 0.5 );
//   glBegin( GL_QUADS );
//   glVertex2d( GLdouble( overlay_start.x() ), GLdouble( overlay_start.y() ) );
//   glVertex2d( GLdouble( overlay_start.x() + overlay_extnt.x() ), GLdouble( overlay_start.y() ) );
//   glVertex2d( GLdouble( overlay_start.x() + overlay_extnt.x() ), GLdouble( overlay_start.y() + overlay_extnt.y() ) );
//   glVertex2d( GLdouble( overlay_start.x() ), GLdouble( overlay_start.y() + overlay_extnt.y() ) );
//   glEnd();

//   glDisable( GL_BLEND );
//   glPopAttrib();

// 	glMatrixMode( GL_MODELVIEW );
//   glPopMatrix();
//   glMatrixMode( GL_PROJECTION );
//   glPopMatrix();
//   glPopAttrib();

//   {
//     assert( checkGLErrors() );

//     QPainter painter( this );
//     painter.setPen( QColor( 255, 255, 255 ) );
//     painter.setFont( fixedFont );
//     painter.drawText( 2, fixedFont.pointSize(), time_string );
//     painter.drawText( 2, 2 * fixedFont.pointSize(), delta_H );
//     painter.drawText( 2, 3 * fixedFont.pointSize(), delta_px );
//     painter.drawText( 2, 4 * fixedFont.pointSize(), delta_py );
//     painter.drawText( 2, 5 * fixedFont.pointSize(), delta_L );
//   }

//   // TODO: Remove this workaround later...
//   // Silently consume the error as a workaround for a bug in Qt 5.8 on OS X
//   while( !checkGLErrors( false ) )
//   {}

//   // QPainter disables multi-sampling, so turn it back on
//   glEnable(GL_MULTISAMPLE);

//   assert( checkGLErrors() );
// }

void GLWidget::mousePressEvent( QMouseEvent* event )
{
  if( m_lock_camera )
  {
    return;
  }

//   bool repaint_needed{ false };

  if( event->buttons() & Qt::LeftButton )
  {
    m_left_mouse_button_pressed = true;
//     repaint_needed = true;
  }
  if( event->buttons() & Qt::RightButton )
  {
    m_right_mouse_button_pressed = true;
  }

//   if( repaint_needed )
//   {
//     update();
//   }

  m_last_pos = event->pos();
}

void GLWidget::mouseReleaseEvent( QMouseEvent* event )
{
  if( m_lock_camera )
  {
    return;
  }

//   bool repaint_needed{ false };

  if( !( event->buttons() & Qt::LeftButton ) && m_left_mouse_button_pressed )
  {
    m_left_mouse_button_pressed = false;
//     repaint_needed = true;
  }
  if( !( event->buttons() & Qt::RightButton ) && m_right_mouse_button_pressed )
  {
    m_right_mouse_button_pressed = false;
  }

//   if( repaint_needed )
//   {
//     update();
//   }
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
    const float new_val{ 0.02f * m_display_scale * dy };
    m_display_scale = std::max( 0.00001f, m_display_scale + new_val );
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
  const float new_val{ -0.002f * m_display_scale * event->delta() };
  m_display_scale = std::max( 0.00001f, m_display_scale + new_val );
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
