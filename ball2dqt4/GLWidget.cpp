#include "GLWidget.h"

#include <QtGui>
#include <QtOpenGL>

#include <cmath>
#include <iostream>
#include <fstream>

#include "scisim/StringUtilities.h"
#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"

#include "ball2dutils/Ball2D.h"

#include "ball2d/Forces/Ball2DForce.h"
#include "ball2d/StaticGeometry/StaticPlane.h"
#include "ball2d/StaticGeometry/StaticDrum.h"
#include "ball2d/Portals/PlanarPortal.h"

#include "ball2dutils/XMLSceneParser.h"

#ifndef NDEBUG
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

static bool checkGLErrors()
{
  const GLenum error_code{ glGetError() };
  if( error_code != GL_NO_ERROR )
  {
    std::cerr << "OpenGL error: " << glErrorToString( error_code ) << std::endl;
    return false;
  }
  return true;
}
#endif

GLWidget::GLWidget( QWidget* parent )
: QGLWidget( QGLFormat( QGL::SampleBuffers ), parent )
, m_camera_controller()
, m_render_at_fps( false )
, m_lock_camera( false )
, m_last_pos()
, m_left_mouse_button_pressed( false )
, m_right_mouse_button_pressed( false )
, m_circle_renderer( 64 )
, m_ball_colors()
, m_ball_color_gen( 1337 )
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
, m_scripting()
, m_iteration( 0 )
, m_dt( 0, 1 )
, m_end_time( SCALAR_INFINITY )
, m_CoR( SCALAR_NAN )
, m_mu( SCALAR_NAN )
, m_state0()
, m_sim()
, m_H0()
, m_p0()
, m_L0()
, m_delta_H0( 0.0 )
, m_delta_p0( Vector2s::Zero() )
, m_delta_L0( 0.0 )
{}

GLWidget::~GLWidget()
{}

QSize GLWidget::minimumSizeHint() const
{
  return QSize{ 50, 50 };
}

QSize GLWidget::sizeHint() const
{
  return QSize{ static_cast<int>( m_camera_controller.width() ), static_cast<int>( m_camera_controller.height() ) };
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

bool GLWidget::openScene( const QString& xml_scene_file_name, const bool& render_on_load, unsigned& fps, bool& render_at_fps, bool& lock_camera )
{
  // TODO: Directly read state into SimulationStateBalls2D
  // State provided by config files
  std::string new_scripting_callback_name;
  std::vector<Ball2D> new_balls;
  std::vector<StaticDrum> new_drums;
  std::vector<StaticPlane> new_planes;
  std::vector<PlanarPortal> new_planar_portals;
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
  std::vector<std::unique_ptr<Ball2DForce>> new_forces;
  bool camera_set{ false };
  Eigen::Vector2d camera_center{ Eigen::Vector2d::Constant( std::numeric_limits<double>::signaling_NaN() ) };
  double camera_scale_factor{ std::numeric_limits<double>::signaling_NaN() };
  unsigned new_fps;
  bool new_render_at_fps;
  bool new_lock_camera;

  // TODO: Instead of std::string as input, just take PythonScripting directly
  const bool loaded_successfully{ XMLSceneParser::parseXMLSceneFile( xml_scene_file_name.toStdString(), new_scripting_callback_name, new_balls, new_drums, new_planes, new_planar_portals, new_unconstrained_map, new_dt_string, new_dt, new_end_time, new_impact_operator, new_imap, new_CoR, new_friction_solver, new_mu, new_if_map, new_forces, camera_set, camera_center, camera_scale_factor, new_fps, new_render_at_fps, new_lock_camera ) };

  if( !loaded_successfully )
  {
    std::cerr << "Failed to load file: " << xml_scene_file_name.toStdString() << std::endl;
    return false;
  }

  // Ensure we have a correct combination of maps.
  assert( ( new_unconstrained_map != nullptr && new_impact_operator == nullptr && new_friction_solver == nullptr && new_if_map == nullptr && new_imap == nullptr ) ||
          ( new_unconstrained_map != nullptr && new_impact_operator != nullptr && new_friction_solver == nullptr && new_if_map == nullptr && new_imap != nullptr ) ||
          ( new_unconstrained_map != nullptr && new_impact_operator == nullptr && new_friction_solver != nullptr && new_if_map != nullptr && new_imap == nullptr ) );

  // TODO: Why are the swaps member functions?
  // Set the new maps
  m_unconstrained_map.swap( new_unconstrained_map );
  m_impact_operator.swap( new_impact_operator );
  m_friction_solver.swap( new_friction_solver );
  m_if_map.swap( new_if_map );
  m_imap.swap( new_imap );

  // Initialize the scripting callback
  {
    std::string path;
    std::string file_name;
    StringUtilities::splitAtLastCharacterOccurence( xml_scene_file_name.toStdString(), path, file_name, '/' );
    if( file_name.empty() )
    {
      using std::swap;
      swap( path, file_name );
    }
    PythonScripting new_scripting{ path, new_scripting_callback_name };
    swap( m_scripting, new_scripting );
  }

  // Save the coefficient of restitution and the coefficient of friction
  m_CoR = new_CoR;
  m_mu = new_mu;

  // Copy the new state over to the simulation
  Ball2DState new_simulation_state;
  // TODO: Move this code to a factory function
  {
    VectorXs q0{ static_cast<VectorXs::Index>( 2 * new_balls.size() ) };
    VectorXs v0{ static_cast<VectorXs::Index>( 2 * new_balls.size() ) };
    VectorXs m0{ static_cast<VectorXs::Index>( 2 * new_balls.size() ) };
    VectorXs r0{ static_cast<VectorXs::Index>( new_balls.size() ) };
    std::vector<bool> fixed0( new_balls.size() );
    for( std::vector<Ball2D>::size_type i = 0; i < new_balls.size(); ++i )
    {
      q0.segment<2>( 2 * i ) = new_balls[i].x();
      v0.segment<2>( 2 * i ) = new_balls[i].v();
      m0.segment<2>( 2 * i ).setConstant( new_balls[i].m() );
      r0( i ) = new_balls[i].r();
      fixed0[ i ] = new_balls[i].fixed();
    }

    Ball2DState new_state{ q0, v0, m0, r0, fixed0, new_drums, new_planes, new_planar_portals, new_forces };
    using std::swap;
    swap( new_simulation_state, new_state );
  }

  // Cache the new state locally to allow one to reset a simulation
  m_state0 = new_simulation_state;

  // Push the new state to the simulation
  m_sim.swapState( new_simulation_state );

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
  m_H0 = m_state0.computeTotalEnergy();
  m_p0 = m_state0.computeMomentum();
  m_L0 = m_state0.computeAngularMomentum();
  // Trivially there is no change in energy, momentum, and angular momentum until we take a timestep
  m_delta_H0 = 0.0;
  m_delta_p0 = Vector2s::Zero();
  m_delta_L0 = 0.0;

  // Compute the number of characters after the decimal point in the timestep string
  m_display_precision = computeTimestepDisplayPrecision( m_dt, new_dt_string );

  // Generate a random color for each ball
  // TODO: Initialize these directly in HSV of CMYK and save as QColor objects
  m_ball_colors.resize( 3 * m_state0.nballs() );
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
    m_camera_controller.setCenter( camera_center.x(), camera_center.y() );
    m_camera_controller.setScaleFactor( camera_scale_factor );
  }

  m_lock_camera = lock_backup;

  if( render_on_load )
  {
    updateGL();
  }

  return true;
}

void GLWidget::stepSystem()
{
  if( m_iteration * scalar( m_dt ) >= m_end_time )
  {
    std::cout << "Simulation complete. Exiting." << std::endl;
    std::exit( EXIT_SUCCESS );
  }

  const unsigned next_iter = m_iteration + 1;

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
    std::cerr << "Impossible code path hit. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  ++m_iteration;

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
    const unsigned original_size{ static_cast<unsigned>( m_ball_colors.size() ) };
    m_ball_colors.conservativeResize( 3 * m_sim.state().r().size() );
    // If the size grew
    if( m_ball_colors.size() > original_size )
    {
      const unsigned num_new_balls = ( m_ball_colors.size() - original_size ) / 3;
      // Add new colors
      for( unsigned new_ball_num = 0; new_ball_num < num_new_balls; ++new_ball_num )
      {
        std::uniform_real_distribution<scalar> color_gen( 0.0, 1.0 );
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
        m_ball_colors.segment<3>( original_size + 3 * new_ball_num ) << r, g, b;
      }
    }
  }

  if( !m_render_at_fps || m_iteration % m_steps_per_frame == 0 )
  {
    updateGL();
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
  m_sim.setState( m_state0 );

  m_iteration = 0;

  m_delta_H0 = 0.0;
  m_delta_p0 = Vector2s::Zero();
  m_delta_L0 = 0.0;

  // Reset the output movie option
  m_movie_dir_name = QString{};
  m_movie_dir = QDir{};
  m_output_frame = 0;

  // Reset ball colors, in case the number of balls changed
  m_ball_colors.resize( 3 * m_state0.nballs() );
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

  updateGL();
}

void GLWidget::getSimData( scalar& time, scalar& T, scalar& U, Vector2s& p, scalar& L )
{
  time = m_iteration * scalar( m_dt );

  const Ball2DState& state{ m_sim.state() };

  T = state.computeKineticEnergy();
  U = state.computePotentialEnergy();

  p = state.computeMomentum();
  L = state.computeAngularMomentum();
}

void GLWidget::initializeGL()
{
  qglClearColor( QColor{ 255, 255, 255, 255 } );
  assert( checkGLErrors() );
}

void GLWidget::resizeGL( int width, int height )
{
  assert( width >= 0 ); assert( height >= 0 );

  m_camera_controller.reshape( width, height ); 

  assert( checkGLErrors() );
}

void GLWidget::paintGL()
{
  glMatrixMode( GL_MODELVIEW );

  glClear( GL_COLOR_BUFFER_BIT );

  if( axesDrawingIsEnabled() )
  {
    paintAxes();
  }

  paintSystem();

  if( m_display_HUD )
  {
    paintHUD();
  }

  assert( autoBufferSwap() );

  assert( checkGLErrors() );
}

bool GLWidget::axesDrawingIsEnabled() const
{
  return m_left_mouse_button_pressed;
}

void GLWidget::paintAxes() const
{
  // Draw the positive x axis
  qglColor( QColor{ 255, 0, 0 } );
  glLineWidth( 2.0 );
  glBegin( GL_LINES );
  glVertex4f( 0.0, 0.0, 0.0, 1.0 );
  glVertex4f( 1.0, 0.0, 0.0, 0.0 );
  glEnd();

  // Draw the negative x axis
  qglColor( QColor{ 255, 0, 0 } );
  glLineWidth( 2.0 );
  glLineStipple( 8, 0xAAAA );
  glEnable( GL_LINE_STIPPLE );
  glBegin( GL_LINES );
  glVertex4f( 0.0, 0.0, 0.0, 1.0 );
  glVertex4f( -1.0, 0.0, 0.0, 0.0 );
  glEnd();
  glDisable( GL_LINE_STIPPLE );

  // Draw the positive y axis
  qglColor( QColor{ 0, 255, 0 } );
  glLineWidth( 2.0 );
  glBegin( GL_LINES );
  glVertex4f( 0.0, 0.0, 0.0, 1.0 );
  glVertex4f( 0.0, 1.0, 0.0, 0.0 );
  glEnd();

  // Draw the negative y axis
  qglColor( QColor{ 0, 255, 0 } );
  glLineWidth( 2.0 );
  glLineStipple( 8, 0xAAAA );
  glEnable( GL_LINE_STIPPLE );
  glBegin( GL_LINES );
  glVertex4f( 0.0, 0.0, 0.0, 1.0 );
  glVertex4f( 0.0, -1.0, 0.0, 0.0 );
  glEnd();
  glDisable( GL_LINE_STIPPLE );
}

void GLWidget::getViewportDimensions( GLint& width, GLint& height ) const
{
  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  width = viewport[2];
  height = viewport[3];
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

  updateGL();
}

void GLWidget::centerCamera( const bool update_gl )
{
  if( m_lock_camera )
  {
    return;
  }

  if( m_sim.empty() )
  {
    m_camera_controller.reset();
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

  GLint width;
  GLint height;
  getViewportDimensions( width, height );

  const scalar ratio{ scalar( height ) / scalar( width ) };
  const scalar size{ 1.2 * std::max( ratio * rx, ry ) };

  m_camera_controller.setCenter( cx, cy );
  m_camera_controller.setScaleFactor( size );

  if( update_gl )
  {
    m_camera_controller.reshape( width, height );
    updateGL();
  }
}

void GLWidget::saveScreenshot( const QString& file_name )
{
  std::cout << "Saving screenshot of time " << m_iteration * scalar( m_dt ) << " to " << file_name.toStdString() << std::endl;
  const QImage frame_buffer{ grabFrameBuffer() };
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
    ++m_output_frame;
  }
}

void GLWidget::setMovieFPS( const unsigned fps )
{
  assert( fps > 0 );
  m_output_fps = fps;
  m_output_frame = 0;
  if( 1.0 < scalar( m_dt * std::intmax_t( m_output_fps ) ) )
  {
    std::cerr << "Warning, requested movie frame rate faster than timestep. Dumping at timestep rate." << std::endl;
    m_steps_per_frame = 1;
  }
  else
  {
    const Rational<std::intmax_t> potential_steps_per_frame{ std::intmax_t( 1 ) / ( m_dt * std::intmax_t( m_output_fps ) ) };
    if( !potential_steps_per_frame.isInteger() )
    {
      if( m_dt != Rational<std::intmax_t>{ 0 } )
      {
        std::cerr << "Warning, timestep and output frequency do not yield an integer number of timesteps for data output. Dumping at timestep rate." << std::endl;
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
  std::cout << "<camera cx=\"" << m_camera_controller.centerX() << "\" cy=\"" << m_camera_controller.centerY() << "\" scale_factor=\"" << m_camera_controller.scaleFactor() << "\" fps=\"" << m_output_fps << "\" render_at_fps=\"" << m_render_at_fps << "\" locked=\"" << m_lock_camera << "\"/>" << std::endl;
}

static void paintInfiniteLine( const Vector2s& x, const Vector2s& n )
{
  const scalar theta{ - scalar( 180.0 ) * atan2( n.x(), n.y() ) / MathDefines::PI<scalar>() };

  glPushMatrix();

  glTranslated( GLdouble( x.x() ), GLdouble( x.y() ), GLdouble( 0.0 ) );
  glRotated( GLdouble( theta ), GLdouble( 0.0 ), GLdouble( 0.0 ), GLdouble( 1.0 ) );

  glBegin( GL_LINES );
  glVertex4d(  0.0,  0.0, 0.0, 1.0 );
  glVertex4d(  1.0,  0.0, 0.0, 0.0 );
  glVertex4d(  0.0,  0.0, 0.0, 1.0 );
  glVertex4d( -1.0,  0.0, 0.0, 0.0 );
  glEnd();

  glPopMatrix();
}

static void paintSolidHalfPlane( const Vector2s& x, const Vector2s& n )
{
  paintInfiniteLine( x, n );

  //const scalar theta = -180.0 * atan2( n.x(), n.y() ) / 3.14159265359;
  //
  //glPushMatrix();
  //
  //glTranslated( (GLdouble) x.x(), (GLdouble) x.y(), (GLdouble) 0.0 );
  //glRotated( (GLdouble) theta, 0.0, (GLdouble) 0.0, (GLdouble) 1.0 );
  //
  //glBegin( GL_TRIANGLES );
  //glVertex4d(  0.0,  0.0, 0.0, 1.0 );
  //glVertex4d(  1.0,  0.0, 0.0, 0.0 );
  //glVertex4d(  0.0, -1.0, 0.0, 0.0 );
  //glVertex4d(  0.0,  0.0, 0.0, 1.0 );
  //glVertex4d( -1.0,  0.0, 0.0, 0.0 );
  //glVertex4d(  0.0, -1.0, 0.0, 0.0 );
  //glEnd();
  //
  //glPopMatrix();
}

// TODO: Abstract the shared code in here into its own function
static void paintPlanarPortal( const PlanarPortal& planar_portal )
{
  // Draw the first plane of the portal
  {
    const scalar theta{ -180.0 * atan2( planar_portal.planeA().n().x(), planar_portal.planeA().n().y() ) / MathDefines::PI<scalar>() };

    glPushMatrix();
    glTranslated( GLdouble( planar_portal.planeAx() ), GLdouble( planar_portal.planeAy() ), 0.0 );
    glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

    glLineStipple( 8, 0xAAAA );
    glEnable( GL_LINE_STIPPLE );
    glBegin( GL_LINES );
    glVertex4d( 0.0, 0.0, 0.0, 1.0 );
    glVertex4d( -1.0, 0.0, 0.0, 0.0 );
    glEnd();
    glDisable( GL_LINE_STIPPLE );

    glLineStipple( 8, 0x5555 );
    glEnable( GL_LINE_STIPPLE );
    glBegin( GL_LINES );
    glVertex4d( 0.0, 0.0, 0.0, 1.0 );
    glVertex4d( 1.0, 0.0, 0.0, 0.0 );
    glEnd();
    glDisable( GL_LINE_STIPPLE );

    glPopMatrix();

    assert( planar_portal.boundsA()(0) < planar_portal.boundsA()(1) );
    assert( ( planar_portal.boundsA()(0) != -SCALAR_INFINITY && planar_portal.boundsA()(1) != SCALAR_INFINITY ) || ( planar_portal.boundsA()(0) == -SCALAR_INFINITY && planar_portal.boundsA()(1) == SCALAR_INFINITY ) );
    if( planar_portal.boundsA()(0) != -SCALAR_INFINITY )
    {
      // Draw the lower bound
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeA().x().x() ), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );
      glTranslated( planar_portal.boundsA()(0), 0.0, 0.0 );
      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex4d( 0.0, 0.0, 0.0, 1.0 );
      glVertex4d( 0.0, -1.0, 0.0, 0.0 );
      glEnd();
      glDisable( GL_LINE_STIPPLE );
      glPopMatrix();

      // Draw the upper bound
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeA().x().x() ), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );
      glTranslated( planar_portal.boundsA()(1), 0.0, 0.0 );
      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex4d( 0.0, 0.0, 0.0, 1.0 );
      glVertex4d( 0.0, -1.0, 0.0, 0.0 );
      glEnd();
      glDisable( GL_LINE_STIPPLE );
      glPopMatrix();

      // Draw a short line to indicate the current position of the center of the portal
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeAx() ), GLdouble( planar_portal.planeAy() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

      // Draw an infinite line to show what half of portal is free
      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex2d( 0.0, 0.0 );
      glVertex2d( 0.0, - 0.1 * ( planar_portal.boundsA()(1) - planar_portal.boundsA()(0) ) );
      glEnd();
      glDisable( GL_LINE_STIPPLE );

      glPopMatrix();
    }
    else
    {
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeAx() ), GLdouble( planar_portal.planeAy() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

      // Draw an infinite line to show what half of portal is free
      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex4d( 0.0, 0.0, 0.0, 1.0 );
      glVertex4d( 0.0, -1.0, 0.0, 0.0 );
      glEnd();
      glDisable( GL_LINE_STIPPLE );

      glPopMatrix();
    }
  }

  // Draw the second plane of the portal
  {
    const scalar theta{ -180.0 * atan2( planar_portal.planeB().n().x(), planar_portal.planeB().n().y() ) / MathDefines::PI<scalar>() };

    glPushMatrix();
    glTranslated( GLdouble( planar_portal.planeBx() ), GLdouble( planar_portal.planeBy() ), 0.0 );
    glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

    glLineStipple( 8, 0xAAAA );
    glEnable( GL_LINE_STIPPLE );
    glBegin( GL_LINES );
    glVertex4d( 0.0, 0.0, 0.0, 1.0 );
    glVertex4d( -1.0, 0.0, 0.0, 0.0 );
    glEnd();
    glDisable( GL_LINE_STIPPLE );

    glLineStipple( 8, 0x5555 );
    glEnable( GL_LINE_STIPPLE );
    glBegin( GL_LINES );
    glVertex4d( 0.0, 0.0, 0.0, 1.0 );
    glVertex4d( 1.0, 0.0, 0.0, 0.0 );
    glEnd();
    glDisable( GL_LINE_STIPPLE );

    glPopMatrix();

    assert( planar_portal.boundsB()(0) < planar_portal.boundsB()(1) );
    assert( ( planar_portal.boundsB()(0) != -SCALAR_INFINITY && planar_portal.boundsB()(1) != SCALAR_INFINITY ) || ( planar_portal.boundsB()(0) == -SCALAR_INFINITY && planar_portal.boundsB()(1) == SCALAR_INFINITY ) );
    if( planar_portal.boundsA()(0) != -SCALAR_INFINITY )
    {
      // Draw the lower bound
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeB().x().x() ), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );
      glTranslated( planar_portal.boundsB()(0), 0.0, 0.0 );
      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex4d( 0.0, 0.0, 0.0, 1.0 );
      glVertex4d( 0.0, -1.0, 0.0, 0.0 );
      glEnd();
      glDisable( GL_LINE_STIPPLE );
      glPopMatrix();

      // Draw the upper bound
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeB().x().x() ), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );
      glTranslated( planar_portal.boundsB()(1), 0.0, 0.0 );
      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex4d( 0.0, 0.0, 0.0, 1.0 );
      glVertex4d( 0.0, -1.0, 0.0, 0.0 );
      glEnd();
      glDisable( GL_LINE_STIPPLE );
      glPopMatrix();

      // Draw a short line to indicate the current position of the center of the portal
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeBx() ), GLdouble( planar_portal.planeBy() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

      // Draw an infinite line to show what half of portal is free
      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex2d( 0.0, 0.0 );
      glVertex2d( 0.0, - 0.1 * ( planar_portal.boundsB()(1) - planar_portal.boundsB()(0) ) );
      glEnd();
      glDisable( GL_LINE_STIPPLE );

      glPopMatrix();
    }
    else
    {
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeBx() ), GLdouble( planar_portal.planeBy() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

      // Draw an infinite line to show what half of portal is free
      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex4d( 0.0, 0.0, 0.0, 1.0 );
      glVertex4d( 0.0, -1.0, 0.0, 0.0 );
      glEnd();
      glDisable( GL_LINE_STIPPLE );

      glPopMatrix();
    }
  }
}

void GLWidget::paintSystem() const
{
  const Ball2DState& state{ m_sim.state() };

  // Draw each static drum
  glPushAttrib( GL_COLOR );
  qglColor( QColor{ 0, 0, 0 } );
  {
    const std::vector<StaticDrum>& drums = state.staticDrums();
    for( std::vector<StaticDrum>::size_type i = 0; i < drums.size(); ++i )
    {
      m_circle_renderer.renderSolidCircle( drums[i].x(), drums[i].r() );
    }
  }
  glPopAttrib();

  // Draw each planar portal
  glPushAttrib( GL_COLOR );
  glPushAttrib( GL_LINE_WIDTH );
  glLineWidth( 2.0 );
  {
    // TODO: Create a set number of nice looking colors for the portal
    std::mt19937_64 mt{ 123456 };
    std::uniform_int_distribution<int> color_gen{ 0, 255 };
    const std::vector<PlanarPortal>& planar_portals = state.planarPortals();
    for( const PlanarPortal& planar_portal : planar_portals )
    {
      const int r = color_gen( mt );
      const int g = color_gen( mt );
      const int b = color_gen( mt );
      qglColor( QColor( r, g, b ) );
      paintPlanarPortal( planar_portal );
    }
  }
  glPopAttrib();
  glPopAttrib();

  // Draw each static plane
  glPushAttrib( GL_COLOR );
  qglColor( QColor{ 0, 0, 0 } );
  {
    const std::vector<StaticPlane>& planes = state.staticPlanes();
    for( const StaticPlane& plane : planes )
    {
      paintSolidHalfPlane( plane.x(), plane.n() );
    }
  }
  glPopAttrib();

  // Draw each ball
  glPushAttrib( GL_COLOR );
  {
    const VectorXs& q{ state.q() };
    const VectorXs& r{ state.r() };
    assert( q.size() == 2 * r.size() );
    assert( m_ball_colors.size() == 3 * r.size() );
    for( int i = 0; i < r.size(); ++i )
    {
      glColor3d( m_ball_colors( 3 * i + 0 ), m_ball_colors( 3 * i + 1 ), m_ball_colors( 3 * i + 2 ) );
      m_circle_renderer.renderSolidCircle( q.segment<2>( 2 * i ), r( i ) );
    }
  }
  glPopAttrib();
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
  static int text_width{ 0 };

  glPushAttrib( GL_MATRIX_MODE );
  glMatrixMode( GL_PROJECTION );
  glPushMatrix();
  glMatrixMode( GL_MODELVIEW );
  glPushMatrix();

  // Set an orthographic projection with height and width equal to window height and width
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  GLint width;
  GLint height;
  getViewportDimensions( width, height );
  glOrtho( 0, width, 0, height, -1, 1 );
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  
  // Enable blending for transparent HUD elements
  glPushAttrib( GL_BLEND );
  glEnable( GL_BLEND );
	glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  // Draw a semi-transparent overlay so text is visible regardless of background color
  const Eigen::Matrix<GLdouble,2,1> overlay_start{ 0, height - 5 * 12 - 2 };
  const Eigen::Matrix<GLdouble,2,1> overlay_extnt{ text_width + 2 + 2, height };
  glColor4d( 0.0, 0.0, 0.0, 0.5 );
  glBegin( GL_QUADS );
  glVertex2d( GLdouble( overlay_start.x() ), GLdouble( overlay_start.y() ) );
  glVertex2d( GLdouble( overlay_start.x() + overlay_extnt.x() ), GLdouble( overlay_start.y() ) );
  glVertex2d( GLdouble( overlay_start.x() + overlay_extnt.x() ), GLdouble( overlay_start.y() + overlay_extnt.y() ) );
  glVertex2d( GLdouble( overlay_start.x() ), GLdouble( overlay_start.y() + overlay_extnt.y() ) );
  glEnd();

  glDisable( GL_BLEND );
  glPopAttrib();
  
	glMatrixMode( GL_MODELVIEW );
  glPopMatrix();
  glMatrixMode( GL_PROJECTION );
  glPopMatrix();
  glPopAttrib();

  // String to display in upper left corner
  const QString time_string{ generateTimeString( m_iteration, m_dt, m_display_precision, m_end_time ) };
  const QString delta_H{ generateNumericString( " dH: ", m_delta_H0 ) };
  const QString delta_px{ generateNumericString( "dpx: ", m_delta_p0.x() ) };
  const QString delta_py{ generateNumericString( "dpy: ", m_delta_p0.y() ) };
  const QString delta_L{ generateNumericString( " dL: ", m_delta_L0 ) };
  {
    const QFontMetrics font_metrics{ QFont{ "Courier", 12 } };
    text_width = std::max( text_width, font_metrics.boundingRect( time_string ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_H ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_px ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_py ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_L ).width() );
  }

  qglColor( QColor{ 255, 255, 255 } );
  const QFont font{ "Courier", 12 };
  renderText( 2, font.pointSize(), time_string, font );
  renderText( 2, 2 * font.pointSize(), delta_H, font );
  renderText( 2, 3 * font.pointSize(), delta_px, font );
  renderText( 2, 4 * font.pointSize(), delta_py, font );
  renderText( 2, 5 * font.pointSize(), delta_L, font );

  assert( checkGLErrors() );
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
    updateGL();
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
    updateGL();
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
    m_camera_controller.translateView( dx, dy );
    repaint_needed = true;
  }

  if( event->buttons() & Qt::RightButton )
  {
    assert( m_right_mouse_button_pressed );
    m_camera_controller.zoomView( 0.02 * m_camera_controller.scaleFactor() * dy );
    repaint_needed = true;
  }

  if( repaint_needed )
  {
    updateGL();
  }

  assert( checkGLErrors() );
}

void GLWidget::wheelEvent( QWheelEvent* event )
{
  if( m_lock_camera )
  {
    return;
  }

  m_camera_controller.zoomView( -0.002 * m_camera_controller.scaleFactor() * event->delta() );
  updateGL();
  assert( checkGLErrors() );
}
