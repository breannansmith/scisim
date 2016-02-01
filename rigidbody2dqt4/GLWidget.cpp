#include "GLWidget.h"

#include <QtGui>
#include <QtOpenGL>
#include <iostream>

#include "scisim/Utilities.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "scisim/ConstrainedMaps/FrictionSolver.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"

#include "rigidbody2d/BoxGeometry.h"
#include "rigidbody2d/CircleGeometry.h"
#include "rigidbody2d/RigidBody2DState.h"

#include "RigidBody2DUtils/RigidBody2DSceneParser.h"
#include "RigidBody2DUtils/CameraSettings2D.h"

#include "BoxGeometryRenderer.h"
#include "CircleGeometryRenderer.h"

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

static void getViewportDimensions( GLint& width, GLint& height )
{
  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  width = viewport[2];
  height = viewport[3];
}

void GLWidget::generateRenderers( const std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry )
{
  m_body_renderers.clear();
  for( const std::unique_ptr<RigidBody2DGeometry>& geo : geometry )
  {
    switch( geo->type() )
    {
      case RigidBody2DGeometryType::CIRCLE:
      {
        const CircleGeometry& circle{ sd_cast<CircleGeometry&>( *geo ) };
        m_body_renderers.emplace_back( new CircleGeometryRenderer{ circle, m_circle_renderer } );
        break;
      }
      case RigidBody2DGeometryType::BOX:
      {
        const BoxGeometry& box{ sd_cast<BoxGeometry&>( *geo ) };
        m_body_renderers.emplace_back( new BoxGeometryRenderer{ box } );
        break;
      }
    }
  }
}

void GLWidget::updateRenderers( const std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry )
{
  assert( m_body_renderers.size() < geometry.size() );

  const unsigned num_old_geo{ static_cast<unsigned>( m_body_renderers.size() ) };

  m_body_renderers.reserve( geometry.size() );
  for( unsigned new_idx = num_old_geo; new_idx < geometry.size(); ++new_idx )
  {
    switch( geometry[new_idx]->type() )
    {
      case RigidBody2DGeometryType::CIRCLE:
      {
        const CircleGeometry& circle{ sd_cast<CircleGeometry&>( *geometry[new_idx] ) };
        m_body_renderers.emplace_back( new CircleGeometryRenderer{ circle, m_circle_renderer } );
        break;
      }
      case RigidBody2DGeometryType::BOX:
      {
        std::cerr << "BOX not handled in GLWidget::updateRenderers" << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
  }
  assert( geometry.size() == m_body_renderers.size() );
}

GLWidget::GLWidget( QWidget* parent )
: QGLWidget( QGLFormat( QGL::SampleBuffers ), parent )
, m_camera_controller()
, m_render_at_fps( false )
, m_lock_camera( false )
, m_last_pos()
, m_left_mouse_button_pressed( false )
, m_right_mouse_button_pressed( false )
, m_circle_renderer( 32 )
, m_body_renderers()
, m_body_colors()
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
, m_CoR( SCALAR_NAN )
, m_mu( SCALAR_NAN )
, m_sim0()
, m_sim()
, m_H0()
, m_p0()
, m_L0()
, m_delta_H0( 0.0 )
, m_delta_p0( Vector2s::Zero() )
, m_delta_L0( 0.0 )
, m_render_contacts( false )
, m_collision_points()
, m_collision_normals()
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
  // Specified as a float
  if( dt_string.find( '.' ) != std::string::npos )
  {
    return int( StringUtilities::computeNumCharactersToRight( dt_string, '.' ) );
  }
  // Specified as a rational
  else if( dt_string.find( '/' ) != std::string::npos )
  {
    std::string converted_dt_string;
    std::stringstream ss;
    ss << std::fixed << scalar( dt );
    ss >> converted_dt_string;
    return int( StringUtilities::computeNumCharactersToRight( converted_dt_string, '.' ) );
  }
  // Specified as an int
  else
  {
    return dt_string.length();
  }
}

// TODO: Initialize colors directly in HSV of CMYK and save as QColor objects
void GLWidget::generateBodyColors()
{
  m_body_color_gen.seed( 1337 );
  // Generate a random color for each ball
  m_body_colors.resize( m_sim.state().q().size() );
  {
    std::uniform_real_distribution<scalar> color_gen{ 0.0, 1.0 };
    for( int i = 0; i < m_body_colors.size(); i += 3 )
    {
      scalar r = 1.0;
      scalar g = 1.0;
      scalar b = 1.0;
      // Generate colors until we get one with a luminance within [0.1,0.9]
      while( ( 0.2126 * r + 0.7152 * g + 0.0722 * b ) > 0.9 || ( 0.2126 * r + 0.7152 * g + 0.0722 * b ) < 0.1 )
      {
        r = color_gen( m_body_color_gen );
        g = color_gen( m_body_color_gen );
        b = color_gen( m_body_color_gen );
      }
      m_body_colors.segment<3>( i ) << r, g, b;
    }
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
  std::unique_ptr<UnconstrainedMap> new_unconstrained_map{ nullptr };
  std::unique_ptr<ImpactOperator> new_impact_operator{ nullptr };
  std::unique_ptr<ImpactMap> new_imap{ nullptr };
  std::unique_ptr<FrictionSolver> new_friction_solver{ nullptr };
  std::unique_ptr<ImpactFrictionMap> new_impact_friction_map{ nullptr };

  std::string dt_string;
  std::string scripting_callback;
  RigidBody2DState new_state;
  Rational<std::intmax_t> dt;
  scalar end_time;
  scalar CoR;
  scalar mu;
  CameraSettings2D camera_settings;

  const bool loaded_successfully{ RigidBody2DSceneParser::parseXMLSceneFile( xml_scene_file_name.toStdString(), scripting_callback, new_state, new_unconstrained_map, dt_string, dt, end_time, new_impact_operator, new_imap, CoR, new_friction_solver, mu, new_impact_friction_map, camera_settings ) };

  if( !loaded_successfully )
  {
    std::cerr << "Failed to load file: " << xml_scene_file_name.toStdString() << std::endl;
    return false;
  }

  // Ensure we have a correct combination of maps.
  assert( ( new_unconstrained_map != nullptr && new_impact_operator == nullptr && new_friction_solver == nullptr && new_impact_friction_map == nullptr && new_imap == nullptr ) ||
         ( new_unconstrained_map != nullptr && new_impact_operator != nullptr && new_friction_solver == nullptr && new_impact_friction_map == nullptr && new_imap != nullptr ) ||
         ( new_unconstrained_map != nullptr && new_impact_operator == nullptr && new_friction_solver != nullptr && new_impact_friction_map != nullptr && new_imap == nullptr ) );

  m_CoR = CoR;
  m_mu = mu;

  // Compute the number of characters after the decimal point in the timestep string
  m_display_precision = computeTimestepDisplayPrecision( m_dt, dt_string );

  m_unconstrained_map.swap( new_unconstrained_map );
  m_impact_operator.swap( new_impact_operator );
  m_friction_solver.swap( new_friction_solver );
  m_if_map.swap( new_impact_friction_map );
  m_imap.swap( new_imap );

  // Initialize the scripting callback
  {
    PythonScripting new_scripting{ xmlFilePath( xml_scene_file_name.toStdString() ), scripting_callback };
    using std::swap;
    swap( m_scripting, new_scripting );
  }

  m_sim = RigidBody2DSim{ new_state };

  // Backup the simulation
  m_sim0 = m_sim;

  // Initially, no change in energy
  m_H0 = m_sim.computeTotalEnergy();
  m_p0 = m_sim.computeTotalMomentum();
  m_L0 = m_sim.computeTotalAngularMomentum();
  m_delta_H0 = 0.0;
  m_delta_p0.setZero();
  m_delta_L0 = 0.0;

  // Cache the initial contacts for rendering, if needed
  if( m_render_contacts )
  {
    m_sim.computeContactPoints( m_collision_points, m_collision_normals );
  }

  generateBodyColors();
  generateRenderers( m_sim.state().geometry() );

  // Save the timestep and compute related quantities
  m_dt = dt;
  assert( m_dt.nonNegative() );
  m_iteration = 0;
  m_end_time = end_time;
  assert( m_end_time > 0.0 );

  m_output_fps = camera_settings.fps;
  m_render_at_fps = camera_settings.render_at_fps;
  m_lock_camera = camera_settings.locked;
  setMovieFPS( m_output_fps );

  // Set the camera
  {
    const bool lock_backup{ m_lock_camera };
    m_lock_camera = false;

    // Set the camera
    if( camera_settings.set )
    {
      m_camera_controller.setCenter( camera_settings.center.x(), camera_settings.center.y() );
      m_camera_controller.setScaleFactor( camera_settings.scale );
    }
    else
    {
      centerCamera( false );
    }

    m_lock_camera = lock_backup;
  }

  // User-provided start of simulation python callback
  m_scripting.setState( m_sim.state() );
  m_scripting.startOfSimCallback();
  m_scripting.forgetState();

  if( render_on_load )
  {
    GLint width;
    GLint height;
    getViewportDimensions( width, height );
    m_camera_controller.reshape( width, height );
    updateGL();
  }

  // For the parent to update the UI
  fps = m_output_fps;
  render_at_fps = m_render_at_fps;
  lock_camera = m_lock_camera;

  return true;
}

void GLWidget::stepSystem()
{
  if( m_iteration * scalar( m_dt ) >= m_end_time )
  {
    // User-provided end of simulation python callback
    m_scripting.setState( m_sim.state() );
    m_scripting.endOfSimCallback();
    m_scripting.forgetState();
    std::cout << "Simulation complete. Exiting." << std::endl;
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
    std::cerr << "Impossible code path hit. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  ++m_iteration;

  m_delta_H0 = std::max( m_delta_H0, fabs( m_H0 - m_sim.computeTotalEnergy() ) );
  {
    const Vector2s p{ m_sim.computeTotalMomentum() };
    m_delta_p0.x() = std::max( m_delta_p0.x(), fabs( m_p0.x() - p.x() ) );
    m_delta_p0.y() = std::max( m_delta_p0.y(), fabs( m_p0.y() - p.y() ) );
  }
  m_delta_L0 = std::max( m_delta_L0, fabs( m_L0 - m_sim.computeTotalAngularMomentum() ) );

  // Cache the contacts for rendering, if needed
  if( m_render_contacts )
  {
    m_sim.computeContactPoints( m_collision_points, m_collision_normals );
  }

  // If the number of bodies changed
  if( m_body_colors.size() != m_sim.state().q().size() )
  {
    // TODO: Could get expensive?
    generateBodyColors();
  }

  // If the geometry count changed
  if( m_body_renderers.size() != m_sim.state().geometry().size() )
  {
    updateRenderers( m_sim.state().geometry() );
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
      QString output_image_name{ QString{ tr( "frame%1.png" ) }.arg( m_output_frame, 10, 10, QLatin1Char{ '0' } ) };
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

  // Cache the initial contacts for rendering, if needed
  if( m_render_contacts )
  {
    m_sim.computeContactPoints( m_collision_points, m_collision_normals );
  }

  generateBodyColors();
  generateRenderers( m_sim.state().geometry() );

  // User-provided start of simulation python callback
  m_scripting.setState( m_sim.state() );
  m_scripting.startOfSimCallback();
  m_scripting.forgetState();

  updateGL();
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

  const GLint width{ sizeHint().width() };
  const GLint height{ sizeHint().height() };

  #ifndef NDEBUG
  if( update_gl )
  {
    GLint gl_width;
    GLint gl_height;
    getViewportDimensions( gl_width, gl_height );
    assert( gl_width == width );
    assert( gl_height == height );
  }
  #endif

  if( m_sim.state().q().size() == 0 )
  {
    m_camera_controller.reset();
  }
  else
  {
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

    const scalar ratio{ scalar( height ) / scalar( width ) };
    const scalar size{ 1.2 * std::max( ratio * rx, ry ) };

    m_camera_controller.setCenter( cx, cy );
    m_camera_controller.setScaleFactor( size );
  }

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

    QString output_image_name{ QString{ tr( "frame%1.png" ) }.arg( m_output_frame, 10, 10, QLatin1Char('0') ) };
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
  std::cout << "<camera center=\"" << m_camera_controller.centerX() << " " << m_camera_controller.centerY() << "\" scale=\"" << m_camera_controller.scaleFactor() << "\" fps=\"" << m_output_fps << "\" render_at_fps=\"" << m_render_at_fps << "\" locked=\"" << m_lock_camera << "\"/>" << std::endl;
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

// TODO: Abstract the shared code in here into its own function
static void paintPlanarPortal( const PlanarPortal& planar_portal )
{
  // Draw the first plane of the portal
  {
    const scalar theta{ -180.0 * atan2( planar_portal.planeA().n().x(), planar_portal.planeA().n().y() ) / MathDefines::PI<scalar>() };

    glPushMatrix();
    glTranslated( GLdouble( planar_portal.planeA().x().x() ), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
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

    assert( planar_portal.bounds() >= 0.0 );
    if( planar_portal.isLeesEdwards() )
    {
      // Draw the lower bound
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeA().x().x() ), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );
      glTranslated( -planar_portal.bounds(), 0.0, 0.0 );
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
      glTranslated( planar_portal.bounds(), 0.0, 0.0 );
      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex4d( 0.0, 0.0, 0.0, 1.0 );
      glVertex4d( 0.0, -1.0, 0.0, 0.0 );
      glEnd();
      glDisable( GL_LINE_STIPPLE );
      glPopMatrix();

      // Draw a short line to indicate the rest position of the center of the portal
      glPushMatrix();
      glTranslated( planar_portal.planeA().x().x(), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex2d( 0.0, 0.0 );
      glVertex2d( 0.0, - 0.2 * planar_portal.bounds() );
      glEnd();
      glDisable( GL_LINE_STIPPLE );

      glPopMatrix();
      
      // Draw a short line to indicate the current position of the center of the portal
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.transformedAx().x() ), GLdouble( planar_portal.transformedAx().y() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex2d( 0.0, 0.0 );
      glVertex2d( 0.0, - 0.2 * planar_portal.bounds() );
      glEnd();
      glDisable( GL_LINE_STIPPLE );

      glPopMatrix();
    }
    else
    {
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeA().x().x() ), GLdouble( planar_portal.planeA().x().y() ), 0.0 );
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
    glTranslated( GLdouble( planar_portal.planeB().x().x() ), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
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

    assert( planar_portal.bounds() >= 0.0 );
    if( planar_portal.isLeesEdwards() )
    {
      // Draw the lower bound
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeB().x().x() ), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );
      glTranslated( -planar_portal.bounds(), 0.0, 0.0 );
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
      glTranslated( planar_portal.bounds(), 0.0, 0.0 );
      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex4d( 0.0, 0.0, 0.0, 1.0 );
      glVertex4d( 0.0, -1.0, 0.0, 0.0 );
      glEnd();
      glDisable( GL_LINE_STIPPLE );
      glPopMatrix();

      // Draw a short line to indicate the rest position of the center of the portal
      glPushMatrix();
      glTranslated( planar_portal.planeB().x().x(), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex2d( 0.0, 0.0 );
      glVertex2d( 0.0, - 0.2 * planar_portal.bounds() );
      glEnd();
      glDisable( GL_LINE_STIPPLE );

      glPopMatrix();

      // Draw a short line to indicate the current position of the center of the portal
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.transformedBx().x() ), GLdouble( planar_portal.transformedBx().y() ), 0.0 );
      glRotated( GLdouble( theta ), 0.0, 0.0, 1.0 );

      glLineStipple( 8, 0x5555 );
      glEnable( GL_LINE_STIPPLE );
      glBegin( GL_LINES );
      glVertex2d( 0.0, 0.0 );
      glVertex2d( 0.0, - 0.2 * planar_portal.bounds() );
      glEnd();
      glDisable( GL_LINE_STIPPLE );

      glPopMatrix();
    }
    else
    {
      glPushMatrix();
      glTranslated( GLdouble( planar_portal.planeB().x().x() ), GLdouble( planar_portal.planeB().x().y() ), 0.0 );
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
  // Draw each body
  {
    const VectorXs& q{ m_sim.state().q() };
    assert( q.size() % 3 == 0 );
    const unsigned nbodies{ static_cast<unsigned>( q.size() / 3 ) };
    for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
    {
      glPushMatrix();
      glTranslated( GLdouble( q( 3 * bdy_idx + 0 ) ), GLdouble( q( 3 * bdy_idx + 1 ) ), GLdouble( 0.0 ) );
      const scalar theta_degrees{ 180.0 * q( 3 * bdy_idx + 2 ) / MathDefines::PI<scalar>() };
      glRotated( theta_degrees, 0.0, 0.0, 1.0 );
      assert( bdy_idx < m_body_colors.size() / 3 );
      assert( bdy_idx < m_sim.state().geometryIndices().size() );
      assert( m_sim.state().geometryIndices()(bdy_idx) < m_body_renderers.size() );
      if( !m_sim.isKinematicallyScripted( bdy_idx ) )
      {
        m_body_renderers[ m_sim.state().geometryIndices()(bdy_idx) ]->render( m_body_colors.segment<3>( 3 * bdy_idx ) );
      }
      else
      {
        m_body_renderers[ m_sim.state().geometryIndices()(bdy_idx) ]->render( Vector3s{ 0.5, 0.5, 0.5 } );
      }
      glPopMatrix();
    }
  }

  // Draw teleported versions of each body
  {
    const VectorXs& q{ m_sim.state().q() };
    assert( q.size() % 3 == 0 );
    const unsigned nbodies{ static_cast<unsigned>( q.size() / 3 ) };
    const std::vector<PlanarPortal>& planar_portals{ m_sim.state().planarPortals() };
    // For each planar portal
    // TODO: More efficient to invert these loops, only cache out pos, theta once per body
    for( const PlanarPortal& planar_portal : planar_portals )
    {
      for( unsigned bdy_idx = 0; bdy_idx < nbodies; ++bdy_idx )
      {
        const Vector2s pos{ q.segment<2>( 3 * bdy_idx ) };
        const scalar theta{ q( 3 * bdy_idx + 2) };
        // Compute the AABB for the current body
        Array2s min;
        Array2s max;
        m_sim.state().bodyGeometry( bdy_idx )->computeAABB( pos, theta, min, max );
        assert( ( min < max ).all() );
        // If the AABB intersects a periodic boundary
        bool intersecting_index;
        if( planar_portal.aabbTouchesPortal( min, max, intersecting_index ) )
        {
          Vector2s teleported_pos;
          planar_portal.teleportPoint( pos, intersecting_index, teleported_pos );
          glPushMatrix();
          glTranslated( GLdouble( teleported_pos.x() ), GLdouble( teleported_pos.y() ), GLdouble( 0.0 ) );
          const scalar theta_degrees{ 180.0 * q( 3 * bdy_idx + 2 ) / MathDefines::PI<scalar>() };
          glRotated( theta_degrees, 0.0, 0.0, 1.0 );
          assert( bdy_idx < m_body_colors.size() / 3 );
          assert( bdy_idx < m_sim.state().geometryIndices().size() );
          assert( m_sim.state().geometryIndices()(bdy_idx) < m_body_renderers.size() );
          m_body_renderers[ m_sim.state().geometryIndices()(bdy_idx) ]->renderTeleported( m_body_colors.segment<3>( 3 * bdy_idx ) );
          glPopMatrix();
        }
      }
    }
  }

  if( m_render_contacts )
  {
    assert( m_collision_points.size() == m_collision_normals.size() );
    // TODO: Draw with circles so they scale nicer
    // Draw the contact points
    glPushAttrib( GL_POINT_SIZE );
    glPushAttrib( GL_COLOR );
    glColor3d( 1.0, 0.0, 0.0 );
    glPointSize( 20.0 / m_camera_controller.scaleFactor() );
    glBegin( GL_POINTS );
    for( const Vector2s& point : m_collision_points )
    {
      glVertex2d( point.x(), point.y() );
    }
    glEnd();
    glPopAttrib();
    glPopAttrib();
    // Draw the contact normals
    glPushAttrib( GL_LINE_WIDTH );
    glPushAttrib( GL_COLOR );
    glColor3d( 1.0, 0.0, 0.0 );
    glLineWidth( 4.0 / m_camera_controller.scaleFactor() );
    glBegin( GL_LINES );
    for( std::vector<Vector2s>::size_type idx = 0; idx < m_collision_points.size(); ++idx )
    {
      glVertex2d( m_collision_points[idx].x(), m_collision_points[idx].y() );
      glVertex2d( m_collision_points[idx].x() + 0.5 * m_collision_normals[idx].x(), m_collision_points[idx].y() + 0.5 * m_collision_normals[idx].y() );
    }
    glEnd();
    glPopAttrib();
    glPopAttrib();
  }

  // Draw each planar portal
  glPushAttrib( GL_COLOR );
  glPushAttrib( GL_LINE_WIDTH );
  glLineWidth( 2.0 );
  {
    // TODO: Create a set number of nice looking colors for the portal ahead of time instead of regenerating them
    std::mt19937_64 mt{ 123456 };
    std::uniform_int_distribution<int> color_gen{ 0, 255 };
    const std::vector<PlanarPortal>& planar_portals{ m_sim.state().planarPortals() };
    for( const PlanarPortal& planar_portal : planar_portals )
    {
      const int r{ color_gen( mt ) };
      const int g{ color_gen( mt ) };
      const int b{ color_gen( mt ) };
      qglColor( QColor{ r, g, b } );
      paintPlanarPortal( planar_portal );
    }
  }
  glPopAttrib();
  glPopAttrib();

  // Draw each static plane
  glPushAttrib( GL_COLOR );
  glPushAttrib( GL_LINE_WIDTH );
  glLineWidth( 2.0 );
  qglColor( QColor{ 0, 0, 0 } );
  {
    const std::vector<RigidBody2DStaticPlane>& planes{ m_sim.state().planes() };
    for( const RigidBody2DStaticPlane& plane : planes )
    {
      paintInfiniteLine( plane.x(), plane.n() );
    }
  }
  glPopAttrib();
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
