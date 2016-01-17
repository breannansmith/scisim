#include "GLWidget.h"

#include <QtGui>
#include <QtOpenGL>
#include <cmath>
#include <iostream>
#include <fstream>
#include <random>

#include "RigidBody3DUtils/RigidBody3DSceneParser.h"

#include "scisim/Utilities.h"
#include "scisim/ConstrainedMaps/ImpactFrictionMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactMap.h"
#include "scisim/ConstrainedMaps/ImpactMaps/ImpactOperator.h"
#include "scisim/UnconstrainedMaps/UnconstrainedMap.h"
#include "scisim/ConstrainedMaps/FrictionSolver.h"

#include "rigidbody3d/PythonScripting.h"
#include "rigidbody3d/Geometry/RigidBodyBox.h"
#include "rigidbody3d/Geometry/RigidBodySphere.h"
#include "rigidbody3d/Geometry/RigidBodyTriangleMesh.h"
#include "rigidbody3d/Geometry/RigidBodyStaple.h"
#include "rigidbody3d/StaticGeometry/StaticPlane.h"

#include "RigidBody3DUtils/RenderingState.h"
#include "Rendering/OpenGL3DSphereRenderer.h"
#include "Rendering/BodyGeometryRenderer.h"
#include "Rendering/StapleRenderer.h"

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
, m_use_perspective_camera( true )
, m_perspective_camera_controller()
, m_orthographic_controller()
, m_render_at_fps( false )
, m_lock_camera( false )
, m_last_pos()
, m_left_mouse_button_pressed( false )
, m_right_mouse_button_pressed( false )
, m_body_colors()
, m_display_precision( 0 )
, m_display_HUD( true )
, m_display_xy_grid( false )
, m_display_yz_grid( false )
, m_display_xz_grid( false )
, m_movie_dir_name()
, m_movie_dir()
, m_output_frame( 0 )
, m_output_fps()
, m_steps_per_frame()
, m_unconstrained_map( nullptr )
, m_impact_operator( nullptr )
, m_friction_solver( nullptr )
, m_impact_friction_map( nullptr )
, m_scripting()
, m_iteration( 0 )
, m_dt( 0, 1 )
, m_end_time( SCALAR_INFINITY )
, m_CoR( SCALAR_NAN )
, m_mu( SCALAR_NAN )
, m_sim()
, m_H0()
, m_p0( Vector3s::Zero() )
, m_L0( Vector3s::Zero() )
, m_delta_H0( 0.0 )
, m_delta_p0( Vector3s::Zero() )
, m_delta_L0( Vector3s::Zero() )
, m_sphere_renderer( nullptr )
, m_plane_renderers()
, m_cylinder_renderers()
, m_portal_renderers()
, m_body_renderers()
{}

GLWidget::~GLWidget()
{}

QSize GLWidget::minimumSizeHint() const
{
  return QSize{ 50, 50 };
}

QSize GLWidget::sizeHint() const
{
  return QSize{ 512, 512 };
}

int computeTimestepDisplayPrecision( const Rational<std::intmax_t>& dt, const std::string& dt_string )
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
  std::unique_ptr<UnconstrainedMap> new_unconstrained_map{ nullptr };
  std::unique_ptr<ImpactOperator> new_impact_operator{ nullptr };
  std::unique_ptr<FrictionSolver> new_friction_solver{ nullptr };
  std::unique_ptr<ImpactFrictionMap> new_impact_friction_map{ nullptr };

  std::string dt_string{ "" };
  std::string scripting_callback_name{ "" };
  RigidBody3DState sim_state;
  Rational<std::intmax_t> dt;
  scalar end_time;
  scalar CoR;
  scalar mu;
  RenderingState new_render_state;

  const bool loaded_successfully{ RigidBody3DSceneParser::parseXMLSceneFile( xml_scene_file_name.toStdString(), scripting_callback_name, sim_state, new_unconstrained_map, dt_string, dt, end_time, new_impact_operator, CoR, new_friction_solver, mu, new_impact_friction_map, new_render_state ) };

  if( !loaded_successfully )
  {
    std::cerr << "Failed to load file: " << xml_scene_file_name.toStdString() << std::endl;
    return false;
  }

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
    PythonScripting new_scripting{ path, scripting_callback_name };
    swap( m_scripting, new_scripting );
  }

  m_CoR = CoR;
  m_mu = mu;

  // Compute the number of characters after the decimal point in the timestep string
  m_display_precision = computeTimestepDisplayPrecision( m_dt, dt_string );

  m_unconstrained_map.swap( new_unconstrained_map );
  m_impact_operator.swap( new_impact_operator );
  m_friction_solver.swap( new_friction_solver );
  m_impact_friction_map.swap( new_impact_friction_map );

  m_sim.setState( sim_state );
  m_sim.clearConstraintCache();

  m_H0 = m_sim.computeTotalEnergy();
  m_p0 = m_sim.computeTotalMomentum();
  m_L0 = m_sim.computeTotalAngularMomentum();
  m_delta_H0 = 0.0;
  m_delta_p0 = Vector3s::Zero();
  m_delta_L0 = Vector3s::Zero();

  m_body_colors.resize( 3 * sim_state.nbodies() );
  {
    std::vector<QColor> colors;
    {
      // 0131 U
      const QColor yellow{ 252, 245, 156 };
      colors.emplace_back( yellow );
      // 0331 U
      const QColor red{ 255, 178, 190 };
      colors.emplace_back( red );
      // 0521 U
      const QColor magenta{ 251, 170, 221 };
      colors.emplace_back( magenta );
      // 0631 U
      const QColor violet{ 188, 148, 222 };
      colors.emplace_back( violet );
      // 0821 U
      const QColor blue{ 103, 208, 238 };
      colors.emplace_back( blue );
      // 0921 U
      const QColor green{ 114, 229, 210 };
      colors.emplace_back( green );
    }

    std::mt19937_64 mt{ 0 };
    std::uniform_int_distribution<unsigned> color_slector( 0, colors.size() - 1 );
    for( int i = 0; i < m_body_colors.size(); i += 3 )
    {
      // Select a random color
      const unsigned color_num{ color_slector( mt ) };
      assert( color_num < colors.size() );
      m_body_colors.segment<3>( i ) << colors[color_num].redF(), colors[color_num].greenF(), colors[color_num].blueF();
    }
  }

  // Create a renderer for the system
  m_body_renderers.resize( m_sim.getState().geometry().size() );
  for( std::vector<std::unique_ptr<BodyGeometryRenderer>>::size_type i = 0; i < m_sim.getState().geometry().size(); ++i )
  {
    if( m_sim.getState().geometry()[i]->getType() == RigidBodyGeometryType::STAPLE )
    {
      const RigidBodyStaple& geo{ sd_cast<const RigidBodyStaple&>( *m_sim.getState().geometry()[i] ) };
      m_body_renderers[i].reset( new StapleRenderer{ 4, geo.points(), geo.r() } );
    }
    else
    {
      m_body_renderers[i].reset( nullptr );
    }
  }

  // Save the timestep and compute related quantities
  m_dt = dt;
  assert( m_dt.positive() );
  m_iteration = 0;
  m_end_time = end_time;
  assert( m_end_time > 0.0 );

  m_output_fps = new_render_state.FPS();
  m_render_at_fps = new_render_state.renderAtFPS();
  m_lock_camera = new_render_state.locked();

  // Output fps was set in ContentWidget's constructor
  assert( m_output_fps > 0 );
  setMovieFPS( m_output_fps );

  // Backup the simulation state
  assert( m_sim.getState().q().size() == 12 * m_sim.getState().nbodies() );
  m_sim_state_backup = m_sim.getState();

  initializeRenderingSettings( new_render_state );

  // If there are any intitial collisions, warn the user
  {
    std::map<std::string,unsigned> collision_counts;
    std::map<std::string,scalar> collision_depths;
    std::map<std::string,scalar> overlap_volumes;
    m_sim.computeNumberOfCollisions( collision_counts, collision_depths, overlap_volumes );
    assert( collision_counts.size() == collision_depths.size() ); assert( collision_counts.size() == overlap_volumes.size() );
    if( !collision_counts.empty() )
    {
      std::cout << "Warning, initial collisions detected (name : count : total_depth : total_volume):" << std::endl;
    }
    for( const std::pair<std::string,unsigned>& count_pair : collision_counts )
    {
      const std::string& constraint_name{ count_pair.first };
      const unsigned& constraint_count{ count_pair.second };
      assert( collision_depths.find( constraint_name ) != collision_depths.end() );
      const scalar& constraint_depth{ collision_depths[constraint_name] };
      const scalar& constraint_volume{ overlap_volumes[constraint_name] };
      std::string depth_string;
      if( !std::isnan( constraint_depth ) )
      {
        depth_string = StringUtilities::convertToString( constraint_depth );
      }
      else
      {
        depth_string = "depth_computation_not_supported";
      }
      std::string volume_string;
      if( !std::isnan( constraint_volume ) )
      {
        volume_string = StringUtilities::convertToString( constraint_volume );
      }
      else
      {
        volume_string = "volume_computation_not_supported";
      }
      std::cout << "   " << constraint_name << " : " << constraint_count << " : " << depth_string << " : " << volume_string << std::endl;
    }
  }

  fps = new_render_state.FPS();
  render_at_fps = new_render_state.renderAtFPS();
  lock_camera = new_render_state.locked();

  // User-provided start of simulation python callback
  m_scripting.setState( m_sim.getState() );
  m_scripting.setInitialIterate( m_iteration );
  m_scripting.startOfSimCallback();
  m_scripting.forgetState();

  if( render_on_load )
  {
    updateGL();
  }

  return true;
}

OrthographicCameraController::ProjectionPlane stringToProjectionPlane( const std::string& projection_plane )
{
  if( projection_plane == "xy" )
  {
    return OrthographicCameraController::ProjectionPlane::XY;
  }
  else if( projection_plane == "zy" )
  {
    return OrthographicCameraController::ProjectionPlane::ZY;
  }
  else if( projection_plane == "zx" )
  {
    return OrthographicCameraController::ProjectionPlane::ZX;
  }

  std::cerr << "Impossible code path in stringToProjectionPlane. This is a bug." << std::endl;
  std::exit( EXIT_FAILURE );
}

void GLWidget::initializeRenderingSettings( const RenderingState& rendering_state )
{
  // Create plane renderers
  m_plane_renderers.clear();
  for( unsigned plane_renderer_index = 0; plane_renderer_index < rendering_state.numPlaneRenderers(); ++plane_renderer_index )
  {
    m_plane_renderers.emplace_back( rendering_state.planeRenderer(plane_renderer_index).index(), rendering_state.planeRenderer(plane_renderer_index).r() );
  }

  // Create cylinder renderers
  m_cylinder_renderers.clear();
  for( unsigned cylinder_renderer_index = 0; cylinder_renderer_index < rendering_state.numCylinderRenderers(); ++cylinder_renderer_index )
  {
    m_cylinder_renderers.emplace_back( rendering_state.cylinderRenderer(cylinder_renderer_index).index(), rendering_state.cylinderRenderer(cylinder_renderer_index).L() );
  }

  // Create the portal renderers
  m_portal_renderers.clear();
  for( unsigned portal_renderer_index = 0; portal_renderer_index < rendering_state.numPlanarPortalRenderers(); ++portal_renderer_index )
  {
    m_portal_renderers.emplace_back( rendering_state.portalRenderer( portal_renderer_index ).portalIndex(), rendering_state.portalRenderer( portal_renderer_index ).halfWidth0(), rendering_state.portalRenderer( portal_renderer_index ).halfWidth1() );
  }

  const bool lock_backup{ m_lock_camera };
  m_lock_camera = false;

  // Set the camera
  {
    GLint width;
    GLint height;
    getViewportDimensions( width, height );
    if( rendering_state.cameraSettingsInitialized() )
    {
      if( rendering_state.perspectiveCameraSelected() )
      {
        m_use_perspective_camera = true;
        m_perspective_camera_controller.setCamera( rendering_state.cameraUp(), rendering_state.cameraTheta(), rendering_state.cameraPhi(), rendering_state.cameraRho(), rendering_state.cameraLookAt() );
        m_perspective_camera_controller.setPerspective( width, height );
      }
      else if( rendering_state.orthographicCameraSelected() )
      {
        m_use_perspective_camera = false;
        m_orthographic_controller.setCamera( stringToProjectionPlane( rendering_state.orthographicProjectionPlane() ), rendering_state.orthographicX(), rendering_state.orthographicScale() );
        m_orthographic_controller.setPerspective( width, height );
      }
      else
      {
        std::cerr << "Impossible code path in GLWidget::initializeRenderingSettings. This is a bug." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
    else
    {
      m_use_perspective_camera = true;
      scalar radius;
      Eigen::Matrix<GLdouble,3,1> center;
      m_sim.computeBoundingSphere( radius, center );
      m_perspective_camera_controller.centerCameraAtSphere( center, radius );
      m_perspective_camera_controller.setPerspective( width, height );
    }
  }

  m_lock_camera = lock_backup;
}

void GLWidget::stepSystem()
{
  if( m_iteration * scalar( m_dt ) >= m_end_time )
  {
    std::cout << "Simulation complete. Exiting." << std::endl;
    // User-provided end of simulation python callback
    m_scripting.setState( m_sim.getState() );
    m_scripting.endOfSimCallback();
    m_scripting.forgetState();
    std::exit( EXIT_SUCCESS );
  }

  const int next_iter{ static_cast<int>( m_iteration + 1 ) };

  if( m_unconstrained_map == nullptr && m_impact_operator == nullptr && m_friction_solver == nullptr )
  {
    return;
  }
  else if( m_unconstrained_map != nullptr && m_impact_operator == nullptr && m_friction_solver == nullptr )
  {
    m_sim.flow( m_scripting, next_iter, m_dt, *m_unconstrained_map );
  }
  else if( m_unconstrained_map != nullptr && m_impact_operator != nullptr && m_friction_solver == nullptr )
  {
    m_sim.flow( m_scripting, next_iter, m_dt, *m_unconstrained_map, *m_impact_operator, m_CoR );
  }
  else if( m_unconstrained_map != nullptr && m_impact_operator == nullptr && m_friction_solver != nullptr && m_impact_friction_map != nullptr )
  {
    m_sim.flow( m_scripting, next_iter, m_dt, *m_unconstrained_map, m_CoR, m_mu, *m_friction_solver, *m_impact_friction_map );
  }
  else
  {
    std::cerr << "Impossible code path hit. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }

  ++m_iteration;

  m_delta_H0 = std::max( m_delta_H0, fabs( m_H0 - m_sim.computeTotalEnergy() ) );
  const Vector3s p{ m_sim.computeTotalMomentum() };
  m_delta_p0.x() = std::max( m_delta_p0.x(), fabs( m_p0.x() - p.x() ) );
  m_delta_p0.y() = std::max( m_delta_p0.y(), fabs( m_p0.y() - p.y() ) );
  m_delta_p0.z() = std::max( m_delta_p0.z(), fabs( m_p0.z() - p.z() ) );
  const Vector3s L{ m_sim.computeTotalAngularMomentum() };
  m_delta_L0.x() = std::max( m_delta_L0.x(), fabs( m_L0.x() - L.x() ) );
  m_delta_L0.y() = std::max( m_delta_L0.y(), fabs( m_L0.y() - L.y() ) );
  m_delta_L0.z() = std::max( m_delta_L0.z(), fabs( m_L0.z() - L.z() ) );

  if( !m_render_at_fps || m_iteration % m_steps_per_frame == 0 )
  {
    updateGL();
  }

  if( m_movie_dir_name.size() != 0 )
  {
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
  m_sim.setState( m_sim_state_backup );
  m_sim.clearConstraintCache();
  if( m_impact_friction_map != nullptr )
  {
    m_impact_friction_map->resetCachedData();
  }

  m_iteration = 0;

  m_H0 = m_sim.computeTotalEnergy();
  m_p0 = m_sim.computeTotalMomentum();
  m_L0 = m_sim.computeTotalAngularMomentum();
  m_delta_H0 = 0.0;
  m_delta_p0 = Vector3s::Zero();
  m_delta_L0 = Vector3s::Zero();

  // User-provided start of simulation python callback
  m_scripting.setState( m_sim.getState() );
  m_scripting.setInitialIterate( m_iteration );
  m_scripting.startOfSimCallback();
  m_scripting.forgetState();

  updateGL();
}

void GLWidget::initializeGL()
{
  glEnable( GL_DEPTH_TEST );
  qglClearColor( QColor{ 255, 255, 255, 255 } );
  glShadeModel( GL_SMOOTH );
  const GLfloat global_ambient[]{ 0.45f, 0.45f, 0.45f, 1.0f };
  glLightModelfv( GL_LIGHT_MODEL_AMBIENT, global_ambient );
  const GLfloat diffuse[]{ 0.7f, 0.7f, 0.7f , 1.0f };
  glLightfv( GL_LIGHT0, GL_DIFFUSE, diffuse );
  glEnable( GL_LIGHTING );
  glEnable( GL_LIGHT0 );
  // TODO: Rework rendering code so GL_NORMALIZE is not needed
  glEnable( GL_NORMALIZE );

  m_sphere_renderer.reset( new OpenGL3DSphereRenderer{ 4 } );

  assert( checkGLErrors() );
}

void GLWidget::resizeGL( int width, int height )
{
  assert( width >= 0 );
  assert( height >= 0 );

  if( m_use_perspective_camera )
  {
    m_perspective_camera_controller.setPerspective( width, height );
  }
  else
  {
    m_orthographic_controller.setPerspective( width, height );
  }

  assert( checkGLErrors() );
}

void GLWidget::paintGL()
{
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();

  if( m_use_perspective_camera )
  {
    m_perspective_camera_controller.positionCamera();
  }
  else
  {
    m_orthographic_controller.positionCamera();
  }
  assert( checkGLErrors() );

  if( axesDrawingIsEnabled() )
  {
    paintAxes();
  }

  if( m_display_xy_grid )
  {
    paintXYRulers();
  }
  
  if( m_display_yz_grid )
  {
    paintYZRulers();
  }
  
  if( m_display_xz_grid )
  {
    paintXZRulers();
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
  glPushAttrib(GL_COLOR);
  glPushAttrib(GL_LIGHTING);
  glPushAttrib(GL_LINE_WIDTH);

  glDisable(GL_LIGHTING);
  glLineWidth(2.0);

  // Draw the positive x, y, and z axis
  glColor3d(1.0,0.0,0.0);
  glBegin(GL_LINES);
  glVertex4d(0.0,0.0,0.0,1.0);
  glVertex4d(1.0,0.0,0.0,0.0);
  glEnd();
  glColor3d(0.0,1.0,0.0);
  glBegin(GL_LINES);
  glVertex4d(0.0,0.0,0.0,1.0);
  glVertex4d(0.0,1.0,0.0,0.0);
  glEnd();
  glColor3d(0.0,0.0,1.0);
  glBegin(GL_LINES);
  glVertex4d(0.0,0.0,0.0,1.0);
  glVertex4d(0.0,0.0,1.0,0.0);
  glEnd();

  // Draw the negative x, y, and z axis
  glLineStipple(1,0x00FF);
  glEnable(GL_LINE_STIPPLE);
  glColor3d(1.0,0.0,0.0);
  glBegin(GL_LINES);
  glVertex4d(-1.0,0.0,0.0,0.0);
  glVertex4d(0.0,0.0,0.0,1.0);
  glEnd();
  glColor3d(0.0,1.0,0.0);
  glBegin(GL_LINES);
  glVertex4d(0.0,-1.0,0.0,0.0);
  glVertex4d(0.0,0.0,0.0,1.0);
  glEnd();
  glColor3d(0.0,0.0,1.0);
  glBegin(GL_LINES);
  glVertex4d(0.0,0.0,-1.0,0.0);
  glVertex4d(0.0,0.0,0.0,1.0);
  glEnd();
  glDisable(GL_LINE_STIPPLE);

  glPopAttrib();
  glPopAttrib();
  glPopAttrib();
}

void GLWidget::paintXYRulers() const
{
  const int grid_width{ 20 };

  glPushAttrib(GL_COLOR);
  glPushAttrib(GL_LIGHTING);
  glPushAttrib(GL_LINE_WIDTH);

  glDisable(GL_LIGHTING);

  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int y = -grid_width; y <= grid_width; ++y )
  {
    if( y == 0 )
    {
      continue;
    }
    glVertex3d( -grid_width, y, 0.0 );
    glVertex3d(  grid_width, y, 0.0 );
  }
  glEnd();

  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( -grid_width, 0.0, 0.0 );
  glVertex3d(  grid_width, 0.0, 0.0 );
  glEnd();

  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int x = -grid_width; x <= grid_width; ++x )
  {
    if( x == 0 )
    {
      continue;
    }
    glVertex3d( x, -grid_width, 0.0 );
    glVertex3d( x,  grid_width, 0.0 );
  }
  glEnd();

  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( 0.0, -grid_width, 0.0 );
  glVertex3d( 0.0,  grid_width, 0.0 );
  glEnd();

  glPopAttrib();
  glPopAttrib();
  glPopAttrib();
}

void GLWidget::paintYZRulers() const
{
  const int grid_width{ 20 };

  glPushAttrib(GL_COLOR);
  glPushAttrib(GL_LIGHTING);
  glPushAttrib(GL_LINE_WIDTH);

  glDisable(GL_LIGHTING);

  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int y = -grid_width; y <= grid_width; ++y )
  {
    if( y == 0 )
    {
      continue;
    }
    glVertex3d( 0.0, y, -grid_width );
    glVertex3d( 0.0, y, grid_width );
  }
  glEnd();

  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( 0.0, 0.0, -grid_width );
  glVertex3d( 0.0, 0.0,  grid_width );
  glEnd();

  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int z = -grid_width; z <= grid_width; ++z )
  {
    if( z == 0 )
    {
      continue;
    }
    glVertex3d( 0.0, -grid_width, z );
    glVertex3d( 0.0,  grid_width, z );
  }
  glEnd();

  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( 0.0, -grid_width, 0.0 );
  glVertex3d( 0.0,  grid_width, 0.0 );
  glEnd();

  glPopAttrib();
  glPopAttrib();
  glPopAttrib();
}

void GLWidget::paintXZRulers() const
{
  const int grid_width{ 20 };

  glPushAttrib(GL_COLOR);
  glPushAttrib(GL_LIGHTING);
  glPushAttrib(GL_LINE_WIDTH);

  glDisable(GL_LIGHTING);

  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int x = -grid_width; x <= grid_width; ++x )
  {
    if( x == 0 )
    {
      continue;
    }
    glVertex3d( x, 0.0, -grid_width );
    glVertex3d( x, 0.0,  grid_width );
  }
  glEnd();

  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( 0.0, 0.0, -grid_width );
  glVertex3d( 0.0, 0.0,  grid_width );
  glEnd();

  glLineWidth(0.5);
  glColor3d(0.5,0.5,0.5);
  glBegin( GL_LINES );
  for( int z = -grid_width; z <= grid_width; ++z )
  {
    if( z == 0 )
    {
      continue;
    }
    glVertex3d( -grid_width, 0.0, z );
    glVertex3d(  grid_width, 0.0, z );
  }
  glEnd();

  glLineWidth(3.0);
  glColor3d(0.0,0.0,0.0);
  glBegin( GL_LINES );
  glVertex3d( -grid_width, 0.0, 0.0 );
  glVertex3d(  grid_width, 0.0, 0.0 );
  glEnd();

  glPopAttrib();
  glPopAttrib();
  glPopAttrib();
}

void GLWidget::getViewportDimensions( GLint& width, GLint& height ) const
{
  GLint viewport[4];
  glGetIntegerv(GL_VIEWPORT,viewport);
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

void GLWidget::toggleXYGrid()
{
  m_display_xy_grid = !m_display_xy_grid;
  updateGL();
}

void GLWidget::toggleYZGrid()
{
  m_display_yz_grid = !m_display_yz_grid;
  updateGL();
}

void GLWidget::toggleXZGrid()
{
  m_display_xz_grid = !m_display_xz_grid;
  updateGL();
}

void GLWidget::centerCamera()
{
  if( m_lock_camera )
  {
    return;
  }

  if( m_sim.empty() )
  {
    if( m_use_perspective_camera )
    {
      m_perspective_camera_controller.reset();
    }
    else
    {
      m_orthographic_controller.reset();
    }
    return;
  }

  scalar radius;
  Eigen::Matrix<GLdouble,3,1> center;
  m_sim.computeBoundingSphere( radius, center );

  if( m_use_perspective_camera )
  {
    m_perspective_camera_controller.centerCameraAtSphere( center, radius );
  }
  else
  {
    GLint viewport[4];
    glGetIntegerv( GL_VIEWPORT, viewport );
    const GLint width{ viewport[2] };
    const GLint height{ viewport[3] };

    const GLdouble ratio{ GLdouble( height ) / GLdouble( width ) };
    const GLdouble scale{ std::max( ratio * radius, radius ) };

    m_orthographic_controller.setCenterAndScale( center, scale );
  }

  updateGL();
}

void GLWidget::enablePerspectiveCamera()
{
  m_use_perspective_camera = true;
  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  const GLint width{ viewport[2] };
  const GLint height{ viewport[3] };
  m_perspective_camera_controller.setPerspective( width, height );
  centerCamera();
}

void GLWidget::enableOrthographicXYCamera()
{
  m_use_perspective_camera = false;
  m_orthographic_controller.useXYView();
  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  const GLint width{ viewport[2] };
  const GLint height{ viewport[3] };
  m_orthographic_controller.setPerspective( width, height );
  centerCamera();
}

void GLWidget::enableOrthographicZYCamera()
{
  m_use_perspective_camera = false;
  m_orthographic_controller.useZYView();
  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  const GLint width{ viewport[2] };
  const GLint height{ viewport[3] };
  m_orthographic_controller.setPerspective( width, height );
  centerCamera();
}

void GLWidget::enableOrthographicZXCamera()
{
  m_use_perspective_camera = false;
  m_orthographic_controller.useZXView();
  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  const GLint width{ viewport[2] };
  const GLint height{ viewport[3] };
  m_orthographic_controller.setPerspective( width, height );
  centerCamera();
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
      if( m_sim.state().nbodies() != 0 )
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

std::string projectionPlaneToString( const OrthographicCameraController::ProjectionPlane projection_plane )
{
  if( projection_plane == OrthographicCameraController::ProjectionPlane::XY )
  {
    return "xy";
  }
  else if( projection_plane == OrthographicCameraController::ProjectionPlane::ZX )
  {
    return "zx";
  }
  else if( projection_plane == OrthographicCameraController::ProjectionPlane::ZY )
  {
    return "zy";
  }
  std::cerr << "Impossible code path hit in projectionPlaneToString. This is a bug." << std::endl;
  std::exit( EXIT_FAILURE );
}

void GLWidget::exportCameraSettings()
{
  if( m_use_perspective_camera )
  {
    std::cout << "<camera_perspective theta=\"" << m_perspective_camera_controller.theta() << "\" phi=\"" << m_perspective_camera_controller.phi() << "\" rho=\"" << m_perspective_camera_controller.rho() << "\" lookat=\"" << m_perspective_camera_controller.lookat().x() << " " << m_perspective_camera_controller.lookat().y() << " " << m_perspective_camera_controller.lookat().z() << "\" up=\"" << m_perspective_camera_controller.up().x() << " " << m_perspective_camera_controller.up().y() << " " << m_perspective_camera_controller.up().z() << "\" fps=\"" << m_output_fps << "\" render_at_fps=\"" << m_render_at_fps << "\" locked=\"" << m_lock_camera << "\"/>" << std::endl;
  }
  else
  {
    std::cout << "<camera_orthographic projection_plane=\"" << projectionPlaneToString( m_orthographic_controller.projectionPlane() ) << "\" x=\"" << m_orthographic_controller.x().x() << " " << m_orthographic_controller.x().y() << " " << m_orthographic_controller.x().z() << "\" scale=\"" << m_orthographic_controller.scale() << "\" fps=\"" << m_output_fps << "\" render_at_fps=\"" << m_render_at_fps << "\" locked=\"" << m_lock_camera << "\"/>" << std::endl;
  }
}

void GLWidget::paintSphere( const RigidBodySphere& sphere, const Vector3s& color ) const
{
  const Eigen::Matrix<GLfloat,3,1> primary_color{ color.cast<GLfloat>() };
  const Eigen::Matrix<GLfloat,3,1> secondary_color{ 1.0, 1.0, 1.0 };
  glScaled( sphere.r(), sphere.r(), sphere.r() );
  assert( m_sphere_renderer != nullptr );
  m_sphere_renderer->drawVertexArray( primary_color, secondary_color );
}

void GLWidget::paintBox( const RigidBodyBox& box, const Vector3s& color ) const
{
  // Vertices of the box
//  Vector3s v0( box_ptr.wx(), box_ptr.wy(), box_ptr.wz()); v0 *= 0.5;
//  Vector3s v1( box_ptr.wx(), box_ptr.wy(),-box_ptr.wz()); v1 *= 0.5;
//  Vector3s v2(-box_ptr.wx(), box_ptr.wy(),-box_ptr.wz()); v2 *= 0.5;
//  Vector3s v3(-box_ptr.wx(), box_ptr.wy(), box_ptr.wz()); v3 *= 0.5;

//  Vector3s v4( box_ptr.wx(),-box_ptr.wy(), box_ptr.wz()); v4 *= 0.5;
//  Vector3s v5( box_ptr.wx(),-box_ptr.wy(),-box_ptr.wz()); v5 *= 0.5;
//  Vector3s v6(-box_ptr.wx(),-box_ptr.wy(),-box_ptr.wz()); v6 *= 0.5;
//  Vector3s v7(-box_ptr.wx(),-box_ptr.wy(), box_ptr.wz()); v7 *= 0.5;

//  GLfloat mcolorambient[] = { 0.8*m_scene->getBodyColor(bdy_idx,0), 0.8*m_scene->getBodyColor(bdy_idx,1), 0.8*m_scene->getBodyColor(bdy_idx,2), 1.0f };
//  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
//  GLfloat mcolordiffuse[] = { 0.9*m_scene->getBodyColor(bdy_idx,0), 0.9*m_scene->getBodyColor(bdy_idx,1), 0.9*m_scene->getBodyColor(bdy_idx,2), 1.0f };
//  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );

  GLfloat mcolorambient[] = { GLfloat( 0.8 * color.x() ), GLfloat( 0.8 * color.y() ), GLfloat( 0.8 * color.z() ), GLfloat( 1.0 ) };
  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
  GLfloat mcolordiffuse[] = { GLfloat( 0.9 * color.x() ), GLfloat( 0.9 * color.y() ), GLfloat( 0.9 * color.z() ), GLfloat( 1.0 ) };
  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );

  glScaled( box.halfWidths().x(), box.halfWidths().y(), box.halfWidths().z() );

  glBegin(GL_QUADS);
    // Top of the box
    glNormal3d((GLdouble)0.0,(GLdouble)1.0,(GLdouble)0.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)1.0);

    // Bottom of the box
    glNormal3d((GLdouble)0.0,(GLdouble)-1.0,(GLdouble)0.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)1.0);

    // Positive x side of the box
    glNormal3d((GLdouble)1.0,(GLdouble)0.0,(GLdouble)0.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)1.0);

    // Negative x side of the box
    glNormal3d((GLdouble)-1.0,(GLdouble)0.0,(GLdouble)0.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)1.0);

    // Positive z side of the box
    glNormal3d((GLdouble)0.0,(GLdouble)0.0,(GLdouble)1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)1.0);

    // Negative z side of the box
    glNormal3d((GLdouble)0.0,(GLdouble)0.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)-1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)1.0,(GLdouble)1.0,(GLdouble)-1.0);
    glVertex3d((GLdouble)-1.0,(GLdouble)1.0,(GLdouble)-1.0);
  glEnd();
}

void GLWidget::paintTriangleMesh( const RigidBodyTriangleMesh& mesh, const Vector3s& color ) const
{
  GLfloat mcolorambient[] = { (GLfloat) (0.8 * color.x()), (GLfloat) (0.8 * color.y()), (GLfloat) (0.8 * color.z()), (GLfloat) 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
  GLfloat mcolordiffuse[] = { (GLfloat) (0.9 * color.x()), (GLfloat) (0.9 * color.y()), (GLfloat) (0.9 * color.z()), (GLfloat) 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );

  const Matrix3Xsc& vertices{ mesh.vertices() };
  const Matrix3Xuc& faces{ mesh.faces() };

  glBegin( GL_TRIANGLES );
  for( int i = 0; i < faces.cols(); ++i )
  {
    // Compute a normal for this face
    const Vector3s n{ ( vertices.col( faces(1,i) ) - vertices.col( faces(0,i) ) ).cross( vertices.col( faces(2,i) ) - vertices.col( faces(0,i) ) ).normalized() };
    // Send the normal to OpenGL
    glNormal3f( (GLfloat) n.x(), (GLfloat) n.y(), (GLfloat) n.z() );
    // Send the vertices to OpenGL
    glVertex3f( (GLfloat) vertices( 0, faces(0,i) ), (GLfloat) vertices( 1, faces(0,i) ), (GLfloat) vertices( 2, faces(0,i) ) );
    glVertex3f( (GLfloat) vertices( 0, faces(1,i) ), (GLfloat) vertices( 1, faces(1,i) ), (GLfloat) vertices( 2, faces(1,i) ) );
    glVertex3f( (GLfloat) vertices( 0, faces(2,i) ), (GLfloat) vertices( 1, faces(2,i) ), (GLfloat) vertices( 2, faces(2,i) ) );
  }
  glEnd();
}

void GLWidget::paintBody( const int geo_idx, const RigidBodyGeometry& geometry, const Vector3s& color ) const
{
  if( geometry.getType() == RigidBodyGeometryType::BOX )
  {
    const RigidBodyBox& box_geom{ sd_cast<const RigidBodyBox&>( geometry ) };
    paintBox( box_geom, color );
  }
  else if( geometry.getType() == RigidBodyGeometryType::SPHERE )
  {
    const RigidBodySphere& sphere_geom{ sd_cast<const RigidBodySphere&>( geometry ) };
    paintSphere( sphere_geom, color );
  }
  else if( geometry.getType() == RigidBodyGeometryType::STAPLE )
  {
    assert( geo_idx >= 0 );
    assert( geo_idx < int( m_body_renderers.size() ) );
    assert( m_body_renderers[geo_idx] != nullptr );
    m_body_renderers[geo_idx]->renderBody( color.cast<GLfloat>() );
  }
  else if( geometry.getType() == RigidBodyGeometryType::TRIANGLE_MESH )
  {
    const RigidBodyTriangleMesh& triangle_geom{ sd_cast<const RigidBodyTriangleMesh&>( geometry ) };
    paintTriangleMesh( triangle_geom, color );
  }
  else
  {
    std::cerr << "Invalid geometry type encountered in GLWidget::paintBody. This is bug. Exiting." << std::endl;
    std::exit( EXIT_FAILURE );
  }
}

void GLWidget::paintSystem() const
{
  const RigidBody3DState& state{ m_sim.getState() };

  // Draw each body
  assert( state.q().size() == 12 * state.nbodies() );
  for( unsigned i = 0; i < state.nbodies(); ++i )
  {
    glPushMatrix();
    // Translate to center of mass position
    glTranslated( state.q()( 3 * i + 0 ), state.q()( 3 * i + 1 ), state.q()( 3 * i + 2 ) );
    // Rotate about center of mass
    Eigen::Matrix<GLdouble,4,4,Eigen::ColMajor> gl_rot_mat;
    gl_rot_mat.setZero();
    gl_rot_mat.block<3,3>(0,0) = Eigen::Map<const Matrix33sr>{ state.q().segment<9>( 3 * state.nbodies() + 9 * i ).data() };
    gl_rot_mat(3,3) = 1.0;
    glMultMatrixd( gl_rot_mat.data() );
    // Draw the geometry
    paintBody( state.getGeometryIndexOfBody( i ), state.getGeometryOfBody( i ), m_body_colors.segment<3>( 3 * i ) );
    glPopMatrix();
  }

  // Draw any static planes
  for( const StaticPlaneRenderer& plane_renderer : m_plane_renderers )
  {
    assert( plane_renderer.idx() < m_sim.state().staticPlanes().size() );
    plane_renderer.draw( m_sim.state().staticPlanes()[ plane_renderer.idx() ] );
  }
  // Draw any static cylinders
  for( const StaticCylinderRenderer& cylinder_renderer : m_cylinder_renderers )
  {
    assert( cylinder_renderer.idx() >= 0 );
    assert( cylinder_renderer.idx() < int( m_sim.state().staticCylinders().size() ) );
    cylinder_renderer.draw( m_sim.state().staticCylinders()[ cylinder_renderer.idx() ] );
  }
  // Draw any planar portals
  glPushAttrib( GL_COLOR );
  glPushAttrib( GL_LINE_WIDTH );
  glPushAttrib( GL_LIGHTING );
  glDisable( GL_LIGHTING );
  glLineWidth( 2.0 );
  {
    std::mt19937_64 mt{ 123456 };
    std::uniform_int_distribution<int> color_gen{ 0, 255 };
    for( const PlanarPortalRenderer& portal_renderer : m_portal_renderers )
    {
      const int r{ color_gen( mt ) };
      const int g{ color_gen( mt ) };
      const int b{ color_gen( mt ) };
      qglColor( QColor{ r, g, b } );
      assert( portal_renderer.idx() < m_sim.state().numPlanarPortals() );
      portal_renderer.draw( m_sim.state().planarPortal( portal_renderer.idx() ) );
    }
  }
  glPopAttrib();
  glPopAttrib();
  glPopAttrib();

  assert( checkGLErrors() );
}

QString generateTimeString( const unsigned iteration, const Rational<std::intmax_t>& dt, const int display_precision, const scalar& end_time )
{
  QString time_string{ QObject::tr( " t: " ) };
  time_string += QString::number( iteration * scalar( dt ), 'f', display_precision );
  if( end_time != SCALAR_INFINITY )
  {
    time_string += QString{ QObject::tr( " / " ) };
    time_string += QString::number( end_time );
  }
  return time_string;
}

QString generateHString( const scalar& delta_H0 )
{
  return QString{ QObject::tr( "dH: " ) } + QString::number( delta_H0 );
}

QString generatePString( const Vector3s& delta_p0 )
{
  QString pstring{ QObject::tr( "dp: " ) };
  pstring += QString::number( delta_p0.x() ) + " ";
  pstring += QString::number( delta_p0.y() ) + " ";
  pstring += QString::number( delta_p0.z() );
  return pstring;
}

QString generateLString( const Vector3s& delta_L0 )
{
  QString Lstring{ QObject::tr( "dL: " ) };
  Lstring += QString::number( delta_L0.x() ) + " ";
  Lstring += QString::number( delta_L0.y() ) + " ";
  Lstring += QString::number( delta_L0.z() );
  return Lstring;
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
  glPushAttrib( GL_LIGHTING );
  glDisable( GL_LIGHTING );
  glPushAttrib( GL_BLEND );
  glEnable( GL_BLEND );
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  // Draw a semi-transparent overlay so text is visible regardless of background color
  const Eigen::Matrix<GLdouble,2,1> overlay_start{ 0, height - 4 * 12 - 2 };
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
  glPopAttrib();

  glMatrixMode( GL_MODELVIEW );
  glPopMatrix();
  glMatrixMode( GL_PROJECTION );
  glPopMatrix();
  glPopAttrib();

  // String to display in upper left corner
  const QString time_string{ generateTimeString( m_iteration, m_dt, m_display_precision, m_end_time ) };
  const QString delta_H{ generateHString( m_delta_H0 ) };
  const QString delta_p{ generatePString( m_delta_p0 ) };
  const QString delta_L{ generateLString( m_delta_L0 ) };
  {
    const QFontMetrics font_metrics{ QFont{ "Courier", 12 } };
    text_width = std::max( text_width, font_metrics.boundingRect( time_string ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_H ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_p ).width() );
    text_width = std::max( text_width, font_metrics.boundingRect( delta_L ).width() );
  }

  qglColor( QColor{ 255, 255, 255 } );
  const QFont font{ "Courier", 12 };
  renderText( 2, font.pointSize(), time_string, font );
  renderText( 2, 2 * font.pointSize(), delta_H, font );
  renderText( 2, 3 * font.pointSize(), delta_p, font );
  renderText( 2, 4 * font.pointSize(), delta_L, font );

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

  if( m_use_perspective_camera )
  {
    if( event->buttons() & Qt::LeftButton )
    {
      const GLdouble horizontal_amount{ 0.01 * GLdouble( dx ) };
      const GLdouble vertical_amount{ 0.01 * GLdouble( dy ) };
      m_perspective_camera_controller.addToZenithAngle( -vertical_amount );
      m_perspective_camera_controller.addToAzimuthAngle( -horizontal_amount );
      updateGL();
    }
    else if( event->buttons() & Qt::MidButton )
    {
      const GLdouble horizontal_amount{ 0.002 * GLdouble( dx ) };
      const GLdouble vertical_amount{ 0.002 * GLdouble( dy ) };
      m_perspective_camera_controller.trackCameraHorizontal( -m_perspective_camera_controller.getDistFromCenter() * horizontal_amount );
      m_perspective_camera_controller.trackCameraVertical( m_perspective_camera_controller.getDistFromCenter() * vertical_amount  );
      updateGL();
    }
    else if( event->buttons() & Qt::RightButton )
    {
      const GLdouble horizontal_amount{ 0.01 * GLdouble( dx ) };
      const GLdouble vertical_amount{ 0.01 * GLdouble( dy ) };
      m_perspective_camera_controller.addToDistFromCenter( m_perspective_camera_controller.getDistFromCenter() * ( horizontal_amount - vertical_amount ) );
      updateGL();
    }
  }
  else
  {
    if( event->buttons() & Qt::LeftButton )
    {
      m_orthographic_controller.translateView( dx, dy );
      updateGL();
    }
    else if( event->buttons() & Qt::RightButton )
    {
      const GLdouble horizontal_amount{ GLdouble( dx ) };
      const GLdouble vertical_amount{ GLdouble( dy ) };
      m_orthographic_controller.addToDistFromCenter( 0.1 * ( horizontal_amount - vertical_amount ) );
      updateGL();
    }
  }

  assert( checkGLErrors() );
}

void GLWidget::wheelEvent( QWheelEvent* event )
{
  if( m_lock_camera )
  {
    return;
  }

  if( m_use_perspective_camera )
  {
    m_perspective_camera_controller.addToDistFromCenter( -0.002 * m_perspective_camera_controller.getDistFromCenter() * event->delta() );
  }
  else
  {
    m_orthographic_controller.addToDistFromCenter( -0.05 * event->delta() );
  }

  updateGL();

  assert( checkGLErrors() );
}
