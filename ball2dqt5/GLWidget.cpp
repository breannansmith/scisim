#include "GLWidget.h"

#include <QFontDatabase>
#include <QMatrix4x4>
#include <QOpenGLFunctions_3_3_Core>
#include <QPainter>
#include <QWheelEvent>

#include <cassert>
#include <iomanip>

#include "ball2d/Ball2DState.h"

GLWidget::GLWidget( QWidget* parent, const QSurfaceFormat& format )
: QOpenGLWidget( parent )
, m_f( nullptr )
, m_axis_shader()
, m_circle_shader()
, m_plane_shader()
, m_annulus_shader()
, m_rectangle_shader()
, m_w( 1280 )
, m_h( 720 )
, m_display_scale( 1.0 )
, m_center_x( 0.0 )
, m_center_y( 0.0 )
, m_lock_camera( false )
, m_last_pos()
, m_left_mouse_button_pressed( false )
, m_right_mouse_button_pressed( false )
, m_display_precision( 0 )
, m_display_HUD( true )
, m_time( 0.0 )
, m_end_time( std::numeric_limits<scalar>::max() )
, m_delta_H( 0.0 )
, m_delta_p( Vector2s::Zero() )
, m_delta_L( 0.0 )
, m_num_circle_subdivs( 32 )
, m_num_drum_subdivs( 32 )
, m_num_aa_samples( format.samples() )
, m_bg_color( 0.098f, 0.098f, 0.098f )
, m_hud_text_color( 230, 230, 230 )
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
    m_annulus_shader.cleanup();
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

void GLWidget::initialize( const bool& render_on_load, const RenderSettings& render_settings, const int dt_display_precision,
                           const Ball2DState& state, const VectorXs& body_colors, const std::vector<PlaneRenderSettings>& plane_settings,
                           const std::vector<DrumRenderSettings>& drum_settings, const std::vector<PortalRenderSettings>& portal_settings,
                           const scalar& end_time, const bool empty, const Vector4s& bbox )
{
  if( render_settings.camera_set )
  {
    m_lock_camera = render_settings.lock_camera;
  }

  // Trivially there is no change in energy, momentum, and angular momentum until we take a timestep
  m_delta_H = 0.0;
  m_delta_p = Vector2s::Zero();
  m_delta_L = 0.0;

  // Compute the number of characters after the decimal point in the timestep string
  m_display_precision = dt_display_precision;

  const bool lock_backup{ m_lock_camera };
  m_lock_camera = false;

  if( !render_settings.camera_set )
  {
    centerCamera( false, empty, bbox );
  }
  else
  {
    m_center_x = float(render_settings.camera_center.x());
    m_center_y = float(render_settings.camera_center.y());
    m_display_scale = float(render_settings.camera_scale_factor);
  }

  m_lock_camera = lock_backup;

  m_num_circle_subdivs = render_settings.num_ball_subdivs;
  m_num_drum_subdivs = render_settings.num_drum_subdivs;

  if( m_f != nullptr && render_on_load )
  {
    // Update the global render settings
    m_circle_shader.cleanup();
    m_circle_shader.initialize( m_num_circle_subdivs, m_f );

    m_annulus_shader.cleanup();
    m_annulus_shader.initialize( m_num_drum_subdivs, m_f );

    // Draw the scene
    resizeGL( m_w, m_h );
    update();
  }

  copyRenderState( state, body_colors, plane_settings, drum_settings, portal_settings, 0.0, end_time, 0.0, Vector2s::Zero(), 0.0 );
}

void GLWidget::copyRenderState( const Ball2DState& state, const VectorXs& body_colors, const std::vector<PlaneRenderSettings>& plane_settings,
                                const std::vector<DrumRenderSettings>& drum_settings, const std::vector<PortalRenderSettings>& portal_settings,
                                const scalar& time, const scalar& end_time, const scalar& delta_H, const Vector2s& delta_p, const scalar& delta_L )
{
  {
    const VectorXs& q{ state.q() };
    const VectorXs& r{ state.r() };

    // Find teleported versions of each ball
    std::vector<Vector2s> teleported_centers;
    std::vector<int> teleported_indices;
    // For each periodic boundary
    const std::vector<PlanarPortal>& planar_portals{ state.planarPortals() };
    for( const PlanarPortal& planar_portal : planar_portals )
    {
      // For each ball
      for( int ball_idx = 0; ball_idx < r.size(); ball_idx++ )
      {
          const Vector2s pos{ q.segment<2>( 2 * ball_idx ) };
          const scalar rad{ r( ball_idx ) };
          // If the current ball intersect a periodic boudnary
          bool intersecting_index;
          if( planar_portal.ballTouchesPortal( pos, rad, intersecting_index ) )
          {
            Vector2s teleported_pos;
            planar_portal.teleportBall( pos, rad, teleported_pos );
            teleported_centers.emplace_back( teleported_pos );
            teleported_indices.emplace_back( ball_idx );
          }
      }
    }

    Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& cd{ m_circle_shader.circleData() };
    cd.resize( 6 * ( state.nballs() + teleported_centers.size() ) );

    // Copy over the non-teleported balls
    for( unsigned ball_idx = 0; ball_idx < state.nballs(); ball_idx++ )
    {
      // Center of mass, radius, and color
      cd.segment<2>( 6 * ball_idx ) = q.segment<2>( 2 * ball_idx ).cast<GLfloat>();
      cd( 6 * ball_idx + 2 ) = GLfloat( r( ball_idx ) );
      cd.segment<3>( 6 * ball_idx + 3 ) = body_colors.segment<3>( 3 * ball_idx ).cast<GLfloat>();
    }

    // Copy over the teleported balls
    for( int tlprtd_idx = 0; tlprtd_idx < int(teleported_centers.size()); tlprtd_idx++ )
    {
      // Center of mass, radius, and color
      cd.segment<2>( 6 * state.nballs() + 6 * tlprtd_idx ) = teleported_centers[tlprtd_idx].cast<GLfloat>();
      cd( 6 * state.nballs() + 6 * tlprtd_idx + 2 ) = GLfloat( r( teleported_indices[tlprtd_idx] ) );
      cd.segment<3>( 6 * state.nballs() + 6 * tlprtd_idx + 3 ) = body_colors.segment<3>( 3 * teleported_indices[tlprtd_idx] ).cast<GLfloat>();
    }
  }

  // Planes
  {
    Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& plane_data{ m_plane_shader.planeData() };
    plane_data.resize( 6 * plane_settings.size() );
    for( int renderer_num = 0; renderer_num < int(plane_settings.size()); renderer_num++ )
    {
      const int plane_idx = plane_settings[renderer_num].idx;
      const StaticPlane& plane = state.staticPlanes()[plane_idx];

      plane_data.segment<2>( 6 * plane_idx ) = plane.x().cast<GLfloat>();
      plane_data.segment<2>( 6 * plane_idx + 2 ) = plane.n().cast<GLfloat>();
      plane_data( 6 * plane_idx + 4 ) = plane_settings[renderer_num].r(0);
      plane_data( 6 * plane_idx + 5 ) = plane_settings[renderer_num].r(1);
    }
  }

  // Annuli
  {
    Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& annulus_data{ m_annulus_shader.annulusData() };
    annulus_data.resize( 4 * state.staticDrums().size() );
    for( int renderer_num = 0; renderer_num < int(drum_settings.size()); renderer_num++ )
    {
      const int drum_idx = drum_settings[renderer_num].idx;
      const StaticDrum& drum = state.staticDrums()[drum_idx];

      annulus_data.segment<2>( 4 * drum_idx ) = drum.x().cast<GLfloat>();
      annulus_data( 4 * drum_idx + 2 ) = GLfloat(drum.r());
      annulus_data( 4 * drum_idx + 3 ) = GLfloat(drum.r()) + drum_settings[renderer_num].r;
    }
  }

  // Portals
  {
    const std::vector<PlanarPortal>& planar_portals{ state.planarPortals() };

    Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& rectangle_data{ m_rectangle_shader.data() };
    rectangle_data.resize( 32 * portal_settings.size() );

    for( int render_num = 0; render_num < int(portal_settings.size()); render_num++ )
    {
      const int portal_idx = portal_settings[render_num].idx;
      const float& r0 = portal_settings[render_num].thickness;
      const float& r1 = portal_settings[render_num].half_width;
      const float& iw = portal_settings[render_num].indicator_half_width;
      const Eigen::Vector3f& portal_color = portal_settings[render_num].color;

      const float thetaA = float(std::atan2(planar_portals[portal_idx].planeA().n().y(), planar_portals[portal_idx].planeA().n().x()));
      const float thetaB = float(std::atan2(planar_portals[portal_idx].planeB().n().y(), planar_portals[portal_idx].planeB().n().x()));

      // Top portal
      // Center of mass
      rectangle_data(32 * render_num + 0) = float(planar_portals[portal_idx].planeA().x().x())
                                            - r0 * float(planar_portals[portal_idx].planeA().n().x());
      rectangle_data(32 * render_num + 1) = float(planar_portals[portal_idx].planeA().x().y())
                                            - r0 * float(planar_portals[portal_idx].planeA().n().y());
      // Orientation
      rectangle_data(32 * render_num + 2) = thetaA;
      // Radii
      rectangle_data(32 * render_num + 3) = r0;
      rectangle_data(32 * render_num + 4) = r1;
      // Color
      rectangle_data(32 * render_num + 5) = portal_color.x();
      rectangle_data(32 * render_num + 6) = portal_color.y();
      rectangle_data(32 * render_num + 7) = portal_color.z();

      // Top portal center
      // Center of mass
      rectangle_data(32 * render_num + 8) = float(planar_portals[portal_idx].transformedAx().x())
                                            - (2.0f * r0 + iw) * float(planar_portals[portal_idx].planeA().n().x());
      rectangle_data(32 * render_num + 9) = float(planar_portals[portal_idx].transformedAx().y())
                                            - (2.0f * r0 + iw) * float(planar_portals[portal_idx].planeA().n().y());
      // Orientation
      rectangle_data(32 * render_num + 10) = thetaA;
      // Radii
      rectangle_data(32 * render_num + 11) = iw;
      rectangle_data(32 * render_num + 12) = r0;
      // Color
      rectangle_data(32 * render_num + 13) = portal_color.x();
      rectangle_data(32 * render_num + 14) = portal_color.y();
      rectangle_data(32 * render_num + 15) = portal_color.z();

      // Bottom portal
      // Center of mass
      rectangle_data(32 * render_num + 16) = float(planar_portals[portal_idx].planeB().x().x())
                                             - r0 * float(planar_portals[portal_idx].planeB().n().x());
      rectangle_data(32 * render_num + 17) = float(planar_portals[portal_idx].planeB().x().y())
                                             - r0 * float(planar_portals[portal_idx].planeB().n().y());
      // Orientation
      rectangle_data(32 * render_num + 18) = thetaB;
      // Radii
      rectangle_data(32 * render_num + 19) = r0;
      rectangle_data(32 * render_num + 20) = r1;
      // Color
      rectangle_data(32 * render_num + 21) = portal_color.x();
      rectangle_data(32 * render_num + 22) = portal_color.y();
      rectangle_data(32 * render_num + 23) = portal_color.z();

      // Bottom portal center
      // Center of mass
      rectangle_data(32 * render_num + 24) = float(planar_portals[portal_idx].transformedBx().x())
                                             - (2.0f * r0 + iw) * float(planar_portals[portal_idx].planeB().n().x());
      rectangle_data(32 * render_num + 25) = float(planar_portals[portal_idx].transformedBx().y())
                                             - (2.0f * r0 + iw) * float(planar_portals[portal_idx].planeB().n().y());
      // Orientation
      rectangle_data(32 * render_num + 26) = thetaB;
      // Radii
      rectangle_data(32 * render_num + 27) = iw;
      rectangle_data(32 * render_num + 28) = r0;
      // Color
      rectangle_data(32 * render_num + 29) = portal_color.x();
      rectangle_data(32 * render_num + 30) = portal_color.y();
      rectangle_data(32 * render_num + 31) = portal_color.z();
    }
  }

  m_time = time;
  m_end_time = end_time;

  m_delta_H = delta_H;
  m_delta_p = delta_p;
  m_delta_L = delta_L;
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

  m_axis_shader.initialize( m_f );
  m_circle_shader.initialize( m_num_circle_subdivs, m_f );
  m_plane_shader.initialize( m_f );
  m_annulus_shader.initialize( m_num_drum_subdivs, m_f );
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
  m_annulus_shader.setTransform( pv );
  m_rectangle_shader.setTransform( pv );

  pv.scale( m_display_scale, m_display_scale );
  m_axis_shader.setTransform( pv );
}

void GLWidget::paintGL()
{
  assert( m_f != nullptr );

  m_f->glClearColor( m_bg_color(0), m_bg_color(1), m_bg_color(2), 1.0 );

  m_f->glClear( GL_COLOR_BUFFER_BIT );

  if( m_left_mouse_button_pressed )
  {
    m_axis_shader.draw();
  }

  m_plane_shader.draw();
  m_annulus_shader.draw();
  m_rectangle_shader.draw();
  m_circle_shader.draw();

  if( m_display_HUD )
  {
    paintHUD();
  }

  assert( checkGLErrors() );
}

void GLWidget::lockCamera( const bool lock_camera )
{
  m_lock_camera = lock_camera;
}

void GLWidget::setBackgroundColor( const Eigen::Matrix<GLfloat, 3, 1>& background_color )
{
  m_bg_color = background_color;
}

void GLWidget::toggleHUD()
{
  m_display_HUD = !m_display_HUD;
  update();
}

void GLWidget::centerCamera( const bool update_gl, const bool empty, const Vector4s& bbox )
{
  if( m_lock_camera )
  {
    return;
  }

  if( empty )
  {
    m_display_scale = 1.0;
    m_center_x = 0.0;
    m_center_y = 0.0;
    return;
  }

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
     << m_time << " to " << file_name.toStdString();
  qInfo( "%s", ss.str().c_str() );
  const QImage frame_buffer{ grabFramebuffer() };
  frame_buffer.save( file_name );
}

std::string GLWidget::exportCameraSettings( const int output_fps, const bool render_at_fps )
{
  std::stringstream ss;
  ss << "<camera cx=\"" << m_center_x << "\" cy=\"" << m_center_y << "\" scale_factor=\"" << m_display_scale
     << "\" fps=\"" << output_fps << "\" render_at_fps=\"" << render_at_fps << "\" locked=\"" << m_lock_camera << "\"/>";
  return ss.str();
}

static QString generateTimeString( const scalar& time, const int display_precision, const scalar& end_time )
{
  QString time_string{ QObject::tr( "  t: " ) };
  time_string += QString::number( time, 'f', display_precision );
  if( end_time != std::numeric_limits<scalar>::max() )
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
  const QString time_string{ generateTimeString( m_time, m_display_precision, m_end_time ) };
  const QString delta_H{ generateNumericString( " dH: ", m_delta_H ) };
  const QString delta_px{ generateNumericString( "dpx: ", m_delta_p.x() ) };
  const QString delta_py{ generateNumericString( "dpy: ", m_delta_p.y() ) };
  const QString delta_L{ generateNumericString( " dL: ", m_delta_L ) };

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

  // const int xextent{ text_width + 2 + 4 };
  // constexpr int yextent{ 5 * 12 + 4 };

  {
    QPainter painter{ this };
    // painter.setPen( QColor{ 0, 0, 0, 125 } );
    // {
    //   const QRect rect{ 0, 0, xextent, yextent };
    //   painter.fillRect( rect, QBrush{ QColor{ 0, 0, 0, 128 } } );
    // }
    painter.setPen( QColor{ m_hud_text_color(0), m_hud_text_color(1), m_hud_text_color(2) } );
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
  // NB: If locked, still need to allow for mouse release events to disable the axis display
  // if( m_lock_camera )
  // {
  //   return;
  // }

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
