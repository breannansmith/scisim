#include "OrthographicCameraController.h"

#include "SCISim/Math/MathDefines.h"

OrthographicCameraController::OrthographicCameraController()
: m_theta_cam()
, m_phi_cam()
, m_cam_lookat( Eigen::Matrix<GLdouble,3,1>::Zero() )
, m_cam_psn( Eigen::Matrix<GLdouble,3,1>::Zero() )
, m_up_orientation( Eigen::Quaternion<GLdouble>::Identity() )
, m_scale_factor( 15.0 )
, m_projection_plane( ProjectionPlane::XY )
{
  useZXView();
}

void OrthographicCameraController::setCamera( const ProjectionPlane& projection_plane, const Eigen::Matrix<GLdouble,3,1>& lookat, const GLdouble& scale )
{
  if( projection_plane == ProjectionPlane::XY )
  {
    m_projection_plane = ProjectionPlane::XY;
    useXYView();
  }
  else if( projection_plane == ProjectionPlane::ZX )
  {
    m_projection_plane = ProjectionPlane::ZX;
    useZXView();
  }
  else if( projection_plane == ProjectionPlane::ZY )
  {
    m_projection_plane = ProjectionPlane::ZY;
    useZYView();
  }

  m_scale_factor = std::max( 0.00001, scale );
  m_cam_lookat.setZero();
  m_cam_lookat = lookat;

  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  const GLint width{ viewport[2] };
  const GLint height{ viewport[3] };
  setPerspective( width, height );
}

void OrthographicCameraController::useXYView()
{
  m_projection_plane = ProjectionPlane::XY;
  m_theta_cam = MathDefines::PI<GLdouble>() / 2.0;
  m_phi_cam = 0.0;
  updatePositionAndFrame();
}

void OrthographicCameraController::useZYView()
{
  m_projection_plane = ProjectionPlane::ZY;
  m_theta_cam = MathDefines::PI<GLdouble>() / 2.0;
  m_phi_cam = - MathDefines::PI<GLdouble>() / 2.0;
  updatePositionAndFrame();
}

void OrthographicCameraController::useZXView()
{
  m_projection_plane = ProjectionPlane::ZX;
  m_theta_cam = 0.0;
  m_phi_cam = - MathDefines::PI<GLdouble>() / 2.0;
  updatePositionAndFrame();
}

void OrthographicCameraController::positionCamera()
{
  {
    Eigen::Matrix<GLdouble,4,4> rotation{ Eigen::Matrix<GLdouble,4,4>::Identity() };
    rotation.block<1,3>(0,0) = m_cam_y;
    rotation.block<1,3>(1,0) = m_cam_x;
    rotation.block<1,3>(2,0) = -m_cam_z;

    assert( ( rotation.block<3,3>(0,0) * rotation.block<3,3>(0,0).transpose() - Eigen::Matrix<GLdouble,3,3>::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );

    glMultMatrixd( rotation.data() );
  }

  {
    const Eigen::Matrix<GLdouble,3,1> camera_psn{ m_cam_lookat + m_cam_psn };
    glTranslated( -camera_psn.x(), -camera_psn.y(), -camera_psn.z() );
  }
}

void OrthographicCameraController::setPerspective( const int width, const int height )
{
  glViewport( 0, 0, width, height );

  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();

  // Set the coordinate system to achieve the desired zoom level, center
  const GLdouble ratio{ GLdouble( height ) / GLdouble( width ) };

  glOrtho( - m_scale_factor / ratio, m_scale_factor / ratio, - m_scale_factor, m_scale_factor, - 100.0, 100.0 );
}

void OrthographicCameraController::translateView( const GLdouble& dx, const GLdouble& dy )
{
  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  const GLint width{ viewport[2] };
  const GLint height{ viewport[3] };

  const GLdouble percent_x{ dx / GLdouble( width ) };
  const GLdouble percent_y{ dy / GLdouble( height ) };
  const GLdouble translate_x{ percent_x * 2.0 * m_scale_factor * GLdouble( width ) / GLdouble( height ) };
  const GLdouble translate_y{ percent_y * 2.0 * m_scale_factor };
  m_cam_lookat += translate_y * m_cam_x;
  m_cam_lookat -= translate_x * m_cam_y;
}

void OrthographicCameraController::addToDistFromCenter( const GLdouble& dst_amnt )
{
  m_scale_factor = std::max( 0.00001, m_scale_factor + dst_amnt );

  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  const GLint width{ viewport[2] };
  const GLint height{ viewport[3] };
  setPerspective( width, height );
}

void OrthographicCameraController::setCenterAndScale( const Eigen::Matrix<GLdouble,3,1>& center, const GLdouble& scale )
{
  m_cam_lookat = center;
  m_scale_factor = std::max( 0.00001, scale );

  GLint viewport[4];
  glGetIntegerv( GL_VIEWPORT, viewport );
  const GLint width{ viewport[2] };
  const GLint height{ viewport[3] };
  setPerspective( width, height );
}

void OrthographicCameraController::updatePositionAndFrame()
{
  assert( fabs( m_up_orientation.norm() - 1.0 ) < 1.0e-6 );

  m_cam_psn = m_up_orientation * Eigen::Matrix<GLdouble,3,1>( sin( m_theta_cam ) * sin( m_phi_cam ), cos( m_theta_cam ), sin( m_theta_cam ) * cos( m_phi_cam ) );
  m_cam_z = -1.0 * m_cam_psn;
  m_cam_psn *= 50.0; // m_rho_cam
  m_cam_y = m_up_orientation * Eigen::Matrix<GLdouble,3,1>{ cos( m_phi_cam ), 0.0, -sin( m_phi_cam ) };
  m_cam_x = m_cam_y.cross( m_cam_z );

  assert( fabs( m_cam_x.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_cam_y.norm() - 1.0 ) <= 1.0e-6 );
  assert( fabs( m_cam_z.norm() - 1.0 ) <= 1.0e-6 );
}

OrthographicCameraController::ProjectionPlane OrthographicCameraController::projectionPlane() const
{
  return m_projection_plane;
}

const GLdouble& OrthographicCameraController::scale() const
{
  return m_scale_factor;
}

const Eigen::Matrix<GLdouble,3,1>& OrthographicCameraController::x() const
{
  return m_cam_lookat;
}

void OrthographicCameraController::reset()
{
  *this = OrthographicCameraController();
}
