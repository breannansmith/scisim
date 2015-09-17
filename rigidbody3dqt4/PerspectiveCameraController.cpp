#include "PerspectiveCameraController.h"

#include "SCISim/Math/MathDefines.h"

#include <cassert>

PerspectiveCameraController::PerspectiveCameraController()
: m_fovy( 60.0 )
, m_aspect( 1.0 )
, m_theta_cam( MathDefines::PI<GLdouble>() / 3.0 )
, m_phi_cam( MathDefines::PI<GLdouble>() / 4.0 )
, m_rho_cam( 3.0 )
, m_cam_lookat( Eigen::Matrix<GLdouble,3,1>::Zero() )
, m_cam_psn( Eigen::Matrix<GLdouble,3,1>::Zero() )
, m_up_orientation( Eigen::Quaternion<GLdouble>::Identity() )
{
  updatePositionAndFrame();
}

void PerspectiveCameraController::positionCamera()
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

void PerspectiveCameraController::setPerspective( const int width, const int height )
{
  const GLdouble fovy{ 54.43 };
  const GLdouble aspect{ GLdouble( width ) / GLdouble( height ) };
  const GLdouble z_near{ 0.1 };
  const GLdouble z_far{ 1000.0 };

  glViewport( 0, 0, width, height );

  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();

  // TODO: Off by a factor of 2?
  const GLdouble top{ tan( fovy / 360.0 * MathDefines::PI<GLdouble>() ) * z_near };
  const GLdouble right{ top * aspect };
  glFrustum( -right, right, -top, top, z_near, z_far );
  m_fovy = fovy;
  m_aspect = aspect;
}

void PerspectiveCameraController::setCamera( const Eigen::Matrix<GLdouble,3,1>& up, const GLdouble& theta, const GLdouble& phi, const GLdouble& rho, const Eigen::Matrix<GLdouble,3,1>& lookat )
{
  m_up_orientation = Eigen::Quaternion<GLdouble>::FromTwoVectors( Eigen::Matrix<GLdouble,3,1>::UnitY(), up );
  assert( fabs( m_up_orientation.norm() - 1.0 ) < 1.0e-6 );
  m_theta_cam = theta;
  m_phi_cam = phi;
  assert( rho > 0.0 );
  m_rho_cam = rho;
  m_cam_lookat = lookat;

  updatePositionAndFrame();
}

void PerspectiveCameraController::centerCameraAtSphere( const Eigen::Matrix<GLdouble,3,1>& center, const GLdouble& radius )
{
  m_cam_lookat = center;

  const GLdouble degrees_to_radians{ MathDefines::PI<GLdouble>() / 180.0 };
  const GLdouble d1{ radius / sin( 0.5 * m_fovy * degrees_to_radians ) };
  const GLdouble d2{ radius / sin( 0.5 * m_aspect * m_fovy * degrees_to_radians ) };

  m_rho_cam = std::max( d1, d2 );

  updatePositionAndFrame();
}

void PerspectiveCameraController::addToDistFromCenter( const GLdouble& dst_amnt )
{
  m_rho_cam = std::max( m_rho_cam + dst_amnt, 0.1 );
  updatePositionAndFrame();
}

const GLdouble& PerspectiveCameraController::getDistFromCenter() const
{
  return m_rho_cam;
}

void PerspectiveCameraController::addToZenithAngle( const GLdouble& znth_amnt )
{
  m_theta_cam += znth_amnt;
  if( m_theta_cam < 0.0 )
  {
    m_theta_cam = 0.0;
  }
  if( m_theta_cam > MathDefines::PI<GLdouble>() )
  {
    m_theta_cam = MathDefines::PI<GLdouble>();
  }
  updatePositionAndFrame();
}

void PerspectiveCameraController::addToAzimuthAngle( const GLdouble& azmth_amnt )
{
  m_phi_cam += azmth_amnt;
  updatePositionAndFrame();
}	

void PerspectiveCameraController::trackCameraHorizontal( const GLdouble& hrz_amnt  )
{
  m_cam_lookat += hrz_amnt * m_cam_y;
}

void PerspectiveCameraController::trackCameraVertical( const GLdouble& vrt_amnt  )
{
  m_cam_lookat += vrt_amnt * m_cam_x;
}

void PerspectiveCameraController::updatePositionAndFrame()
{
  assert( fabs( m_up_orientation.norm() - 1.0 ) < 1.0e-6 );

  m_cam_psn = m_up_orientation * Eigen::Matrix<GLdouble,3,1>{ sin(m_theta_cam) * sin(m_phi_cam), cos(m_theta_cam), sin(m_theta_cam) * cos(m_phi_cam) };
  m_cam_z = -1.0 * m_cam_psn;
  m_cam_psn *= m_rho_cam;
  m_cam_y = m_up_orientation * Eigen::Matrix<GLdouble,3,1>{ cos(m_phi_cam), 0.0, -sin(m_phi_cam) };
  m_cam_x = m_cam_y.cross( m_cam_z );

  assert( fabs( m_cam_x.norm() - 1.0 ) < 1.0e-6 );
  assert( fabs( m_cam_y.norm() - 1.0 ) < 1.0e-6 );
  assert( fabs( m_cam_z.norm() - 1.0 ) < 1.0e-6 );
}

void PerspectiveCameraController::reset()
{
  *this = PerspectiveCameraController();
}
