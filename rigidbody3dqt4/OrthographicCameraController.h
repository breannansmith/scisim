#ifndef ORTHOGRAPHIC_CAMERA_CONTROLLER_H
#define ORTHOGRAPHIC_CAMERA_CONTROLLER_H

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

class OrthographicCameraController final
{

public:

  enum class ProjectionPlane : std::uint8_t { XY, ZY, ZX };

  OrthographicCameraController();

  void useXYView();
  void useZYView();
  void useZXView();

  void setCamera( const ProjectionPlane& projection_plane, const Eigen::Matrix<GLdouble,3,1>& lookat, const GLdouble& scale );

  // Sets the ModelViewMatrix to have the camera settings defined by this CameraController.
  void positionCamera();

  // Sets the projection matrix.
  void setPerspective( const int width, const int height );

  void translateView( const GLdouble& dx, const GLdouble& dy );

  void addToDistFromCenter( const GLdouble& dst_amnt );

  void setCenterAndScale( const Eigen::Matrix<GLdouble,3,1>& center, const GLdouble& scale );

  ProjectionPlane projectionPlane() const;
  const GLdouble& scale() const;

  const Eigen::Matrix<GLdouble,3,1>& x() const;

  void reset();

private:

  void updatePositionAndFrame();

  // Local frame of camera
  Eigen::Matrix<GLdouble,3,1> m_cam_x;
  Eigen::Matrix<GLdouble,3,1> m_cam_y;
  Eigen::Matrix<GLdouble,3,1> m_cam_z;

  // Spherical coordinates of camera
  // Note: theta wrt y
  //       phi wrt z
  GLdouble m_theta_cam;
  GLdouble m_phi_cam;

  // Position camera sphere is centered at
  Eigen::Matrix<GLdouble,3,1> m_cam_lookat;

  // Position of camera in cartesian coordinates
  // Note: This is not set directly, but computed from spherical coords
  Eigen::Matrix<GLdouble,3,1> m_cam_psn;

  // Rotation wrt <0,1,0> of up for the spherical coordinate system
  Eigen::Quaternion<GLdouble> m_up_orientation;

  GLdouble m_scale_factor;

  ProjectionPlane m_projection_plane;

};

#endif
