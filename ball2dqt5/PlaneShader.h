#ifndef PLANE_SHADER_H
#define PLANE_SHADER_H

#include <QOpenGLFunctions_3_3_Core>
#include <Eigen/Core>

class QMatrix4x4;

class PlaneShader final
{

public:

  PlaneShader();

  PlaneShader( const PlaneShader& ) = delete;
  PlaneShader( PlaneShader&& ) = delete;
  PlaneShader& operator=( const PlaneShader& ) = delete;
  PlaneShader& operator=( PlaneShader&& ) = delete;

  void initialize( QOpenGLFunctions_3_3_Core* f );
  void cleanup();

  void setTransform( const QMatrix4x4& pv );

  void draw();

  Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& planeData();

private:

  QOpenGLFunctions_3_3_Core* m_f;

  GLuint m_VAO;
  GLuint m_instance_VBO;
  GLuint m_program;

  GLint m_pv_mat_loc;

  // Storage for the planes to render
  // Size is: 6 * num_planes
  // center_of_mass normal depth width ...
  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_plane_data;

  bool m_data_buffered;

};

#endif
