#ifndef CIRCLE_SHADER_H
#define CIRCLE_SHADER_H

#include <QOpenGLFunctions_3_3_Core>
#include <Eigen/Core>

class QMatrix4x4;

class CircleShader final
{

public:

  CircleShader();

  CircleShader( const CircleShader& ) = delete;
  CircleShader( CircleShader&& ) = delete;
  CircleShader& operator=( const CircleShader& ) = delete;
  CircleShader& operator=( CircleShader&& ) = delete;

  void initialize( const int half_num_subdivs, QOpenGLFunctions_3_3_Core* f );
  void cleanup();

  void setTransform( const QMatrix4x4& pv );

  void draw();

  Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& circleData();

private:

  GLuint m_num_subdivs;

  QOpenGLFunctions_3_3_Core* m_f;

  GLuint m_VAO;
  GLuint m_instance_VBO;
  GLuint m_program;

  GLint m_pv_mat_loc;

  // Storage for the circles to render
  // Size is: 6 * num_circles
  // center_of_mass radius color ...
  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_circle_data;

  bool m_data_buffered;
  int m_buffer_size;

};

#endif
