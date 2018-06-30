#ifndef RECTANGLE_SHADER_H
#define RECTANGLE_SHADER_H

#include <QOpenGLFunctions_3_3_Core>
#include <Eigen/Core>

class QMatrix4x4;

class RectangleShader final
{

public:

  RectangleShader();

  RectangleShader( const RectangleShader& ) = delete;
  RectangleShader( RectangleShader&& ) = delete;
  RectangleShader& operator=( const RectangleShader& ) = delete;
  RectangleShader& operator=( RectangleShader&& ) = delete;

  void initialize( QOpenGLFunctions_3_3_Core* f );
  void cleanup();

  void setTransform( const QMatrix4x4& pv );

  void draw();

  Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& data();

private:

  QOpenGLFunctions_3_3_Core* m_f;

  GLuint m_VAO;
  GLuint m_instance_VBO;
  GLuint m_program;

  GLint m_pv_mat_loc;

  // Storage for the planes to render
  // Size is: 8 * num_rectangles
  // center_of_mass theta r0 r1 color ...
  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_data;

  bool m_data_buffered;
  int m_last_copied_size;

};

#endif
