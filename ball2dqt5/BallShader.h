#ifndef BALL_SHADER_H
#define BALL_SHADER_H

#include "scisim/Math/MathDefines.h"

#include <QOpenGLFunctions_3_3_Core>

class QMatrix4x4;

class BallShader final
{

public:

  BallShader();

  BallShader( const BallShader& ) = delete;
  BallShader( BallShader&& ) = delete;
  BallShader& operator=( const BallShader& ) = delete;
  BallShader& operator=( BallShader&& ) = delete;

  void initialize( QOpenGLFunctions_3_3_Core* f );
  void cleanup();

  void setTransform( const QMatrix4x4& pv );

  void draw();

  Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& circleData();

private:

  QOpenGLFunctions_3_3_Core* m_f;

  GLuint m_VAO;
  GLuint m_VBO;
  GLuint m_instance_VBO;
  GLuint m_program;

  GLint m_pv_mat_loc;

  // The number of verts in the circle template
  GLsizei m_num_circle_verts;

  // Storage for the circles to render
  // Size is: 6 * num_circles
  // center_of_mass radius color ...
  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_circle_data;

  bool m_data_buffered;

};

#endif
