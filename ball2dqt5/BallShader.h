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

  // pmv: projection * modelview
  // void draw( const QMatrix4x4& pmv, const QMatrix4x4& world );
  void draw();

  Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& circleData();

private:

  QOpenGLFunctions_3_3_Core* m_f;

  GLuint m_VBO;
  GLuint m_VAO;
  GLuint m_program;

  GLint m_pv_mat_loc;
  GLint m_trans_vec_loc;
  GLint m_radius_float_loc;
  GLint m_color_vec_loc;

  // The number of verts in the circle template
  GLsizei m_num_circle_verts;

  // Storage for the circles to render
  // size is: 6 * num_circles
  Eigen::Matrix<GLfloat,Eigen::Dynamic,1> m_circle_data;

};

#endif
