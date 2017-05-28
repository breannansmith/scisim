#ifndef BALL_SHADER_H
#define BALL_SHADER_H

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

private:

  QOpenGLFunctions_3_3_Core* m_f;

  GLuint m_VBO;
  GLuint m_VAO;
  GLuint m_program;

  GLint m_pv_mat_loc;

  // TODO: allocate storage for circle centers of mass, radii, and colors
  // size is: 2 + 1 + 3 == 6

};

#endif
