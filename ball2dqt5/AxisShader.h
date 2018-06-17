#ifndef AXIS_SHADER_H
#define AXIS_SHADER_H

#include <QOpenGLFunctions_3_3_Core>

class QMatrix4x4;

class AxisShader final
{

public:

  AxisShader();

  AxisShader( const AxisShader& ) = delete;
  AxisShader( AxisShader&& ) = delete;
  AxisShader& operator=( const AxisShader& ) = delete;
  AxisShader& operator=( AxisShader&& ) = delete;

  void initialize( QOpenGLFunctions_3_3_Core* f );
  void cleanup();

  void setTransform( const QMatrix4x4& pv );

  void draw();

private:

  QOpenGLFunctions_3_3_Core* m_f;

  GLuint m_VAO;
  GLuint m_program;
  GLint m_pv_mat_loc;

};

#endif
