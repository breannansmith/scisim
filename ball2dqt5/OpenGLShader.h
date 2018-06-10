#ifndef OPEN_GL_SHADER_H
#define OPEN_GL_SHADER_H

#include <QOpenGLFunctions_3_3_Core>

// TODO: Make this movable
class OpenGLShader final
{

public:

  OpenGLShader( const OpenGLShader& ) = delete;
  OpenGLShader( OpenGLShader&& ) = delete;
  OpenGLShader& operator=( const OpenGLShader& ) = delete;
  OpenGLShader& operator=( OpenGLShader&& ) = delete;

  OpenGLShader( GLenum shader_type, const char* const shader_source );

  ~OpenGLShader();

  bool compiledSuccessfully() const;

  const std::string& compileStatus() const;

  GLuint shader() const;

private:

  GLuint m_shader;
  std::string m_compile_status;

};

#endif
