#include "OpenGLShader.h"

OpenGLShader::OpenGLShader( GLenum shader_type, const char* const shader_source )
: m_shader( 0 )
, m_compile_status()
{
  m_shader = glCreateShader( shader_type );
  glShaderSource( m_shader, 1, &shader_source, nullptr );
  glCompileShader( m_shader );
  {
    GLint success;
    glGetShaderiv( m_shader, GL_COMPILE_STATUS, &success );
    if( success == 0 )
    {
      GLchar info_log[512];
      glGetShaderInfoLog( m_shader, 512, nullptr, info_log );
      m_compile_status = info_log;
    }
  }
}

OpenGLShader::~OpenGLShader()
{
  glDeleteShader( m_shader );
}

bool OpenGLShader::compiledSuccessfully() const
{
  return m_shader != 0 && m_compile_status.empty();
}

const std::string& OpenGLShader::compileStatus() const
{
  return m_compile_status;
}

GLuint OpenGLShader::shader() const
{
  return m_shader;
}
