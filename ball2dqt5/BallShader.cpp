#include "BallShader.h"

#include <QMatrix4x4>

#include <cassert>

static const GLfloat vertices[] = {
  -1.0f, -1.0f,
   1.0f, -1.0f,
  -1.0f, 1.0f
};

static const char* const vertex_shader_source = {
  "#version 330 core\n"
  "layout (location = 0) in vec2 position;\n"   // This is a vertex attrib
  "uniform mat4 projection_view;\n"
  "void main()\n"
  "{\n"
  "  gl_Position = projection_view * vec4(position.x, position.y, 0.0, 1.0);\n"
  "}\n"
};

static const char* const fragment_shader_source = {
  "#version 330 core\n"
  "out vec4 color;\n"
  "void main()\n"
  "{\n"
  "  color = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
  "}\n"
};

BallShader::BallShader()
: m_f( nullptr )
, m_VBO( 0 )
, m_VAO( 0 )
, m_program( 0 )
, m_pv_mat_loc( -1 )
{}

// TODO: Make this movable
class OpenGLShader final
{

public:

  OpenGLShader( const OpenGLShader& ) = delete;
  OpenGLShader( OpenGLShader&& ) = delete;
  OpenGLShader& operator=( const OpenGLShader& ) = delete;
  OpenGLShader& operator=( OpenGLShader&& ) = delete;

  OpenGLShader( GLenum shader_type, const char* const shader_source )
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

  ~OpenGLShader()
  {
    glDeleteShader( m_shader );
  }

  bool compiledSuccessfully() const
  {
    return m_shader != 0 && m_compile_status.empty();
  }

  const std::string& compileStatus() const
  {
    return m_compile_status;
  }

  GLuint shader() const
  {
    return m_shader;
  }

private:

  GLuint m_shader;
  std::string m_compile_status;

};

void BallShader::initialize( QOpenGLFunctions_3_3_Core* f )
{
  assert( f != nullptr );
  assert( m_f == nullptr );
  m_f = f;

  // Create a vertex shader
  const OpenGLShader vertex_shader{ GL_VERTEX_SHADER, vertex_shader_source };
  if( !vertex_shader.compiledSuccessfully() )
  {
    const std::string message{ std::string{"Error, failed to compile vertex shader.\n"} + vertex_shader.compileStatus() };
    qFatal( "%s", message.c_str() );
  }
  assert( vertex_shader.shader() > 0 );

  // Create a fragment shader
  const OpenGLShader fragment_shader{ GL_FRAGMENT_SHADER, fragment_shader_source };
  if( !fragment_shader.compiledSuccessfully() )
  {
    const std::string message{ std::string{"Error, failed to compile fragment shader.\n"} + fragment_shader.compileStatus() };
    qFatal( "%s", message.c_str() );
  }
  assert( fragment_shader.shader() > 0 );

  // Link the shaders
  m_program = m_f->glCreateProgram();
  m_f->glAttachShader( m_program, vertex_shader.shader() );
  m_f->glAttachShader( m_program, fragment_shader.shader() );
  m_f->glLinkProgram( m_program );
  {
    GLint success;
    m_f->glGetProgramiv( m_program, GL_LINK_STATUS, &success );
    if( !success )
    {
        GLchar infoLog[512];
        m_f->glGetProgramInfoLog( m_program, 512, nullptr, infoLog );
        const std::string message{ std::string{"Error, failed to compile fragment shader.\n"} + std::string{infoLog} };
        qFatal( "%s", message.c_str() );
    }
  }
  assert( m_program > 0 );

  // Create a buffer for the vertex information
  assert( m_VBO == 0 );
  m_f->glGenBuffers( 1, &m_VBO );

  // Toss the buffer and binding commands into a VAO
  m_f->glGenVertexArrays( 1, &m_VAO );
  m_f->glBindVertexArray( m_VAO );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_VBO );
    m_f->glBufferData( GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW );
    m_f->glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), static_cast<GLvoid*>(0) );
    m_f->glEnableVertexAttribArray( 0 );
  // m_f->glBindVertexArray( 0 );

  // Cache the uniform locations
  m_pv_mat_loc = m_f->glGetUniformLocation( m_program, "projection_view" );
  if( m_pv_mat_loc < 0 )
  {
    qFatal( "Error, failed to get unfirom location for 'projection_view'." );
  }
}

void BallShader::cleanup()
{
  assert( m_f != nullptr );
  m_f->glDeleteVertexArrays( 1, &m_VAO );
  m_f->glDeleteBuffers( 1, &m_VBO );
  m_f->glDeleteProgram( m_program );
  m_f = nullptr;
}

void BallShader::setTransform( const QMatrix4x4& pv )
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_pv_mat_loc >= 0 );

  m_f->glUseProgram( m_program );
  m_f->glUniformMatrix4fv( m_pv_mat_loc, 1, GL_FALSE, pv.data() );
}

void BallShader::draw()
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_VAO > 0 );

  m_f->glUseProgram( m_program );
  m_f->glBindVertexArray( m_VAO );
  m_f->glDrawArrays( GL_TRIANGLES, 0, 3 );
  // m_f->glBindVertexArray( 0 );
}