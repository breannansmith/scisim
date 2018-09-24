#include "RectangleShader.h"

#include <QMatrix4x4>

#include <cassert>

#include "OpenGLShader.h"

static const char* const vertex_shader_source = {
  "#version 330 core\n"
  "layout (location = 0) in vec2 position;\n"
  "layout (location = 1) in vec2 cm;\n"
  "layout (location = 2) in float theta;\n"
  "layout (location = 3) in float r0;\n"
  "layout (location = 4) in float r1;\n"
  "layout (location = 5) in vec3 rect_color;\n"
  "uniform mat4 projection_view;\n"
  "out vec3 render_color;\n"
  "void main()\n"
  "{\n"
  "  float sx = r0 * position.x;\n"
  "  float sy = r1 * position.y;\n"
  "  float c = cos(theta);\n"
  "  float s = sin(theta);\n"
  "  float x = c * sx - s * sy;\n"
  "  float y = s * sx + c * sy;\n"
  "  gl_Position = projection_view * vec4(cm.x + x, cm.y + y, 0.0f, 1.0f);\n"
  "  render_color = rect_color;\n"
  "}\n"
};

static const char* const fragment_shader_source = {
  "#version 330 core\n"
  "in vec3 render_color;\n"
  "out vec4 color;\n"
  "void main()\n"
  "{\n"
  "  color = vec4(render_color.x, render_color.y, render_color.z, 1.0f);\n"
  "}\n"
};

RectangleShader::RectangleShader()
: m_f( nullptr )
, m_VAO( 0 )
, m_instance_VBO( 0 )
, m_program( 0 )
, m_pv_mat_loc( -1 )
, m_data()
, m_data_buffered( false )
, m_last_copied_size( 0 )
{}

void RectangleShader::initialize( QOpenGLFunctions_3_3_Core* f )
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

  // Create a buffer for the rectangle geometry
  GLuint vbo = 0;
  m_f->glGenBuffers( 1, &vbo );
  {
    Eigen::Matrix<GLfloat,12,1> data;
    data << -1.0, -1.0,
             1.0, -1.0,
            -1.0,  1.0,
             1.0, -1.0,
             1.0,  1.0,
            -1.0,  1.0;
    m_f->glBindBuffer( GL_ARRAY_BUFFER, vbo );
    m_f->glBufferData( GL_ARRAY_BUFFER, data.size() * sizeof(GLfloat), data.data(), GL_STATIC_DRAW );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
  }

  // Create a buffer for each rectangle's center, orientation, and render width
  m_f->glGenBuffers( 1, &m_instance_VBO );

  // Toss the buffer and binding commands into a VAO
  m_f->glGenVertexArrays( 1, &m_VAO );
  m_f->glBindVertexArray( m_VAO );
    // The rectangle geometry
    m_f->glEnableVertexAttribArray( 0 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, vbo );
    m_f->glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), static_cast<GLvoid*>(nullptr) );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The rectangle centers
    m_f->glEnableVertexAttribArray( 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), static_cast<GLvoid*>(nullptr) );
    m_f->glVertexAttribDivisor( 1, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The rectangle orientations
    m_f->glEnableVertexAttribArray( 2 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 2, 1, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (GLvoid*)( 2 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 2, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The rectangle's first half-width
    m_f->glEnableVertexAttribArray( 3 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 3, 1, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (GLvoid*)( 3 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 3, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The rectangle's second half-width
    m_f->glEnableVertexAttribArray( 4 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 4, 1, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (GLvoid*)( 4 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 4, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The rectangle colors
    m_f->glEnableVertexAttribArray( 5 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 5, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(GLfloat), (GLvoid*)( 5 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 5, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
  m_f->glBindVertexArray( 0 );

  // VBO is no longer needed
  m_f->glDeleteBuffers( 1, &vbo );

  // Cache the uniform locations
  m_pv_mat_loc = m_f->glGetUniformLocation( m_program, "projection_view" );
  if( m_pv_mat_loc < 0 )
  {
    qFatal( "Error, failed to get unfirom location for 'projection_view'." );
  }
}

void RectangleShader::cleanup()
{
  assert( m_f != nullptr );
  m_f->glDeleteVertexArrays( 1, &m_VAO );
  m_f->glDeleteBuffers( 1, &m_instance_VBO );
  m_f->glDeleteProgram( m_program );
  m_f = nullptr;
}

void RectangleShader::setTransform( const QMatrix4x4& pv )
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_pv_mat_loc >= 0 );

  m_f->glUseProgram( m_program );
  m_f->glUniformMatrix4fv( m_pv_mat_loc, 1, GL_FALSE, pv.data() );
}

void RectangleShader::draw()
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_VAO > 0 );

  assert( m_data.size() % 8 == 0 );
  const long num_planes{ m_data.size() / 8 };

  if( !m_data_buffered )
  {
    if( m_data.size() != 0 )
    {
      m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
      if( m_last_copied_size != m_data.size() )
      {
        m_f->glBufferData( GL_ARRAY_BUFFER, sizeof(GLfloat) * m_data.size(), m_data.data(), GL_DYNAMIC_DRAW );
        m_last_copied_size = int(m_data.size());
      }
      else
      {
        m_f->glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * m_data.size(), m_data.data() );
      }
      m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    }
    m_data_buffered = true;
  }

  m_f->glUseProgram( m_program );
  m_f->glBindVertexArray( m_VAO );
  m_f->glDrawArraysInstanced( GL_TRIANGLES, 0, 6, GLsizei(num_planes) );
  m_f->glBindVertexArray( 0 );
}

Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& RectangleShader::data()
{
  m_data_buffered = false;
  return m_data;
}
