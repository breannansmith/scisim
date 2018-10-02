#include "AxisShader.h"

#include <QMatrix4x4>

#include <cassert>

#include <Eigen/Core>

#include "OpenGLShader.h"

static const char* const vertex_shader_source = {
  "#version 330 core\n"
  "layout (location = 0) in vec3 position;\n"
  "layout (location = 1) in vec3 color;\n"
  "uniform mat4 projection_view;\n"
  "out vec3 render_color;\n"
  "void main()\n"
  "{\n"
  "  gl_Position = projection_view * vec4(position.x, position.y, 0.0f, position.z);\n"
  "  render_color = color;\n"
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

AxisShader::AxisShader()
: m_f( nullptr )
, m_VAO( 0 )
, m_program( 0 )
, m_pv_mat_loc( -1 )
{}

void AxisShader::initialize( QOpenGLFunctions_3_3_Core* f )
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

  // Create a buffer for the geometry
  GLuint vbo = 0;
  m_f->glGenBuffers( 1, &vbo );
  {
    Eigen::Matrix<GLfloat,72,1> plane_data;
    constexpr GLfloat hw = 0.01f;
                  // The x axis
    plane_data << hw,  hw, 1, 1, 0, 0,
                   1,   0, 0, 1, 0, 0,
                  hw, -hw, 1, 1, 0, 0,
                  hw,  hw, 1, 1, 0, 0,
                 -hw, -hw, 1, 1, 0, 0,
                  hw, -hw, 1, 1, 0, 0,
                  // The y axis
                 -hw,  hw, 1, 0, 1, 0,
                  hw,  hw, 1, 0, 1, 0,
                   0,   1, 0, 0, 1, 0,
                  hw,  hw, 1, 0, 1, 0,
                 -hw,  hw, 1, 0, 1, 0,
                 -hw, -hw, 1, 0, 1, 0;
    m_f->glBindBuffer( GL_ARRAY_BUFFER, vbo );
    m_f->glBufferData( GL_ARRAY_BUFFER, plane_data.size() * sizeof(GLfloat), plane_data.data(), GL_STATIC_DRAW );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
  }

  // Toss the buffer and binding commands into a VAO
  m_f->glGenVertexArrays( 1, &m_VAO );
  m_f->glBindVertexArray( m_VAO );
    // Vertex positions
    m_f->glEnableVertexAttribArray( 0 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, vbo );
    m_f->glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), static_cast<GLvoid*>(nullptr) );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // Colors
    m_f->glEnableVertexAttribArray( 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, vbo );
    m_f->glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)( 3 * sizeof(GLfloat) ) );
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

void AxisShader::cleanup()
{
  assert( m_f != nullptr );
  m_f->glDeleteVertexArrays( 1, &m_VAO );
  m_f->glDeleteProgram( m_program );
  m_f = nullptr;
}

void AxisShader::setTransform( const QMatrix4x4& pv )
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_pv_mat_loc >= 0 );

  m_f->glUseProgram( m_program );
  m_f->glUniformMatrix4fv( m_pv_mat_loc, 1, GL_FALSE, pv.data() );
}

void AxisShader::draw()
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_VAO > 0 );

  m_f->glUseProgram( m_program );
  m_f->glBindVertexArray( m_VAO );
  m_f->glDrawArrays( GL_TRIANGLES, 0, 12 );
  m_f->glBindVertexArray( 0 );
}
