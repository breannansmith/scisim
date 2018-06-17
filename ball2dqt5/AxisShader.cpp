#include "AxisShader.h"

#include <QMatrix4x4>

#include <cassert>

#include <Eigen/Core>

#include "OpenGLShader.h"

static const char* const vertex_shader_source = {
  "#version 330 core\n"
  "layout (location = 0) in vec3 position;\n"
//   "layout (location = 1) in vec2 cm;\n"
//   "layout (location = 2) in vec2 n;\n"
//   "layout (location = 3) in float width;\n"
//   "layout (location = 4) in float depth;\n"
  "uniform mat4 projection_view;\n"
  "void main()\n"
  "{\n"
//   "  float sx = width * position.x;\n"
//   "  float sy = depth * position.y;\n"
//   "  float rx = n.x * sx - n.y * sy;\n"
//   "  float ry = n.y * sx + n.x * sy;\n"
//   "  gl_Position = projection_view * vec4(rx + cm.x, ry + cm.y, 0.0f, 1.0f);\n"
  "  gl_Position = projection_view * vec4(position.x, position.y, 0.0f, position.z);\n"
  "}\n"
};

static const char* const fragment_shader_source = {
  "#version 330 core\n"
  "out vec4 color;\n"
  "void main()\n"
  "{\n"
  "  color = vec4(1.0f, 0.0f, 0.0f, 1.0f);\n"
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
    Eigen::Matrix<GLfloat,18,1> plane_data;
    constexpr GLfloat hw = 0.01;
                  // The x axis
    plane_data << 0,  hw, 1,
                  1,  0, 0,
                  0, -hw, 1,
                  // The y axis
                  -hw, 0, 1,
                   hw, 0, 1,
                    0, 1, 0;
    m_f->glBindBuffer( GL_ARRAY_BUFFER, vbo );
    m_f->glBufferData( GL_ARRAY_BUFFER, plane_data.size() * sizeof(GLfloat), plane_data.data(), GL_STATIC_DRAW );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
  }

  // Toss the buffer and binding commands into a VAO
  m_f->glGenVertexArrays( 1, &m_VAO );
  m_f->glBindVertexArray( m_VAO );
    m_f->glEnableVertexAttribArray( 0 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, vbo );
    m_f->glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), static_cast<GLvoid*>(nullptr) );
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
  m_f->glDrawArrays( GL_TRIANGLES, 0, 6 );
  m_f->glBindVertexArray( 0 );
}
