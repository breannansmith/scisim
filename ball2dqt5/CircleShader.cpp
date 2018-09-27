#include "CircleShader.h"

#include <QMatrix4x4>

#include <cassert>

#include "OpenGLShader.h"

#include "scisim/Math/MathDefines.h"

// TODO: Try a version that uses an EBO

static const char* const vertex_shader_source = {
  "#version 330 core\n"
  "layout (location = 0) in vec2 position;\n"
  "layout (location = 1) in vec2 center_of_mass;\n"
  "layout (location = 2) in float radius;\n"
  "layout (location = 3) in vec3 circle_color;\n"
  "uniform mat4 projection_view;\n"
  "out vec3 render_color;\n"
  "void main()\n"
  "{\n"
  "  gl_Position = projection_view * vec4(radius * position.x + center_of_mass.x, radius * position.y + center_of_mass.y, 0.0f, 1.0f);\n"
  "  render_color = circle_color;\n"
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

CircleShader::CircleShader()
: m_num_subdivs( 1 )
, m_f( nullptr )
, m_VAO( 0 )
, m_instance_VBO( 0 )
, m_program( 0 )
, m_pv_mat_loc( -1 )
, m_circle_data()
, m_data_buffered( false )
, m_buffer_size( 0 )
{}

static std::vector<GLfloat> tesselateCircle( const GLuint num_subdivs )
{
  std::vector<GLfloat> vertices( 6 * num_subdivs );
  const double dtheta{ 2.0 * PI<double> / double( num_subdivs ) };
  for( GLuint div_num = 0; div_num < num_subdivs; div_num++ )
  {
    const GLuint base_idx = 6 * div_num;
    vertices[base_idx + 0] = 0.0;
    vertices[base_idx + 1] = 0.0;
    vertices[base_idx + 2] = GLfloat( std::cos( div_num * dtheta ) );
    vertices[base_idx + 3] = GLfloat( std::sin( div_num * dtheta ) );
    vertices[base_idx + 4] = GLfloat( std::cos( ((div_num + 1) % num_subdivs) * dtheta ) );
    vertices[base_idx + 5] = GLfloat( std::sin( ((div_num + 1) % num_subdivs) * dtheta ) );
  }
  return vertices;
}

void CircleShader::initialize( const int num_subdivs, QOpenGLFunctions_3_3_Core* f )
{
  assert( num_subdivs > 0 );
  m_num_subdivs = num_subdivs;

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

  // Create a buffer for the circle geometry
  GLuint circle_vbo = 0;
  m_f->glGenBuffers( 1, &circle_vbo );
  {
    const std::vector<GLfloat> vertices = tesselateCircle( m_num_subdivs );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, circle_vbo );
    m_f->glBufferData( GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat), vertices.data(), GL_STATIC_DRAW );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
  }

  // Create a buffer for each body's center of mass, radius, and color
  m_f->glGenBuffers( 1, &m_instance_VBO );

  // Toss the buffer and binding commands into a VAO
  m_f->glGenVertexArrays( 1, &m_VAO );
  m_f->glBindVertexArray( m_VAO );
    // The circle geometry
    m_f->glEnableVertexAttribArray( 0 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, circle_vbo );
    m_f->glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), static_cast<GLvoid*>(nullptr) );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The circle centers of mass
    m_f->glEnableVertexAttribArray( 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), static_cast<GLvoid*>(nullptr) );
    m_f->glVertexAttribDivisor( 1, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The circle radii
    m_f->glEnableVertexAttribArray( 2 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 2, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)( 2 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 2, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The circle colors
    m_f->glEnableVertexAttribArray( 3 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 3, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)( 3 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 3, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
  m_f->glBindVertexArray( 0 );

  // Circle vbo is no longer needed
  m_f->glDeleteBuffers( 1, &circle_vbo );

  // Cache the uniform locations
  m_pv_mat_loc = m_f->glGetUniformLocation( m_program, "projection_view" );
  if( m_pv_mat_loc < 0 )
  {
    qFatal( "Error, failed to get unfirom location for 'projection_view'." );
  }
}

void CircleShader::cleanup()
{
  m_num_subdivs = 1;
  assert( m_f != nullptr );
  m_f->glDeleteVertexArrays( 1, &m_VAO );
  m_VAO = 0;
  m_f->glDeleteBuffers( 1, &m_instance_VBO );
  m_instance_VBO = 0;
  m_f->glDeleteProgram( m_program );
  m_program = 0;
  m_f = nullptr;
  m_pv_mat_loc = -1;
  // m_circle_data
  m_data_buffered = false;
  m_buffer_size = 0;
}

void CircleShader::setTransform( const QMatrix4x4& pv )
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_pv_mat_loc >= 0 );

  m_f->glUseProgram( m_program );
  m_f->glUniformMatrix4fv( m_pv_mat_loc, 1, GL_FALSE, pv.data() );
}

void CircleShader::draw()
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_VAO > 0 );

  assert( m_circle_data.size() % 6 == 0 );
  const long num_circles{ m_circle_data.size() / 6 };

  if( !m_data_buffered )
  {
    if( m_circle_data.size() != 0 )
    {
      m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
      if( m_buffer_size < m_circle_data.size() )
      {
        m_f->glBufferData( GL_ARRAY_BUFFER, sizeof(GLfloat) * m_circle_data.size(), m_circle_data.data(), GL_DYNAMIC_DRAW );
        m_buffer_size = int(m_circle_data.size());
      }
      else
      {
        m_f->glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * m_circle_data.size(), m_circle_data.data() );
      }
      m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    }
    m_data_buffered = true;
  }

  m_f->glUseProgram( m_program );
  m_f->glBindVertexArray( m_VAO );
  m_f->glDrawArraysInstanced( GL_TRIANGLES, 0, 3 * m_num_subdivs, GLsizei(num_circles) );
  m_f->glBindVertexArray( 0 );
}

Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& CircleShader::circleData()
{
  m_data_buffered = false;
  return m_circle_data;
}
