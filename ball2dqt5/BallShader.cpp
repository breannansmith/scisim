#include "BallShader.h"

#include <QMatrix4x4>

#include <cassert>

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

BallShader::BallShader()
: m_f( nullptr )
, m_VAO( 0 )
, m_instance_VBO( 0 )
, m_program( 0 )
, m_pv_mat_loc( -1 )
, m_num_circle_verts( 0 )
, m_circle_data()
, m_data_buffered( false )
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

static std::vector<GLfloat> tesselateCircle(const int num_divs)
{
  std::vector<GLfloat> vertices( 6 * num_divs );
  const GLfloat dtheta{ GLfloat( 2.0 ) * PI<GLfloat> / GLfloat( num_divs ) };
  for( int div_num = 0; div_num < num_divs; div_num++ )
  {
    const int base_idx = 6 * div_num;
    vertices[base_idx + 0] = 0.0;
    vertices[base_idx + 1] = 0.0;
    vertices[base_idx + 2] = std::cos( div_num * dtheta );
    vertices[base_idx + 3] = std::sin( div_num * dtheta );
    vertices[base_idx + 4] = std::cos( ((div_num + 1) % num_divs) * dtheta );
    vertices[base_idx + 5] = std::sin( ((div_num + 1) % num_divs) * dtheta );
  }
  return vertices;
}

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

  // Create a buffer for the circle geometry
  GLuint circle_vbo = 0;
  m_f->glGenBuffers( 1, &circle_vbo );
  {
    // TODO: Pull the circle generation code into a separate support function
    const std::vector<GLfloat> vertices = tesselateCircle( 32 );

    assert( vertices.size() % 2 == 0 );
    m_num_circle_verts = vertices.size() / 2;

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
    m_f->glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), static_cast<GLvoid*>(0) );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The circle centers of mass
    m_f->glEnableVertexAttribArray( 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), static_cast<GLvoid*>(0) );
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

void BallShader::cleanup()
{
  assert( m_f != nullptr );
  m_f->glDeleteVertexArrays( 1, &m_VAO );
  m_f->glDeleteBuffers( 1, &m_instance_VBO );
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

  assert( m_circle_data.size() % 6 == 0 );
  const long num_balls{ m_circle_data.size() / 6 };

  if( !m_data_buffered )
  {
    if( m_circle_data.size() != 0 )
    {
      m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
      m_f->glBufferData( GL_ARRAY_BUFFER, sizeof(GLfloat) * m_circle_data.size(), m_circle_data.data(), GL_DYNAMIC_DRAW );
      m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    }
    m_data_buffered = true;
  }

  m_f->glUseProgram( m_program );
  m_f->glBindVertexArray( m_VAO );
  m_f->glDrawArraysInstanced( GL_TRIANGLES, 0, m_num_circle_verts, num_balls );
  m_f->glBindVertexArray( 0 );
}

Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& BallShader::circleData()
{
  m_data_buffered = false;
  return m_circle_data;
}
