#include "PlaneShader.h"

#include <QMatrix4x4>

#include <cassert>

#include "OpenGLShader.h"

static const char* const vertex_shader_source = {
  "#version 330 core\n"
  "layout (location = 0) in vec2 position;\n"
  "layout (location = 1) in vec2 cm;\n"
  "layout (location = 2) in vec2 n;\n"
  "layout (location = 3) in float width;\n"
  "layout (location = 4) in float depth;\n"
  "uniform mat4 projection_view;\n"
  "void main()\n"
  "{\n"
  "  float sx = width * position.x;\n"
  "  float sy = depth * position.y;\n"
  "  float rx = n.x * sx - n.y * sy;\n"
  "  float ry = n.y * sx + n.x * sy;\n"
  "  gl_Position = projection_view * vec4(rx + cm.x, ry + cm.y, 0.0f, 1.0f);\n"
  "}\n"
};

static const char* const fragment_shader_source = {
  "#version 330 core\n"
  "out vec4 color;\n"
  "void main()\n"
  "{\n"
  "  color = vec4(0.0f, 0.0f, 0.0f, 1.0f);\n"
  "}\n"
};

PlaneShader::PlaneShader()
: m_f( nullptr )
, m_VAO( 0 )
, m_instance_VBO( 0 )
, m_program( 0 )
, m_pv_mat_loc( -1 )
, m_plane_data()
, m_data_buffered( false )
, m_last_copied_size( 0 )
{}

void PlaneShader::initialize( QOpenGLFunctions_3_3_Core* f )
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

  // Create a buffer for the plane geometry
  GLuint plane_vbo = 0;
  m_f->glGenBuffers( 1, &plane_vbo );
  {
    Eigen::Matrix<GLfloat,12,1> plane_data;
    plane_data <<  0.0,  0.5,
                  -1.0,  0.5,
                   0.0, -0.5,
                  -1.0,  0.5,
                   0.0, -0.5,
                  -1.0, -0.5;
    m_f->glBindBuffer( GL_ARRAY_BUFFER, plane_vbo );
    m_f->glBufferData( GL_ARRAY_BUFFER, plane_data.size() * sizeof(GLfloat), plane_data.data(), GL_STATIC_DRAW );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
  }

  // Create a buffer for each plane's center, orientation, and render width
  m_f->glGenBuffers( 1, &m_instance_VBO );

  // Toss the buffer and binding commands into a VAO
  m_f->glGenVertexArrays( 1, &m_VAO );
  m_f->glBindVertexArray( m_VAO );
    // The plane geometry
    m_f->glEnableVertexAttribArray( 0 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, plane_vbo );
    m_f->glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), static_cast<GLvoid*>(nullptr) );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The plane centers
    m_f->glEnableVertexAttribArray( 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), static_cast<GLvoid*>(nullptr) );
    m_f->glVertexAttribDivisor( 1, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The plane normals
    m_f->glEnableVertexAttribArray( 2 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 2, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)( 2 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 2, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The plane widths
    m_f->glEnableVertexAttribArray( 3 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 3, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)( 4 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 3, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    // The plane depths
    m_f->glEnableVertexAttribArray( 4 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 4, 1, GL_FLOAT, GL_FALSE, 6 * sizeof(GLfloat), (GLvoid*)( 5 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 4, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
  m_f->glBindVertexArray( 0 );

  // VBO is no longer needed
  m_f->glDeleteBuffers( 1, &plane_vbo );

  // Cache the uniform locations
  m_pv_mat_loc = m_f->glGetUniformLocation( m_program, "projection_view" );
  if( m_pv_mat_loc < 0 )
  {
    qFatal( "Error, failed to get unfirom location for 'projection_view'." );
  }
}

void PlaneShader::cleanup()
{
  assert( m_f != nullptr );
  m_f->glDeleteVertexArrays( 1, &m_VAO );
  m_f->glDeleteBuffers( 1, &m_instance_VBO );
  m_f->glDeleteProgram( m_program );
  m_f = nullptr;
}

void PlaneShader::setTransform( const QMatrix4x4& pv )
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_pv_mat_loc >= 0 );

  m_f->glUseProgram( m_program );
  m_f->glUniformMatrix4fv( m_pv_mat_loc, 1, GL_FALSE, pv.data() );
}

void PlaneShader::draw()
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_VAO > 0 );

  assert( m_plane_data.size() % 6 == 0 );
  const long num_planes{ m_plane_data.size() / 6 };

  // Check that normals are unit length
  #ifndef NDEBUG
  for (long plane_idx = 0; plane_idx < num_planes; plane_idx++)
  {
    const GLfloat norm = m_plane_data.segment<2>( 6 * plane_idx + 2 ).norm();
    assert( std::fabs(norm - 1.0f) <= 1.0e-6f );
  }
  #endif

  if( !m_data_buffered )
  {
    if( m_plane_data.size() != 0 )
    {
      m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
      if( m_last_copied_size != m_plane_data.size() )
      {
        m_f->glBufferData( GL_ARRAY_BUFFER, sizeof(GLfloat) * m_plane_data.size(), m_plane_data.data(), GL_DYNAMIC_DRAW );
        m_last_copied_size = m_plane_data.size();
      }
      else
      {
        m_f->glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * m_plane_data.size(), m_plane_data.data() );
      }
      m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    }
    m_data_buffered = true;
  }

  m_f->glUseProgram( m_program );
  m_f->glBindVertexArray( m_VAO );
  m_f->glDrawArraysInstanced( GL_TRIANGLES, 0, 6, num_planes );
  m_f->glBindVertexArray( 0 );
}

Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& PlaneShader::planeData()
{
  m_data_buffered = false;
  return m_plane_data;
}
