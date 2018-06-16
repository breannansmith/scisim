#include "AnnulusShader.h"

#include <QMatrix4x4>

#include <cassert>

#include "OpenGLShader.h"

#include "scisim/Math/MathDefines.h"

static const char* const vertex_shader_source = {
  "#version 330 core\n"
  "layout (location = 0) in vec2 position;\n"
  "layout (location = 1) in vec2 center_of_mass;\n"
  "layout (location = 2) in float radius0;\n"
  "layout (location = 3) in float radius1;\n"
  "uniform mat4 projection_view;\n"
  "void main()\n"
  "{\n"
  "  float xpos = (1 - mod(gl_VertexID, 2)) * radius0 * position.x + mod(gl_VertexID, 2) * radius1 * position.x + center_of_mass.x;\n"
  "  float ypos = (1 - mod(gl_VertexID, 2)) * radius0 * position.y + mod(gl_VertexID, 2) * radius1 * position.y + center_of_mass.y;\n"
  "  gl_Position = projection_view * vec4(xpos, ypos, 0.0, 1.0);\n"
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

constexpr GLuint g_num_subdivs = 64;

AnnulusShader::AnnulusShader()
: m_f( nullptr )
, m_VAO( 0 )
, m_instance_VBO( 0 )
, m_program( 0 )
, m_pv_mat_loc( -1 )
, m_annulus_data()
, m_data_buffered( false )
, m_last_copied_size( 0 )
{}

static void generateIndexedAnnulus( Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& vertices, Eigen::Matrix<GLuint,Eigen::Dynamic,1>& indices )
{
  vertices.resize( 4 * g_num_subdivs );
  for( GLuint dvsn = 0; dvsn < g_num_subdivs; dvsn++ )
  {
    const double theta = double(dvsn) * 2.0 * PI<double> / double(g_num_subdivs);
    const GLfloat c = GLfloat(std::cos(theta));
    const GLfloat s = GLfloat(std::sin(theta));

    vertices[4 * dvsn + 0] = c;
    vertices[4 * dvsn + 1] = s;
    vertices[4 * dvsn + 2] = c;
    vertices[4 * dvsn + 3] = s;
  }

  indices.resize( 6 * g_num_subdivs );
  for( GLuint dvsn = 0; dvsn < g_num_subdivs; dvsn++ )
  {
    indices[6 * dvsn + 0] = (0 + 2 * dvsn) % (2 * g_num_subdivs);
    indices[6 * dvsn + 1] = (1 + 2 * dvsn) % (2 * g_num_subdivs);
    indices[6 * dvsn + 2] = (2 + 2 * dvsn) % (2 * g_num_subdivs);
    indices[6 * dvsn + 3] = (2 + 2 * dvsn) % (2 * g_num_subdivs);
    indices[6 * dvsn + 4] = (1 + 2 * dvsn) % (2 * g_num_subdivs);
    indices[6 * dvsn + 5] = (3 + 2 * dvsn) % (2 * g_num_subdivs);
  }
}

void AnnulusShader::initialize( QOpenGLFunctions_3_3_Core* f )
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

  // Create buffers for the circle geometry
  GLuint vbo;
  GLuint ebo;
  {
    Eigen::Matrix<GLfloat,Eigen::Dynamic,1> vertices;
    Eigen::Matrix<GLuint,Eigen::Dynamic,1> indices;
    generateIndexedAnnulus( vertices, indices );

    m_f->glGenBuffers( 1, &vbo );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, vbo );
    m_f->glBufferData( GL_ARRAY_BUFFER, vertices.size() * sizeof(GLfloat), vertices.data(), GL_STATIC_DRAW );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );

    m_f->glGenBuffers( 1, &ebo );
    m_f->glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, ebo );
    m_f->glBufferData( GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW );
    m_f->glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );
  }

  // Create a buffer for the centers of mass, inner radii, and outer radii
  m_f->glGenBuffers( 1, &m_instance_VBO );

  // Toss the buffer and binding commands into a VAO
  m_f->glGenVertexArrays( 1, &m_VAO );
  m_f->glBindVertexArray( m_VAO );
    // The annulus geometry
    m_f->glEnableVertexAttribArray( 0) ;
    m_f->glBindBuffer( GL_ARRAY_BUFFER, vbo );
    m_f->glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, ebo );
    m_f->glVertexAttribPointer( 0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), static_cast<GLvoid*>(nullptr) );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 ); 

    // The circle centers of mass
    m_f->glEnableVertexAttribArray( 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), static_cast<GLvoid*>(nullptr) );
    m_f->glVertexAttribDivisor( 1, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );

    // The inner radius
    m_f->glEnableVertexAttribArray( 2 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 2, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), (GLvoid*)( 2 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 2, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );

    // The outer radius
    m_f->glEnableVertexAttribArray( 3 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
    m_f->glVertexAttribPointer( 3, 1, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), (GLvoid*)( 3 * sizeof(GLfloat) ) );
    m_f->glVertexAttribDivisor( 3, 1 );
    m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    m_f->glBindVertexArray( 0 );

    // NB: *don't* unbind
    // m_f->glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );

  m_f->glBindVertexArray(0); 

  // VBO and EBO no longer needed
  m_f->glDeleteBuffers( 1, &vbo );
  m_f->glDeleteBuffers( 1, &ebo );

  // Cache the uniform locations
  m_pv_mat_loc = m_f->glGetUniformLocation( m_program, "projection_view" );
  if( m_pv_mat_loc < 0 )
  {
    qFatal( "Error, failed to get unfirom location for 'projection_view'." );
  }
}

void AnnulusShader::cleanup()
{
  assert( m_f != nullptr );
  m_f->glDeleteVertexArrays( 1, &m_VAO );
  m_f->glDeleteBuffers( 1, &m_instance_VBO );
  m_f->glDeleteProgram( m_program );
  m_f = nullptr;
}

void AnnulusShader::setTransform( const QMatrix4x4& pv )
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_pv_mat_loc >= 0 );

  m_f->glUseProgram( m_program );
  m_f->glUniformMatrix4fv( m_pv_mat_loc, 1, GL_FALSE, pv.data() );
}

void AnnulusShader::draw()
{
  assert( m_f != nullptr );
  assert( m_program > 0 );
  assert( m_VAO > 0 );

  assert( m_annulus_data.size() % 4 == 0 );
  const GLsizei num_annuli{ GLsizei(m_annulus_data.size() / 4) };

  if( !m_data_buffered )
  {
    if( m_annulus_data.size() != 0 )
    {
      m_f->glBindBuffer( GL_ARRAY_BUFFER, m_instance_VBO );
      if( m_last_copied_size != m_annulus_data.size() )
      {
        m_f->glBufferData( GL_ARRAY_BUFFER, sizeof(GLfloat) * m_annulus_data.size(), m_annulus_data.data(), GL_DYNAMIC_DRAW );
        m_last_copied_size = m_annulus_data.size();
      }
      else
      {
        m_f->glBufferSubData( GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * m_annulus_data.size(), m_annulus_data.data() );
      }
      m_f->glBindBuffer( GL_ARRAY_BUFFER, 0 );
    }
    m_data_buffered = true;
  }

  m_f->glUseProgram( m_program );
  m_f->glBindVertexArray( m_VAO );
  m_f->glDrawElementsInstanced( GL_TRIANGLES, 6 * g_num_subdivs, GL_UNSIGNED_INT, nullptr, num_annuli );
  m_f->glBindVertexArray( 0 );
}

Eigen::Matrix<GLfloat,Eigen::Dynamic,1>& AnnulusShader::annulusData()
{
  m_data_buffered = false;
  return m_annulus_data;
}
