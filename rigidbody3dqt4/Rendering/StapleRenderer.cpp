#include "StapleRenderer.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

static std::vector<Eigen::Matrix<GLfloat,3,1>> generatePointVector( const std::vector<Vector3s>& points )
{
  std::vector<Eigen::Matrix<GLfloat,3,1>> new_points( points.size() );
  for( std::vector<Vector3s>::size_type i = 0; i < points.size(); ++i )
  {
    new_points[i] = points[i].cast<GLfloat>();
  }
  return new_points;
}

StapleRenderer::StapleRenderer( const int num_subdivs, const std::vector<Vector3s>& points, const scalar& r )
: m_num_samples( computeNumSamples( num_subdivs ) )
, m_points( generatePointVector(points) )
, m_r( GLfloat( r ) )
, m_sphere_renderer( num_subdivs )
, m_cylinder_verts()
, m_cylinder_normals()
{
  assert( num_subdivs >= 0 );
  assert( m_points.size() == 4 );
  assert( m_r > 0.0f );

  initializeCylinderMemory();
}

StapleRenderer::~StapleRenderer()
{}

void StapleRenderer::renderBody( const Eigen::Matrix<GLfloat,3,1>& color )
{
  // Cap the ends of the staple
  for( int i = 0; i < 4; ++i )
  {
    glPushMatrix();
    glTranslatef( m_points[i].x(), m_points[i].y(), m_points[i].z() );
    glScalef( m_r, m_r, m_r );
    m_sphere_renderer.drawVertexArray( color, color );
    glPopMatrix();
  }

  GLfloat mcolorambient[] = { GLfloat( 0.3 ) * color.x(), GLfloat( 0.3 ) * color.y(), GLfloat( 0.3 ) * color.z(), 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
  GLfloat mcolordiffuse[] = { color.x(), color.y(), color.z(), 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );

  // Draw the horizontal cylinder
  glPushMatrix();
  assert( m_points[1].y() == m_points[2].y() );
  glTranslatef( 0.0, m_points[1].y(), 0.0 );
  assert( m_points[2].x() > m_points[1].x() );
  glScalef( m_points[2].x()-m_points[1].x(), 1.0, 1.0 );
  drawCylinder();
  glPopMatrix();

  // Draw the left vertical cylinder
  glPushMatrix();
  assert( m_points[0].x() == m_points[1].x() );
  assert( m_points[0].y() > m_points[1].y() );
  glTranslatef( m_points[0].x(), m_points[1].y() + 0.5f * ( m_points[0].y() - m_points[1].y() ), 0.0 );
  glRotatef( 90.0, 0.0, 0.0, 1.0 );
  assert( m_points[0].y() > m_points[1].y() );
  glScalef( m_points[0].y() - m_points[1].y(), 1.0, 1.0 );
  drawCylinder();
  glPopMatrix();

  // Draw the right vertical cylinder
  glPushMatrix();
  assert( m_points[2].x() == m_points[3].x() );
  assert( m_points[0].y() > m_points[1].y() );
  glTranslatef( m_points[2].x(), m_points[1].y() + 0.5f * ( m_points[0].y() - m_points[1].y() ), 0.0 );
  glRotatef( 90.0, 0.0, 0.0, 1.0 );
  assert( m_points[0].y() > m_points[1].y() );
  glScalef( m_points[0].y() - m_points[1].y(), 1.0, 1.0 );
  drawCylinder();
  glPopMatrix();
  
//  glPushAttrib(GL_COLOR);
//  glPushAttrib(GL_LIGHTING);
//  glPushAttrib(GL_POINT_SIZE);
//  
//  glDisable(GL_LIGHTING);
//  glPointSize(2.0);
//
//  glColor3d(1.0,0.0,0.0);
//  
//  glBegin( GL_POINTS );
//  glVertex3d( m_points[0].x(), m_points[0].y(), m_points[0].z() );
//  glVertex3d( m_points[1].x(), m_points[1].y(), m_points[1].z() );
//  glVertex3d( m_points[2].x(), m_points[2].y(), m_points[2].z() );
//  glVertex3d( m_points[3].x(), m_points[3].y(), m_points[3].z() );
//  glEnd();
//
//  glPopAttrib();
//  glPopAttrib();
//  glPopAttrib();
}

void StapleRenderer::drawCylinder()
{
  const int nv{ 4 * m_num_samples };

  glEnableClientState( GL_VERTEX_ARRAY );
  glEnableClientState( GL_NORMAL_ARRAY );

  glVertexPointer( 3, GL_FLOAT, 0, m_cylinder_verts.data() );
  glNormalPointer( GL_FLOAT, 0, m_cylinder_normals.data() );

  glDrawArrays( GL_QUADS, 0, nv );

  glDisableClientState( GL_NORMAL_ARRAY );
  glDisableClientState( GL_VERTEX_ARRAY );

//  const float dtheta = 2.0 * PI / ((float)m_num_samples);
//
//  glBegin( GL_TRIANGLES );
//  for( int quad_num = 0; quad_num < m_num_samples; ++ quad_num )
//  {
//    const float c0 = m_r * cos( quad_num * dtheta );
//    const float s0 = m_r * sin( quad_num * dtheta );
//    const float c1 = m_r * cos( ((quad_num+1)%m_num_samples) * dtheta );
//    const float s1 = m_r * sin( ((quad_num+1)%m_num_samples) * dtheta );
//
//    const Eigen::Vector3f v0( -0.5, c0, s0 );
//    const Eigen::Vector3f v1( -0.5, c1, s1 );
//    const Eigen::Vector3f v2(  0.5, c0, s0 );
//    const Eigen::Vector3f v3(  0.5, c1, s1 );
//
//    const Eigen::Vector3f n0 = ((v1-v0).cross(v2-v0)).normalized();
//
//    glNormal3f( n0.x(), n0.y(), n0.z() );
//    glVertex3f( v1.x(), v1.y(), v1.z() );
//    glVertex3f( v0.x(), v0.y(), v0.z() );
//    glVertex3f( v2.x(), v2.y(), v2.z() );
//    glVertex3f( v1.x(), v1.y(), v1.z() );
//    glVertex3f( v2.x(), v2.y(), v2.z() );
//    glVertex3f( v3.x(), v3.y(), v3.z() );
//  }
//  glEnd();
}

int StapleRenderer::computeNumSamples( const int num_subdivs ) const
{
  int num_samples{ 4 };
  for( int i = 0; i < num_subdivs; ++i )
  {
    num_samples *= 2;
  }
  return num_samples;
}

void StapleRenderer::initializeCylinderMemory()
{
  m_cylinder_verts.resize( 3 * 4 * m_num_samples );
  m_cylinder_normals.resize( 3 * 4 * m_num_samples );

  const GLfloat dtheta{ static_cast<GLfloat>( 2.0 ) * PI<GLfloat> / GLfloat( m_num_samples ) };

  using std::cos;
  using std::sin;
  for( int quad_num = 0; quad_num < m_num_samples; ++ quad_num )
  {
    const GLfloat c0{ m_r * cos( GLfloat(quad_num) * dtheta ) };
    const GLfloat s0{ m_r * sin( GLfloat(quad_num) * dtheta ) };
    const GLfloat c1{ m_r * cos( GLfloat( ( quad_num + 1 ) % m_num_samples ) * dtheta ) };
    const GLfloat s1{ m_r * sin( GLfloat( ( quad_num + 1 ) % m_num_samples ) * dtheta ) };

    m_cylinder_verts.segment<3>( 12 * quad_num + 0 ) = Eigen::Matrix<GLfloat,3,1>{ -0.5, c0, s0 };
    m_cylinder_verts.segment<3>( 12 * quad_num + 3 ) = Eigen::Matrix<GLfloat,3,1>{ -0.5, c1, s1 };
    m_cylinder_verts.segment<3>( 12 * quad_num + 6 ) = Eigen::Matrix<GLfloat,3,1>{  0.5, c1, s1 };
    m_cylinder_verts.segment<3>( 12 * quad_num + 9 ) = Eigen::Matrix<GLfloat,3,1>{  0.5, c0, s0 };

    const Eigen::Matrix<GLfloat,3,1> n0{ Eigen::Matrix<GLfloat,3,1>{ 0.0, c0, s0 }.normalized() };
    const Eigen::Matrix<GLfloat,3,1> n1{ Eigen::Matrix<GLfloat,3,1>{ 0.0, c1, s1 }.normalized() };

    m_cylinder_normals.segment<3>( 12 * quad_num + 0 ) = n0;
    m_cylinder_normals.segment<3>( 12 * quad_num + 3 ) = n1;
    m_cylinder_normals.segment<3>( 12 * quad_num + 6 ) = n1;
    m_cylinder_normals.segment<3>( 12 * quad_num + 9 ) = n0;
  }
}
