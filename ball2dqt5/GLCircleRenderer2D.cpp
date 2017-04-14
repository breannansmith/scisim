#include "GLCircleRenderer2D.h"

GLCircleRenderer2D::GLCircleRenderer2D( const unsigned num_pts )
: m_x_crds( num_pts )
, m_y_crds( num_pts )
{
  assert( num_pts >= 3 );
  // Sample the circle at equal intervals
  const GLdouble dtheta{ 2.0 * PI<GLdouble> / GLdouble( num_pts ) };
  for( unsigned pnt_num = 0; pnt_num < num_pts; ++pnt_num )
  {
    m_x_crds[pnt_num] = pnt_num * dtheta;
  }
  m_y_crds = m_x_crds;
  m_x_crds = m_x_crds.array().cos();
  m_y_crds = m_y_crds.array().sin();
}

void GLCircleRenderer2D::renderCircle( const Vector2s& x, const scalar& r ) const
{
  glPushMatrix();

  glTranslated( GLdouble( x.x() ), GLdouble( x.y() ), GLdouble( 0.0 ) );
  glScaled( GLdouble( r ), GLdouble( r ), GLdouble( 1.0 ) );

  glBegin( GL_LINE_LOOP );
  for( int pnt_num = 0; pnt_num < m_x_crds.size(); ++pnt_num )
  {
    glVertex2d( m_x_crds( pnt_num ), m_y_crds( pnt_num ) );
  }
  glEnd();

  glPopMatrix();
}

void GLCircleRenderer2D::renderSolidCircle( const Vector2s& x, const scalar& r ) const
{
  glPushMatrix();

  glTranslated( GLdouble( x.x() ), GLdouble( x.y() ), GLdouble( 0.0 ) );
  glScaled( GLdouble( r ), GLdouble( r ), GLdouble( 1.0 ) );

  glBegin( GL_POLYGON );
  for( int pnt_num = 0; pnt_num < m_x_crds.size(); ++pnt_num )
  {
    glVertex2d( m_x_crds( pnt_num ), m_y_crds( pnt_num ) );
  }
  glEnd();

  glPopMatrix();
}
