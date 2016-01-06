#include "BoxGeometryRenderer.h"

#include "rigidbody2d/BoxGeometry.h"

BoxGeometryRenderer::BoxGeometryRenderer( const BoxGeometry& geo )
: m_geo( geo )
{}

void BoxGeometryRenderer::render( const Eigen::Matrix<GLdouble,3,1>& color )
{
  glPushMatrix();
  glScaled( GLdouble( m_geo.r().x() ), GLdouble( m_geo.r().y() ), GLdouble( 1.0 ) );
  glPushAttrib( GL_COLOR );
  glColor3d( color.x(), color.y(), color.z() );
  glBegin( GL_TRIANGLE_STRIP );
  glVertex2d( -1.0, 1.0 );
  glVertex2d( -1.0, -1.0 );
  glVertex2d( 1.0, -1.0 );
  glVertex2d( 1.0, 1.0 );
  glVertex2d( -1.0, 1.0 );
  glEnd();
  glPopAttrib();
  glPopMatrix();
}

void BoxGeometryRenderer::renderTeleported( const Eigen::Matrix<GLdouble,3,1>& color )
{
  glPushMatrix();
  glScaled( GLdouble( m_geo.r().x() ), GLdouble( m_geo.r().y() ), GLdouble( 1.0 ) );
  glPushAttrib( GL_COLOR );
  glColor3d( color.x(), color.y(), color.z() );
  glBegin( GL_LINE_LOOP );
  glVertex2d( 1.0, 1.0 );
  glVertex2d( -1.0, 1.0 );
  glVertex2d( -1.0, -1.0 );
  glVertex2d( 1.0, -1.0 );
  glEnd();
  glPopAttrib();
  glPopMatrix();
}
