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
  glPushAttrib( GL_LINE_WIDTH );
  glLineWidth( 3.0 );
  glBegin( GL_LINE_LOOP );
  glVertex2d( 1.0, 1.0 );
  glVertex2d( -1.0, 1.0 );
  glVertex2d( -1.0, -1.0 );
  glVertex2d( 1.0, -1.0 );
  glEnd();
  glPopAttrib();
  glPopAttrib();
  glPopMatrix();
}

// Debug code to draw the principle axes of the box
//{
//  glPushAttrib( GL_LINE_WIDTH );
//  glColor3d( 0.8588235294, 0.6, 0.9098039216 );
//  glLineWidth( 5.0 );
//  glBegin( GL_LINES );
//  glVertex2d( 0.0, 0.0 );
//  glVertex2d( 1.0, 0.0 );
//  glEnd();
//  glColor3d( 0.2745098039, 0.8862745098, 0.6666666667 );
//  glBegin( GL_LINES );
//  glVertex2d( 0.0, 0.0 );
//  glVertex2d( 0.0, 1.0 );
//  glEnd();
//  glPopAttrib();
//}
