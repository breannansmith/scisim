#include "StaticCylinderRenderer.h"

#include "scisim/Math/MathDefines.h"
#include "rigidbody3d/StaticGeometry/StaticCylinder.h"

StaticCylinderRenderer::StaticCylinderRenderer( const int idx, const scalar& length )
: m_idx( idx )
, m_L( GLfloat( length ) )
, m_crds()
{
  const int num_pts{ 64 };
  m_crds.resize( 3 * num_pts );
  // Sample the circle at equal intervals
  const GLfloat dtheta{ static_cast<GLfloat>( 2.0 ) * MathDefines::PI<GLfloat>() / GLfloat( num_pts ) };
  using std::cos;
  using std::sin;
  for( int i = 0; i < num_pts; ++i )
  {
    m_crds( 3 * i + 0 ) = cos( GLfloat(i) * dtheta );
    m_crds( 3 * i + 1 ) = 0.0;
    m_crds( 3 * i + 2 ) = sin( GLfloat(i) * dtheta );
  }
}

void StaticCylinderRenderer::draw( const StaticCylinder& cylinder ) const
{
  glPushAttrib( GL_COLOR );
  glPushAttrib( GL_LIGHTING );
  glPushAttrib( GL_LINE_WIDTH );

  glDisable( GL_LIGHTING );
  glColor3d( 0.0, 0.0, 0.0 );
  glLineWidth( 1.0 );

  const Eigen::Matrix<GLfloat,3,1> center{ cylinder.x().cast<GLfloat>() };
  const Eigen::Matrix<GLfloat,3,1> translation{ 0.5f * m_L * cylinder.axis().cast<GLfloat>() };
  const Eigen::AngleAxis<GLfloat> rotation{ cylinder.R().cast<GLfloat>() };
  const GLfloat r{ static_cast<GLfloat>( cylinder.r() ) };

  // Draw the top circle
  glPushMatrix();
  glTranslatef( center.x() + translation.x(), center.y() + translation.y(), center.z() + translation.z() );
  glRotatef( ( 180.0f / MathDefines::PI<GLfloat>() ) * rotation.angle(), rotation.axis().x(), rotation.axis().y(), rotation.axis().z() );
  glScalef( r, 1.0, r );

  glEnableClientState( GL_VERTEX_ARRAY );
  glVertexPointer( 3, GL_FLOAT, 0, m_crds.data() );
  glDrawArrays( GL_LINE_LOOP, 0, GLsizei( m_crds.size() / 3 ) );
  glDisableClientState( GL_VERTEX_ARRAY );
  glPopMatrix();
  
  // Draw the bottom circle
  glPushMatrix();
  glTranslatef( center.x() - translation.x(), center.y() - translation.y(), center.z() - translation.z() );
  glRotatef( ( 180.0f / MathDefines::PI<GLfloat>() ) * rotation.angle(), rotation.axis().x(), rotation.axis().y(), rotation.axis().z() );
  glScalef( r, 1.0, r );
  
  glEnableClientState( GL_VERTEX_ARRAY );
  glVertexPointer( 3, GL_FLOAT, 0, m_crds.data() );
  glDrawArrays( GL_LINE_LOOP, 0, GLsizei( m_crds.size() / 3 ) );
  glDisableClientState( GL_VERTEX_ARRAY );
  glPopMatrix();

  // Draw the 'supports'
  glPushMatrix();
  glTranslatef( center.x(), center.y(), center.z() );
  glRotatef( ( 180.0f / MathDefines::PI<GLfloat>() ) * rotation.angle(), rotation.axis().x(), rotation.axis().y(), rotation.axis().z() );

  glBegin( GL_LINES );
  glVertex3f( 0.0, 0.5f * m_L, r );
  glVertex3f( 0.0, -0.5f * m_L, r );
  glVertex3f( 0.0, 0.5f * m_L, -r );
  glVertex3f( 0.0, -0.5f * m_L, -r );
  glVertex3f( r, 0.5f * m_L, 0.0 );
  glVertex3f( r, -0.5f * m_L, 0.0 );
  glVertex3f( -r, 0.5f * m_L, 0.0 );
  glVertex3f( -r, -0.5f * m_L, 0.0 );
  glEnd();
  glPopMatrix();

  glPopAttrib();
  glPopAttrib();
  glPopAttrib();
}
