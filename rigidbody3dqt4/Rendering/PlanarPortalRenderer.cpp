#include "PlanarPortalRenderer.h"

#include <QtOpenGL>

#include "RigidBody3D/Portals/PlanarPortal.h"
#include "RigidBody3D/StaticGeometry/StaticPlane.h"

PlanarPortalRenderer::PlanarPortalRenderer( const unsigned idx, const Array2s& half_width0, const Array2s& half_width1 )
: m_idx( idx )
, m_half_width0( half_width0 )
, m_half_width1( half_width1 )
{
  assert( ( m_half_width0 >= 0.0 ).all() );
  assert( ( m_half_width1 >= 0.0 ).all() );
}

unsigned PlanarPortalRenderer::idx() const
{
  return m_idx;
}

void PlanarPortalRenderer::draw( const PlanarPortal& planar_portal ) const
{
  drawHalfPlane( planar_portal.planeA(), m_half_width0 );
  drawHalfPlane( planar_portal.planeB(), m_half_width1 );
}

void PlanarPortalRenderer::drawHalfPlane( const StaticPlane& plane, const Array2s& half_width ) const
{
  const Array3s scale{ half_width.x(), 0.0, half_width.y() };

  const Vector3s v0{ plane.R() * ( scale * Array3s{  1.0, 0.0,  1.0 } ).matrix() + plane.x() };
  const Vector3s v1{ plane.R() * ( scale * Array3s{ -1.0, 0.0,  1.0 } ).matrix() + plane.x() };
  const Vector3s v2{ plane.R() * ( scale * Array3s{ -1.0, 0.0, -1.0 } ).matrix() + plane.x() };
  const Vector3s v3{ plane.R() * ( scale * Array3s{  1.0, 0.0, -1.0 } ).matrix() + plane.x() };

  const Vector3s v4{ plane.R() * ( scale * Array3s{  0.0, 0.0,  1.0 } ).matrix() + plane.x() };
  const Vector3s v5{ plane.R() * ( scale * Array3s{ -1.0, 0.0,  0.0 } ).matrix() + plane.x() };
  const Vector3s v6{ plane.R() * ( scale * Array3s{  0.0, 0.0, -1.0 } ).matrix() + plane.x() };
  const Vector3s v7{ plane.R() * ( scale * Array3s{  1.0, 0.0,  0.0 } ).matrix() + plane.x() };

  glPushAttrib( GL_LINE_STIPPLE );
  glPushAttrib( GL_LINE_STIPPLE_PATTERN );

  glEnable( GL_LINE_STIPPLE );
  glLineStipple( 8, 0xAAAA );

  glBegin( GL_LINE_LOOP );
    glVertex3d( v0.x(), v0.y(), v0.z() );
    glVertex3d( v1.x(), v1.y(), v1.z() );
    glVertex3d( v2.x(), v2.y(), v2.z() );
    glVertex3d( v3.x(), v3.y(), v3.z() );
  glEnd();

  glBegin( GL_LINES );
    glVertex3d( v4.x(), v4.y(), v4.z() );
    glVertex3d( v6.x(), v6.y(), v6.z() );
    glVertex3d( v5.x(), v5.y(), v5.z() );
    glVertex3d( v7.x(), v7.y(), v7.z() );
  glEnd();

  glPopAttrib();
  glPopAttrib();
}
