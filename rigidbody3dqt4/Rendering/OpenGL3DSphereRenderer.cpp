#include "OpenGL3DSphereRenderer.h"

const GLfloat OpenGL3DSphereRenderer::g_vdata[4][3] =
{
  {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}
};

const GLint OpenGL3DSphereRenderer::g_tindices[2][3] =
{
  {0, 1, 2}, {3, 2, 1}
};


OpenGL3DSphereRenderer::OpenGL3DSphereRenderer( const unsigned num_subdivs )
: m_sphere_verts()
, m_sphere_normals()
{
  generateSphere( num_subdivs );
}

void OpenGL3DSphereRenderer::drawVertexArray( const Eigen::Matrix<GLfloat,3,1>& primary_color, const Eigen::Matrix<GLfloat,3,1>& secondary_color )
{
  const int num_mesh_verts{ static_cast<int>( m_sphere_verts.cols() ) };

  glEnableClientState( GL_VERTEX_ARRAY );
  glEnableClientState( GL_NORMAL_ARRAY );
  assert( m_sphere_verts.cols() == m_sphere_normals.cols() );
  glVertexPointer( 3, GL_FLOAT, 0, m_sphere_verts.data() );
  glNormalPointer( GL_FLOAT, 0, m_sphere_normals.data() );

  // Quad 1
  GLfloat mcolorambient[] = { GLfloat(0.3) * primary_color.x(), GLfloat(0.3) * primary_color.y(), GLfloat(0.3) * primary_color.z(), 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient );
  GLfloat mcolordiffuse[] = { primary_color.x(), primary_color.y(), primary_color.z(), (GLfloat) 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse );
  glDrawArrays( GL_TRIANGLES, 0, num_mesh_verts );

  // Quad 2
  glPushMatrix();
  glRotatef( 180.0, 0.0, 0.0, 1.0 );
  glDrawArrays( GL_TRIANGLES, 0, num_mesh_verts );
  glPopMatrix();

  // Quad 3
  GLfloat mcolorambient2[] = { GLfloat(0.3) * secondary_color.x(), GLfloat(0.3) * secondary_color.y(), GLfloat(0.3) * secondary_color.z(), 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, mcolorambient2 );
  GLfloat mcolordiffuse2[] = { secondary_color.x(), secondary_color.y(), secondary_color.z(), 1.0 };
  glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, mcolordiffuse2 );
  glPushMatrix();
  glRotatef( 90.0, 0.0, 0.0, 1.0 );
  glDrawArrays( GL_TRIANGLES, 0, num_mesh_verts );
  glPopMatrix();

  // Quad 4
  glPushMatrix();
  glRotatef( 270.0, 0.0, 0.0, 1.0 );
  glDrawArrays( GL_TRIANGLES, 0, num_mesh_verts );
  glPopMatrix();

  glDisableClientState( GL_NORMAL_ARRAY );
  glDisableClientState( GL_VERTEX_ARRAY );
}

void OpenGL3DSphereRenderer::saveTriangleInMem( const Eigen::Matrix<GLfloat,3,1>& v1, const Eigen::Matrix<GLfloat,3,1>& v2, const Eigen::Matrix<GLfloat,3,1>& v3, unsigned& current_vertex )
{
  assert( fabs( v1.norm() - 1.0 ) <= 1.0e-6 );
  m_sphere_verts.col( current_vertex ) = v1;
  m_sphere_normals.col( current_vertex ) = v1;
  ++current_vertex;

  assert( fabs( v2.norm() - 1.0 ) <= 1.0e-6 );
  m_sphere_verts.col( current_vertex ) = v2;
  m_sphere_normals.col( current_vertex ) = v2;
  ++current_vertex;

  assert( fabs( v3.norm() - 1.0 ) <= 1.0e-6 );
  m_sphere_verts.col( current_vertex ) = v3;
  m_sphere_normals.col( current_vertex ) = v3;
  ++current_vertex;
}

void OpenGL3DSphereRenderer::subdivide( const Eigen::Matrix<GLfloat,3,1>& v1, const Eigen::Matrix<GLfloat,3,1>& v2, const Eigen::Matrix<GLfloat,3,1>& v3, unsigned& current_vertex, const unsigned depth )
{
  // If we hit the lowest level of recursion
  if( depth == 0 )
  {
    // Save the current triangle
    saveTriangleInMem( v1, v2, v3, current_vertex );
    return;
  }

  // New vertices lie on the midpoint of the three edges of the larger triangle
  Eigen::Matrix<GLfloat,3,1> v12{ 0.5 * ( v1 + v2 ) };
  Eigen::Matrix<GLfloat,3,1> v23{ 0.5 * ( v2 + v3 ) };
  Eigen::Matrix<GLfloat,3,1> v31{ 0.5 * ( v3 + v1 ) };

  // Ensure that the new vertices are on the surface of the sphere
  v12 = v12.normalized();
  v23 = v23.normalized();
  v31 = v31.normalized();

  // This triangle is divided into four children
  subdivide( v1, v12, v31, current_vertex, depth - 1 );
  subdivide( v2, v23, v12, current_vertex, depth - 1 );
  subdivide( v3, v31, v23, current_vertex, depth - 1 );
  subdivide( v12, v23, v31, current_vertex, depth - 1 );
}

unsigned computeNumTriangles( const unsigned num_subdivs )
{
  unsigned num_tri{ 2 };
  for( unsigned i = 0; i < num_subdivs; ++i )
  {
    num_tri *= 4;
  }
  return num_tri;
}

void OpenGL3DSphereRenderer::allocateSphereMemory( const unsigned num_subdivs )
{
  const unsigned num_mesh_verts{ 3 * computeNumTriangles( num_subdivs ) };

  // 3 coordinates per vertex
  m_sphere_verts.resize( 3, num_mesh_verts );

  // 3 coordinates per vertex
  m_sphere_normals.resize( 3, num_mesh_verts );
}

void OpenGL3DSphereRenderer::generateSphere( const unsigned num_subdivs )
{
  allocateSphereMemory( num_subdivs );

  unsigned current_vertex{ 0 };
  // For each initial triangle in the quadrant
  for( unsigned i = 0; i < 2; ++i )
  {
    const Eigen::Matrix<GLfloat,3,1> vertex_a{ g_vdata[g_tindices[i][0]][0], g_vdata[g_tindices[i][0]][1], g_vdata[g_tindices[i][0]][2] };
    const Eigen::Matrix<GLfloat,3,1> vertex_b{ g_vdata[g_tindices[i][1]][0], g_vdata[g_tindices[i][1]][1], g_vdata[g_tindices[i][1]][2] };
    const Eigen::Matrix<GLfloat,3,1> vertex_c{ g_vdata[g_tindices[i][2]][0], g_vdata[g_tindices[i][2]][1], g_vdata[g_tindices[i][2]][2] };
    subdivide( vertex_a, vertex_b, vertex_c, current_vertex, num_subdivs );
  }
}
