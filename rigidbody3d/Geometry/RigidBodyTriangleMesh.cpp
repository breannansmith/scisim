// RigidBodyTriangleMesh.cpp
//
// Breannan Smith
// Last updated: 09/28/2015

#include "RigidBodyTriangleMesh.h"

#include "scisim/Math/MathUtilities.h"
#include "scisim/StringUtilities.h"
#include "scisim/Utilities.h"

#ifndef NDEBUG
#include "rigidbody3d/Geometry/MomentTools.h"
#endif

#ifdef USE_HDF5
#include "scisim/HDF5File.h"
#else
#include <iostream>
#endif


RigidBodyTriangleMesh::RigidBodyTriangleMesh( const RigidBodyTriangleMesh& other )
: m_input_file_name( other.m_input_file_name )
, m_verts( other.m_verts )
, m_faces( other.m_faces )
, m_volume( other.m_volume )
, m_I_on_rho( other.m_I_on_rho )
, m_center_of_mass( other.m_center_of_mass )
, m_R( other.m_R )
, m_samples( other.m_samples )
, m_convex_hull_samples( other.m_convex_hull_samples )
, m_cell_delta( other.m_cell_delta )
, m_grid_dimensions( other.m_grid_dimensions )
, m_grid_origin( other.m_grid_origin )
, m_signed_distance( other.m_signed_distance )
, m_grid_end( other.m_grid_end )
{}

// TODO: Make value return versions of HDF5 readMatrix
// TODO: Call the HDF5 version from the thing below

RigidBodyTriangleMesh::RigidBodyTriangleMesh( const std::string& input_file_name )
: m_input_file_name( input_file_name )
, m_verts()
, m_faces()
, m_volume()
, m_I_on_rho()
, m_center_of_mass()
, m_R()
, m_samples()
, m_convex_hull_samples()
, m_cell_delta()
, m_grid_dimensions()
, m_grid_origin()
, m_signed_distance()
, m_grid_end()
{
  #ifdef USE_HDF5
  HDF5File mesh_file( input_file_name, HDF5AccessType::READ_ONLY );

  // Load the mesh
  m_verts = mesh_file.read<Matrix3Xsc>( "mesh/vertices" );
  m_faces = mesh_file.read<Matrix3Xuc>( "mesh/faces" );
  assert( ( m_faces.array() < unsigned( m_verts.cols() ) ).all() );
  // Verify that each vertex is part of a face
  #ifndef NDEBUG
  {
    std::vector<bool> vertex_in_face( m_verts.cols(), false );
    for( int fce_num = 0; fce_num < m_faces.cols(); ++fce_num )
    {
      vertex_in_face[m_faces(0,fce_num)] = true;
      vertex_in_face[m_faces(1,fce_num)] = true;
      vertex_in_face[m_faces(2,fce_num)] = true;
    }
    assert( std::all_of( vertex_in_face.cbegin(), vertex_in_face.cend(), [](const bool in_face){ return in_face; } ) );
  }
  #endif

  // Load the moments
  m_volume = mesh_file.read<scalar>( "moments/volume" );
  assert( m_volume > 0.0 );
  m_I_on_rho = mesh_file.read<Vector3s>( "moments/I_on_rho" );
  assert( ( m_I_on_rho.array() > 0.0 ).all() );
  m_center_of_mass = mesh_file.read<Vector3s>( "moments/x" );
  m_R = mesh_file.read<Matrix3s>( "moments/R" );
  assert( fabs( m_R.determinant() - 1.0 ) <= 1.0e-6 );
  assert( ( m_R * m_R.transpose() - Eigen::Matrix3d::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  // TODO: Remove these checks and the duplicated code
  #ifndef NDEBUG
  {
    scalar volume_test;
    Vector3s I_test;
    Vector3s cm_test;
    Matrix3s R_test;
    MomentTools::computeMoments( m_verts, m_faces, volume_test, I_test, cm_test, R_test );
    assert( fabs( volume_test - m_volume ) <= 1.0e-6 );
    assert( ( I_test - m_I_on_rho ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    assert( ( cm_test - Vector3s::Zero() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    // TODO: Get the code so it doesn't try to rotate a diagonal again needlessly
    //assert( ( R_test - Matrix3s::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  }
  #endif

  // Load the surface samples
  m_samples = mesh_file.read<Matrix3Xsc>( "surface_samples/samples" );

  // Load the convex hull samples
  m_convex_hull_samples = mesh_file.read<Matrix3Xsc>( "convex_hull/vertices" );

  // Load the signed distance field
  m_cell_delta = mesh_file.read<Vector3s>( "sdf/cell_delta" );
  assert( ( m_cell_delta.array() > 0.0 ).all() );
  m_grid_dimensions = mesh_file.read<Vector3u>( "sdf/grid_dimensions" );
  assert( ( m_grid_dimensions.array() >= 1 ).all() );
  m_grid_origin = mesh_file.read<Vector3s>( "sdf/grid_origin" );
  m_signed_distance = mesh_file.read<VectorXs>( "sdf/signed_distance" );

  // For convienience, cache the opposite corner of the grid to the origin
  m_grid_end = m_grid_origin + ( ( m_grid_dimensions.array() - 1 ).cast<scalar>() * m_cell_delta.array() ).matrix();
  #else
  std::cerr << "Error, loading rigid body triangle meshes requires HDF5 support. Please recompile with USE_HDF5=ON." << std::endl;
  std::exit( EXIT_FAILURE );
  #endif
}

RigidBodyTriangleMesh::RigidBodyTriangleMesh( std::istream& input_stream )
: m_input_file_name( StringUtilities::deserialize( input_stream ) )
, m_verts( MathUtilities::deserialize<Matrix3Xsc>( input_stream ) )
, m_faces( MathUtilities::deserialize<Matrix3Xuc>( input_stream ))
, m_volume( Utilities::deserialize<scalar>( input_stream ) )
, m_I_on_rho( MathUtilities::deserialize<Vector3s>( input_stream ) )
, m_center_of_mass( MathUtilities::deserialize<Vector3s>( input_stream ) )
, m_R( MathUtilities::deserialize<Matrix3s>( input_stream ) )
, m_samples( MathUtilities::deserialize<Matrix3Xsc>( input_stream ) )
, m_convex_hull_samples( MathUtilities::deserialize<Matrix3Xsc>( input_stream ) )
, m_cell_delta( MathUtilities::deserialize<Vector3s>( input_stream ) )
, m_grid_dimensions( MathUtilities::deserialize<Vector3u>( input_stream ) )
, m_grid_origin( MathUtilities::deserialize<Vector3s>( input_stream ) )
, m_signed_distance( MathUtilities::deserialize<VectorXs>( input_stream ) )
, m_grid_end( MathUtilities::deserialize<Vector3s>( input_stream ) )
{
  assert( ( m_faces.array() < unsigned( m_verts.cols() ) ).all() );
  // Verify that each vertex is part of a face
  #ifndef NDEBUG
  {
    std::vector<bool> vertex_in_face( m_verts.cols(), false );
    for( int fce_num = 0; fce_num < m_faces.cols(); ++fce_num )
    {
      vertex_in_face[m_faces(0,fce_num)] = true;
      vertex_in_face[m_faces(1,fce_num)] = true;
      vertex_in_face[m_faces(2,fce_num)] = true;
    }
    assert( std::all_of( vertex_in_face.cbegin(), vertex_in_face.cend(), [](const bool in_face){ return in_face; } ) );
  }
  #endif
  assert( m_volume > 0.0 );
  assert( ( m_I_on_rho.array() > 0.0 ).all() );
  assert( fabs( m_R.determinant() - 1.0 ) <= 1.0e-6 );
  assert( ( m_R * m_R.transpose() - Eigen::Matrix3d::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  // TODO: Remove these checks and the duplicated code
  #ifndef NDEBUG
  {
    scalar volume_test;
    Vector3s I_test;
    Vector3s cm_test;
    Matrix3s R_test;
    MomentTools::computeMoments( m_verts, m_faces, volume_test, I_test, cm_test, R_test );
    assert( fabs( volume_test - m_volume ) <= 1.0e-6 );
    assert( ( I_test - m_I_on_rho ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    assert( ( cm_test - Vector3s::Zero() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
    // TODO: Get the code so it doesn't try to rotate a diagonal again needlessly
    //assert( ( R_test - Matrix3s::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-6 );
  }
  #endif
  assert( ( m_cell_delta.array() > 0.0 ).all() );
  assert( ( m_grid_dimensions.array() >= 1 ).all() );
}

RigidBodyTriangleMesh::~RigidBodyTriangleMesh()
{}

RigidBodyGeometryType RigidBodyTriangleMesh::getType() const
{
  return RigidBodyGeometryType::TRIANGLE_MESH;
}

std::unique_ptr<RigidBodyGeometry> RigidBodyTriangleMesh::clone() const
{
  return std::unique_ptr<RigidBodyGeometry>{ new RigidBodyTriangleMesh{ *this } };
}

void RigidBodyTriangleMesh::computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const
{
  min.setConstant(  std::numeric_limits<scalar>::infinity() );
  max.setConstant( -std::numeric_limits<scalar>::infinity() );

  // For each vertex
  for( int vrt_num = 0; vrt_num < m_verts.cols(); ++vrt_num )
  {
    const Array3s transformed_vertex{ R * m_verts.col( vrt_num ) + cm };
    min = min.min( transformed_vertex );
    max = max.max( transformed_vertex );
  }
  assert( ( min <= max ).all() );
}

void RigidBodyTriangleMesh::computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const
{
  M = density * m_volume;
  CM = m_center_of_mass;
  I = density * m_I_on_rho;
  R = m_R;
}

std::string RigidBodyTriangleMesh::name() const
{
  return "triangle_mesh";
}

void RigidBodyTriangleMesh::serialize( std::ostream& output_stream ) const
{
  Utilities::serialize( RigidBodyGeometryType::TRIANGLE_MESH, output_stream );
  StringUtilities::serialize( m_input_file_name, output_stream );
  MathUtilities::serialize( m_verts, output_stream );
  MathUtilities::serialize( m_faces, output_stream );
  Utilities::serialize( m_volume, output_stream );
  MathUtilities::serialize( m_I_on_rho, output_stream );
  MathUtilities::serialize( m_center_of_mass, output_stream );
  MathUtilities::serialize( m_R, output_stream );
  MathUtilities::serialize( m_samples, output_stream );
  MathUtilities::serialize( m_convex_hull_samples, output_stream );
  MathUtilities::serialize( m_cell_delta, output_stream );
  MathUtilities::serialize( m_grid_dimensions, output_stream );
  MathUtilities::serialize( m_grid_origin, output_stream );
  MathUtilities::serialize( m_signed_distance, output_stream );
  MathUtilities::serialize( m_grid_end, output_stream );
}

scalar RigidBodyTriangleMesh::volume() const
{
  return m_volume;
}

const Matrix3Xsc& RigidBodyTriangleMesh::vertices() const
{
  return m_verts;
}

const Matrix3Xuc& RigidBodyTriangleMesh::faces() const
{
  return m_faces;
}

const Matrix3Xsc& RigidBodyTriangleMesh::convexHullVertices() const
{
  return m_convex_hull_samples;
}

const std::string& RigidBodyTriangleMesh::inputFileName() const
{
  return m_input_file_name;
}

const Matrix3s& RigidBodyTriangleMesh::R() const
{
  return m_R;
}

const Matrix3Xsc& RigidBodyTriangleMesh::samples() const
{
  return m_samples;
}

const scalar& RigidBodyTriangleMesh::v( const unsigned i, const unsigned j, const unsigned k ) const
{
  assert( i < m_grid_dimensions.x() ); assert( j < m_grid_dimensions.y() ); assert( k < m_grid_dimensions.z() );
  assert( ( k * m_grid_dimensions.y() + j ) * m_grid_dimensions.x() + i < m_signed_distance.size() );
  return m_signed_distance( ( k * m_grid_dimensions.y() + j ) * m_grid_dimensions.x() + i );
}

bool RigidBodyTriangleMesh::detectCollision( const Vector3s& x, Vector3s& n ) const
{
  // If the point lies outside the grid, no collisions are possible
  if( ( x.array() < m_grid_origin.array() ).any() )
  {
    return false;
  }
  if( ( x.array() > m_grid_end.array() ).any() )
  {
    return false;
  }
  assert( ( x.array() >= m_grid_origin.array() ).all() );
  assert( ( x.array() <= m_grid_end.array() ).all() );

  // Determine which cell this point lies within
  const Array3u indices{ ( ( x - m_grid_origin ).array() / m_cell_delta.array() ).unaryExpr( [](const scalar& y) { return floor(y); } ).cast<unsigned>() };
  assert( ( indices + 1 < m_grid_dimensions.array() ).all() );

  // Compute the 'barycentric' coordinates of the point in the cell
  const Vector3s bc{ ( x.array() - ( m_grid_origin.array() + indices.cast<scalar>().array() * m_cell_delta.array() ) ) / m_cell_delta.array() };
  assert( ( bc.array() >= 0.0 ).all() ); assert( ( bc.array() <= 1.0 ).all() );

  // One minus the barycentric coordinates
  const Vector3s bci{ Vector3s::Ones() - bc };

  // Grab the value of the distance field at each grid point
  const scalar v000{ v( indices.x(),     indices.y(),     indices.z() ) };
  const scalar v100{ v( indices.x() + 1, indices.y(),     indices.z() ) };
  const scalar v010{ v( indices.x(),     indices.y() + 1, indices.z() ) };
  const scalar v110{ v( indices.x() + 1, indices.y() + 1, indices.z() ) };
  const scalar v001{ v( indices.x(),     indices.y(),     indices.z() + 1 ) };
  const scalar v101{ v( indices.x() + 1, indices.y(),     indices.z() + 1 ) };
  const scalar v011{ v( indices.x(),     indices.y() + 1, indices.z() + 1 ) };
  const scalar v111{ v( indices.x() + 1, indices.y() + 1, indices.z() + 1 ) };

  const scalar dist{ bci.z() * ( bci.y() * ( bci.x() * v000 + bc.x() * v100 ) + bc.y() * ( bci.x() * v010 + bc.x() * v110 ) ) +
                      bc.z() * ( bci.y() * ( bci.x() * v001 + bc.x() * v101 ) + bc.y() * ( bci.x() * v011 + bc.x() * v111 ) ) };

  // If the distance is positive, there was no collision
  if( dist > 0.0 )
  {
    return false;
  }

  // Gradient of trilinear interpolation
  n.x() = bci.z() * ( bci.y() * ( v100 - v000 ) + bc.y() * ( v110 - v010 ) )
         + bc.z() * ( bci.y() * ( v101 - v001 ) + bc.y() * ( v111 - v011 ) );

  n.y() = bci.z() * ( bci.x() * ( v010 - v000 ) + bc.x() * ( v110 - v100 ) )
         + bc.z() * ( bci.x() * ( v011 - v001 ) + bc.x() * ( v111 - v101 ) );

  n.z() = bci.y() * ( bci.x() * ( v001 - v000 ) + bc.x() * ( v101 - v100 ) )
         + bc.y() * ( bci.x() * ( v011 - v010 ) + bc.x() * ( v111 - v110 ) );

  n.array() /= m_cell_delta.array();
  n.normalize();

  return true;
}
