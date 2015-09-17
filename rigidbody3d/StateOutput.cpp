// StateOutput.cpp
//
// Breannan Smith
// Last updated: 09/14/2015

#include "StateOutput.h"

#include "scisim/HDF5File.h"
#include "scisim/Utilities.h"
#include "Geometry/RigidBodyGeometry.h"
#include "Geometry/RigidBodyBox.h"
#include "Geometry/RigidBodySphere.h"
#include "Geometry/RigidBodyTriangleMesh.h"
#include "StaticGeometry/StaticPlane.h"
#include "StaticGeometry/StaticCylinder.h"

#include <iostream>

// TODO: Move this HDF5 stuff out of here
#ifdef USE_HDF5
using HDFGID = HDFID<H5Gclose>;
using HDFTID = HDFID<H5Tclose>;
using HDFSID = HDFID<H5Sclose>;
using HDFDID = HDFID<H5Dclose>;
#endif

void StateOutput::writeGeometryIndices( const std::vector<std::unique_ptr<RigidBodyGeometry>>& geometry, const std::vector<unsigned>& indices, const std::string& group, HDF5File& output_file )
{
  #ifdef USE_HDF5
  // Map global indices to 'local' indices; that is, given a global index, gives the index into the particular type (e.g. global index 10 could be sphere number 3)
  VectorXu global_local_geo_mapping{ static_cast<VectorXu::Index>( geometry.size() ) };
  {
    unsigned current_geo_idx{ 0 };
    Vector4u geo_type_local_indices{ Vector4u::Zero() };
    for( const std::unique_ptr<RigidBodyGeometry>& current_geo : geometry )
    {
      global_local_geo_mapping( current_geo_idx++ ) = geo_type_local_indices( static_cast<int>( current_geo->getType() ) )++;
    }
  }

  // Create an HDF5 dataspace
  const hsize_t dim[]{ 2, indices.size() };
  const HDFSID data_space{ H5Screate_simple( 2, dim, nullptr ) };
  if( data_space < 0 )
  {
    throw std::string{ "Failed to create HDF dataspace for geometry indices" };
  }

  // Create an HDF5 dataset
  const HDFDID data_set{ H5Dcreate2( output_file.fileID(), ( group + "/geometry_indices" ).c_str(), H5T_NATIVE_UINT, data_space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT ) };
  if( data_set < 0 )
  {
    throw std::string{ "Failed to create HDF dataset for geometry indices" };
  }

  // Create an HDF5 memspace to allow us to insert elements one by one
  const HDFSID mem_space{ H5Screate_simple( 1, dim, nullptr ) };
  if( mem_space < 0 )
  {
    throw std::string{ "Failed to create HDF memspace for geometry indices" };
  }

  // Insert the geometry indices one by one
  unsigned current_geo{ 0 };
  for( const unsigned geo_idx : indices )
  {
    const hsize_t count[]{ 2, 1 };
    const hsize_t offset[]{ 0, current_geo++ };
    const hsize_t mem_offset[]{ 0, 0 };
    H5Sselect_hyperslab( data_space, H5S_SELECT_SET, offset, nullptr, count, nullptr );
    H5Sselect_hyperslab( mem_space, H5S_SELECT_SET, mem_offset, nullptr, count, nullptr );
    const unsigned data[]{ unsigned( geometry[geo_idx]->getType() ), global_local_geo_mapping( geo_idx ) };
    if( H5Dwrite( data_set, H5T_NATIVE_UINT, mem_space, data_space, H5P_DEFAULT, &data ) < 0 )
    {
      throw std::string{ "Failed to write geometry indices to HDF" };
    }
  }
  assert( current_geo == indices.size() );
  #else
  throw std::string{ "writeGeometryIndices not compiled with HDF5 support" };
  #endif
}

static void writeBoxGeometry( const std::vector<std::unique_ptr<RigidBodyGeometry>>& geometry, const unsigned box_count, const std::string& group, HDF5File& output_file )
{
  #ifdef USE_HDF5
  struct LocalBoxData
  {
    scalar r[3];
  };

  // Create an HDF5 dataspace
  const hsize_t dim[]{ box_count };
  const HDFSID data_space{ H5Screate_simple( 1, dim, nullptr ) };
  if( data_space < 0 )
  {
    throw std::string{ "Failed to create HDF dataspace for box geometry" };
  }

  // Create an HDF5 struct for the data
  const HDFTID struct_tid{ H5Tcreate( H5T_COMPOUND, sizeof( LocalBoxData ) ) };
  if( struct_tid < 0 )
  {
    throw std::string{ "Failed to create HDF struct for box geometry" };
  }
  // Insert the r type in the struct
  {
    const hsize_t array_dim[]{ 3 };
    const HDFTID array_tid{ H5Tarray_create2( H5T_NATIVE_DOUBLE, 1, array_dim ) };
    if( array_tid < 0 )
    {
      throw std::string{ "Failed to create HDF r type for box geometry" };
    }
    if( H5Tinsert( struct_tid, "r", HOFFSET( LocalBoxData, r ), array_tid ) < 0 )
    {
      throw std::string{ "Failed to insert r in HDF struct for box geometry" };
    }
  }

  // Create an HDF5 dataset
  const HDFDID data_set{ H5Dcreate2( output_file.fileID(), ( group + "/boxes" ).c_str(), struct_tid, data_space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT ) };
  if( data_set < 0 )
  {
    throw std::string{ "Failed to create HDF dataset for box geometry" };
  }

  // Create an HDF5 memspace to allow us to insert elements one by one
  const HDFSID memspace{ H5Screate_simple( 1, dim, nullptr ) };

  // Insert the box geometry one by one
  unsigned current_box{ 0 };
  LocalBoxData local_data;
  for( const std::unique_ptr<RigidBodyGeometry>& geometry_instance : geometry )
  {
    if( geometry_instance->getType() == RigidBodyGeometryType::BOX )
    {
      const RigidBodyBox& box{ sd_cast<const RigidBodyBox&>( *geometry_instance ) };
      Eigen::Map<Vector3s>{ local_data.r } = box.halfWidths();
      const hsize_t count[]{ 1 };
      const hsize_t offset[]{ current_box++ };
      const hsize_t mem_offset[]{ 0 };
      H5Sselect_hyperslab( data_space, H5S_SELECT_SET, offset, nullptr, count, nullptr );
      H5Sselect_hyperslab( memspace, H5S_SELECT_SET, mem_offset, nullptr, count, nullptr );
      if( H5Dwrite( data_set, struct_tid, memspace, data_space, H5P_DEFAULT, &local_data ) < 0 )
      {
        throw std::string{ "Failed to write box geometry struct to HDF" };
      }
    }
  }
  assert( current_box == box_count );
  #else
  throw std::string{ "writeSphereGeometry not compiled with HDF5 support" };
  #endif
}

static void writeSphereGeometry( const std::vector<std::unique_ptr<RigidBodyGeometry>>& geometry, const unsigned sphere_count, const std::string& group, HDF5File& output_file )
{
  #ifdef USE_HDF5
  struct SphereData
  {
    scalar r;
  };

  // Create an HDF5 dataspace
  const hsize_t dim[]{ sphere_count };
  const HDFSID data_space{ H5Screate_simple( 1, dim, nullptr ) };
  if( data_space < 0 )
  {
    throw std::string{ "Failed to create HDF dataspace for sphere geometry" };
  }

  // Create an HDF5 struct for the data
  const HDFTID struct_tid{ H5Tcreate( H5T_COMPOUND, sizeof( SphereData ) ) };
  if( struct_tid < 0 )
  {
    throw std::string{ "Failed to create HDF struct" };
  }
  // Insert the r type in the struct
  if( H5Tinsert( struct_tid, "r", HOFFSET( SphereData, r ), H5T_NATIVE_DOUBLE ) < 0 )
  {
    throw std::string{ "Failed to create HDF r type for sphere geometry" };
  }

  // Create an HDF5 dataset
  const HDFDID data_set{ H5Dcreate2( output_file.fileID(), ( group + "/spheres" ).c_str(), struct_tid, data_space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT ) };
  if( data_set < 0 )
  {
    throw std::string{ "Failed to create HDF dataset for sphere geometry" };
  }

  // Create an HDF5 memspace to allow us to insert elements one by one
  const HDFSID mem_space{ H5Screate_simple( 1, dim, nullptr ) };
  if( mem_space < 0 )
  {
    throw std::string{ "Failed to create HDF memspace for sphere geometry" };
  }

  // Insert the spheres one by one
  unsigned current_sphere{ 0 };
  SphereData data;
  for( const std::unique_ptr<RigidBodyGeometry>& geometry_instance : geometry )
  {
    if( geometry_instance->getType() == RigidBodyGeometryType::SPHERE )
    {
      const RigidBodySphere& sphere{ sd_cast<const RigidBodySphere&>( *geometry_instance ) };
      hsize_t count[]{ 1 };
      hsize_t offset[]{ current_sphere++ };
      hsize_t mem_offset[]{ 0 };
      H5Sselect_hyperslab( data_space, H5S_SELECT_SET, offset, nullptr, count, nullptr );
      H5Sselect_hyperslab( mem_space, H5S_SELECT_SET, mem_offset, nullptr, count, nullptr );
      data.r = sphere.r();
      if( H5Dwrite( data_set, struct_tid, mem_space, data_space, H5P_DEFAULT, &data ) < 0 )
      {
        throw std::string{ "Failed to write sphere geometry struct to HDF" };
      }
    }
  }
  assert( current_sphere == sphere_count );
  #else
  throw std::string{ "writeSphereGeometry not compiled with HDF5 support" };
  #endif
}

static void writeMeshGeometry( const std::vector<std::unique_ptr<RigidBodyGeometry>>& geometry, const unsigned mesh_count, const std::string& group, HDF5File& output_file )
{
  #ifdef USE_HDF5
  struct LocalMeshData
  {
    const char* file_name;
  };

  // Create an HDF5 dataspace
  const hsize_t dim[]{ mesh_count };
  const HDFSID data_space{ H5Screate_simple( 1, dim, nullptr ) };
  if( data_space < 0 )
  {
    throw std::string{ "Failed to create HDF dataspace for mesh geometry" };
  }

  // Create an HDF5 struct for the data
  const HDFTID struct_tid{ H5Tcreate( H5T_COMPOUND, sizeof( LocalMeshData ) ) };
  if( struct_tid < 0 )
  {
    throw std::string{ "Failed to create HDF struct for mesh geometry" };
  }
  // Insert the string type in the struct
  {
    HDFTID vlen_tid{ H5Tcopy( H5T_C_S1 ) };
    if( vlen_tid < 0 )
    {
      throw std::string{ "Failed to create HDF copy string type for mesh geometry" };
    }
    if( H5Tset_size( vlen_tid, H5T_VARIABLE ) < 0 )
    {
      throw std::string{ "Failed to create set size for string type for mesh geometry" };
    }
    if( H5Tinsert( struct_tid, "file_name", HOFFSET( LocalMeshData, file_name ), vlen_tid ) < 0 )
    {
      throw std::string{ "Failed to insert file_name in HDF struct for mesh geometry" };
    }
  }

  // Create an HDF5 dataset
  const HDFDID data_set{ H5Dcreate2( output_file.fileID(), ( group + "/meshes" ).c_str(), struct_tid, data_space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT ) };
  if( data_set < 0 )
  {
    throw std::string{ "Failed to create HDF dataset for mesh geometry" };
  }

  // Create an HDF5 memspace to allow us to insert elements one by one
  const HDFSID memspace{ H5Screate_simple( 1, dim, nullptr ) };

  // Insert the mesh geometry one by one
  unsigned current_mesh{ 0 };
  for( const std::unique_ptr<RigidBodyGeometry>& geometry_instance : geometry )
  {
    if( geometry_instance->getType() == RigidBodyGeometryType::TRIANGLE_MESH )
    {
      const RigidBodyTriangleMesh& mesh{ sd_cast<const RigidBodyTriangleMesh&>( *geometry_instance ) };
      const LocalMeshData local_data{ mesh.inputFileName().c_str() };
      const hsize_t count[]{ 1 };
      const hsize_t offset[]{ current_mesh++ };
      const hsize_t mem_offset[]{ 0 };
      H5Sselect_hyperslab( data_space, H5S_SELECT_SET, offset, nullptr, count, nullptr );
      H5Sselect_hyperslab( memspace, H5S_SELECT_SET, mem_offset, nullptr, count, nullptr );
      if( H5Dwrite( data_set, struct_tid, memspace, data_space, H5P_DEFAULT, &local_data ) < 0 )
      {
        throw std::string{ "Failed to write mesh geometry struct to HDF" };
      }
    }
  }
  assert( current_mesh == mesh_count );
  #else
  throw std::string{ "writeMeshGeometry not compiled with HDF5 support" };
  #endif
}

void StateOutput::writeGeometry( const std::vector<std::unique_ptr<RigidBodyGeometry>>& geometry, const std::string& group, HDF5File& output_file )
{
  Vector4u body_count{ Vector4u::Zero() };
  for( const std::unique_ptr<RigidBodyGeometry>& geometry_instance : geometry )
  {
    const RigidBodyGeometryType geo_type{ geometry_instance->getType() };
    switch( geo_type )
    {
      case RigidBodyGeometryType::BOX:
        ++body_count( 0 );
        break;
      case RigidBodyGeometryType::SPHERE:
        ++body_count( 1 );
        break;
      case RigidBodyGeometryType::STAPLE:
        ++body_count( 2 );
        break;
      case RigidBodyGeometryType::TRIANGLE_MESH:
        ++body_count( 3 );
        break;
    }
  }
  assert( body_count.sum() == geometry.size() );

  if( body_count( 0 ) != 0 )
  {
    writeBoxGeometry( geometry, body_count( 0 ), group, output_file );
  }
  if( body_count( 1 ) != 0 )
  {
    writeSphereGeometry( geometry, body_count( 1 ), group, output_file );
  }
  if( body_count( 2 ) != 0 )
  {
    std::cerr << "Staple output not coded up" << std::endl;
    std::exit( EXIT_FAILURE );
  }
  if( body_count( 3 ) != 0 )
  {
    writeMeshGeometry( geometry, body_count( 3 ), group, output_file );
  }
}

void StateOutput::writeStaticPlanes( const std::vector<StaticPlane>& static_planes, const std::string& group, HDF5File& output_file )
{
  #ifdef USE_HDF5
  struct LocalStaticPlaneData
  {
    scalar x[3];
    scalar R[4];
    scalar v[3];
    scalar omega[3];
  };

  // Create an HDF5 dataspace
  const hsize_t dim[]{ static_planes.size() };
  const HDFSID data_space{ H5Screate_simple( 1, dim, nullptr ) };
  if( data_space < 0 )
  {
    throw std::string{ "Failed to create HDF dataspace for static planes" };
  }

  // Create an HDF5 struct for the data
  const HDFTID struct_tid{ H5Tcreate( H5T_COMPOUND, sizeof( LocalStaticPlaneData ) ) };
  if( struct_tid < 0 )
  {
    throw std::string{ "Failed to create HDF struct for static planes" };
  }
  // Insert the x type in the struct
  {
    const hsize_t array_dim[]{ 3 };
    const HDFTID array_tid{ H5Tarray_create2( H5T_NATIVE_DOUBLE, 1, array_dim ) };
    if( array_tid < 0 )
    {
      throw std::string{ "Failed to create HDF x type for static planes" };
    }
    if( H5Tinsert( struct_tid, "x", HOFFSET( LocalStaticPlaneData, x ), array_tid ) < 0 )
    {
      throw std::string{ "Failed to insert x in HDF struct for static planes" };
    }
  }
  // Insert the R type in the struct
  {
    const hsize_t array_dim[]{ 4 };
    const HDFTID array_tid{ H5Tarray_create2( H5T_NATIVE_DOUBLE, 1, array_dim ) };
    if( array_tid < 0 )
    {
      throw std::string{ "Failed to create HDF R type for static planes" };
    }
    if( H5Tinsert( struct_tid, "R", HOFFSET( LocalStaticPlaneData, R ), array_tid ) < 0 )
    {
      throw std::string{ "Failed to insert R in HDF struct for static planes" };
    }
  }
  // Insert the v type in the struct
  {
    const hsize_t array_dim[]{ 3 };
    const HDFTID array_tid{ H5Tarray_create2( H5T_NATIVE_DOUBLE, 1, array_dim ) };
    if( array_tid < 0 )
    {
      throw std::string{ "Failed to create HDF v type for static planes" };
    }
    if( H5Tinsert( struct_tid, "v", HOFFSET( LocalStaticPlaneData, v ), array_tid ) < 0 )
    {
      throw std::string{ "Failed to insert v in HDF struct for static planes" };
    }
  }
  // Insert the omega type in the struct
  {
    const hsize_t array_dim[]{ 3 };
    const HDFTID array_tid{ H5Tarray_create2( H5T_NATIVE_DOUBLE, 1, array_dim ) };
    if( array_tid < 0 )
    {
      throw std::string{ "Failed to create HDF omega type for static planes" };
    }
    if( H5Tinsert( struct_tid, "omega", HOFFSET( LocalStaticPlaneData, omega ), array_tid ) < 0 )
    {
      throw std::string{ "Failed to insert omega in HDF struct for static planes" };
    }
  }

  // Create an HDF5 dataset
  const HDFDID data_set{ H5Dcreate2( output_file.fileID(), ( group + "/static_planes" ).c_str(), struct_tid, data_space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT ) };
  if( data_set < 0 )
  {
    throw std::string{ "Failed to create HDF dataset for static planes" };
  }

  // Create an HDF5 memspace to allow us to insert elements one by one
  const HDFSID mem_space{ H5Screate_simple( 1, dim, nullptr ) };
  if( mem_space < 0 )
  {
    throw std::string{ "Failed to create HDF memspace for static planes" };
  }

  // Insert the static planes one by one
  unsigned current_plane{ 0 };
  LocalStaticPlaneData local_data;
  for( const StaticPlane& plane : static_planes )
  {
    {
      Eigen::Map<Vector3s>{ local_data.x } = plane.x();
      local_data.R[0] = plane.R().x();
      local_data.R[1] = plane.R().y();
      local_data.R[2] = plane.R().z();
      local_data.R[3] = plane.R().w();
      Eigen::Map<Vector3s>{ local_data.v } = plane.v();
      Eigen::Map<Vector3s>{ local_data.omega } = plane.omega();
    }
    const hsize_t count[]{ 1 };
    const hsize_t offset[]{ current_plane++ };
    const hsize_t mem_offset[]{ 0 };
    H5Sselect_hyperslab( data_space, H5S_SELECT_SET, offset, nullptr, count, nullptr );
    H5Sselect_hyperslab( mem_space, H5S_SELECT_SET, mem_offset, nullptr, count, nullptr );
    if( H5Dwrite( data_set, struct_tid, mem_space, data_space, H5P_DEFAULT, &local_data ) < 0 )
    {
      throw std::string{ "Failed to write static plane struct to HDF" };
    }
  }
  assert( current_plane == static_planes.size() );
  #else
  throw std::string{ "writeStaticPlanes not compiled with HDF5 support" };
  #endif
}

void StateOutput::writeStaticCylinders( const std::vector<StaticCylinder>& static_cylinders, const std::string& group, HDF5File& output_file )
{
  #ifdef USE_HDF5
  struct LocalStaticCylinderData
  {
    scalar x[3];
    scalar R[4];
    scalar v[3];
    scalar omega[3];
    scalar r;
  };

  // Create an HDF5 dataspace
  const hsize_t dim[]{ static_cylinders.size() };
  const HDFSID data_space{ H5Screate_simple( 1, dim, nullptr ) };
  if( data_space < 0 )
  {
    throw std::string{ "Failed to create HDF dataspace for static cylinders" };
  }

  // Create an HDF5 struct for the data
  const HDFTID struct_tid{ H5Tcreate( H5T_COMPOUND, sizeof( LocalStaticCylinderData ) ) };
  if( struct_tid < 0 )
  {
    throw std::string{ "Failed to create HDF struct for static cylinders" };
  }
  // Insert the x type in the struct
  {
    const hsize_t array_dim[]{ 3 };
    const HDFTID array_tid{ H5Tarray_create2( H5T_NATIVE_DOUBLE, 1, array_dim ) };
    if( array_tid < 0 )
    {
      throw std::string{ "Failed to create HDF x type for static cylinders" };
    }
    if( H5Tinsert( struct_tid, "x", HOFFSET( LocalStaticCylinderData, x ), array_tid ) < 0 )
    {
      throw std::string{ "Failed to insert x in HDF struct for static cylinders" };
    }
  }
  // Insert the R type in the struct
  {
    const hsize_t array_dim[]{ 4 };
    const HDFTID array_tid{ H5Tarray_create2( H5T_NATIVE_DOUBLE, 1, array_dim ) };
    if( array_tid < 0 )
    {
      throw std::string{ "Failed to create HDF R type for static cylinders" };
    }
    if( H5Tinsert( struct_tid, "R", HOFFSET( LocalStaticCylinderData, R ), array_tid ) < 0 )
    {
      throw std::string{ "Failed to insert R in HDF struct for static cylinders" };
    }
  }
  // Insert the v type in the struct
  {
    const hsize_t array_dim[]{ 3 };
    const HDFTID array_tid{ H5Tarray_create2( H5T_NATIVE_DOUBLE, 1, array_dim ) };
    if( array_tid < 0 )
    {
      throw std::string{ "Failed to create HDF v type for static cylinders" };
    }
    if( H5Tinsert( struct_tid, "v", HOFFSET( LocalStaticCylinderData, v ), array_tid ) < 0 )
    {
      throw std::string{ "Failed to insert v in HDF struct for static cylinders" };
    }
  }
  // Insert the omega type in the struct
  {
    const hsize_t array_dim[]{ 3 };
    const HDFTID array_tid{ H5Tarray_create2( H5T_NATIVE_DOUBLE, 1, array_dim ) };
    if( array_tid < 0 )
    {
      throw std::string{ "Failed to create HDF omega type for static cylinders" };
    }
    if( H5Tinsert( struct_tid, "omega", HOFFSET( LocalStaticCylinderData, omega ), array_tid ) < 0 )
    {
      throw std::string{ "Failed to insert omega in HDF struct for static cylinders" };
    }
  }
  // Insert the r type in the struct
  if( H5Tinsert( struct_tid, "r", HOFFSET( LocalStaticCylinderData, r ), H5T_NATIVE_DOUBLE ) < 0 )
  {
    throw std::string{ "Failed to create HDF r type for static cylinders" };
  }

  // Create an HDF5 dataset
  const HDFDID data_set{ H5Dcreate2( output_file.fileID(), ( group + "/static_cylinders" ).c_str(), struct_tid, data_space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT ) };
  if( data_set < 0 )
  {
    throw std::string{ "Failed to create HDF dataset for static cylinders" };
  }

  // Create an HDF5 memspace to allow us to insert elements one by one
  const HDFSID mem_space{ H5Screate_simple( 1, dim, nullptr ) };
  if( mem_space < 0 )
  {
    throw std::string{ "Failed to create HDF memspace for static cylinders" };
  }

  // Insert the static cylinders one by one
  unsigned current_cylinder{ 0 };
  LocalStaticCylinderData local_data;
  for( const StaticCylinder& cylinder : static_cylinders )
  {
    {
      Eigen::Map<Vector3s>{ local_data.x } = cylinder.x();
      local_data.R[0] = cylinder.R().x();
      local_data.R[1] = cylinder.R().y();
      local_data.R[2] = cylinder.R().z();
      local_data.R[3] = cylinder.R().w();
      Eigen::Map<Vector3s>{ local_data.v } = cylinder.v();
      Eigen::Map<Vector3s>{ local_data.omega } = cylinder.omega();
      local_data.r = cylinder.r();
    }
    const hsize_t count[]{ 1 };
    const hsize_t offset[]{ current_cylinder++ };
    const hsize_t mem_offset[]{ 0 };
    H5Sselect_hyperslab( data_space, H5S_SELECT_SET, offset, nullptr, count, nullptr );
    H5Sselect_hyperslab( mem_space, H5S_SELECT_SET, mem_offset, nullptr, count, nullptr );
    if( H5Dwrite( data_set, struct_tid, mem_space, data_space, H5P_DEFAULT, &local_data ) < 0 )
    {
      throw std::string{ "Failed to write static cylinder struct to HDF" };
    }
  }
  assert( current_cylinder == static_cylinders.size() );
  #else
  throw std::string{ "writeStaticCylinders not compiled with HDF5 support" };
  #endif
}
