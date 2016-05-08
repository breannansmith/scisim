// HDF5File.cpp
//
// Breannan Smith
// Last updated: 09/28/2015

#include "HDF5File.h"

#include <cassert>

using HDFGID = HDFID<H5Gclose>;
using HDFTID = HDFID<H5Tclose>;
using HDFSID = HDFID<H5Sclose>;
using HDFDID = HDFID<H5Dclose>;

HDF5File::HDF5File()
: m_hdf_file_id( -1 )
{}

HDF5File::HDF5File( const std::string& file_name, const HDF5AccessType& access_type )
: m_hdf_file_id( -1 )
{
  open( file_name, access_type );
}

HDF5File::~HDF5File()
{
  if( m_hdf_file_id >= 0 )
  {
    H5Fclose( m_hdf_file_id );
  }
}

HDF5File::HDF5File( HDF5File&& other )
: m_hdf_file_id( other.m_hdf_file_id )
{
  other.m_hdf_file_id = -1;
}

HDF5File& HDF5File::operator=( HDF5File&& other )
{
  using std::swap;
  swap( other.m_hdf_file_id, m_hdf_file_id );
  return *this;
}

hid_t HDF5File::fileID()
{
  return m_hdf_file_id;
}

void HDF5File::open( const std::string& file_name, const HDF5AccessType& access_type )
{
  // Attempt to open a file
  switch( access_type )
  {
    case HDF5AccessType::READ_WRITE:
      m_hdf_file_id = H5Fcreate( file_name.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT );
      break;
    case HDF5AccessType::READ_ONLY:
      m_hdf_file_id = H5Fopen( file_name.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT );
      break;
  }
  // Check that the file successfully opened
  if( m_hdf_file_id < 0 )
  {
    throw std::string{ "Failed to open file: " } + file_name;
  }
}

bool HDF5File::is_open() const
{
  return m_hdf_file_id >= 0;
}

void HDF5File::write( const std::string& full_name, const std::string& string_variable ) const
{
  const auto split_name = splitFullName( full_name );

  // HDF5 expects an array of strings
  const char* string_data[1] = { string_variable.c_str() };
  const HDFTID file_type{ H5Tcopy( H5T_FORTRAN_S1 ) };
  if( file_type < 0 )
  {
    throw std::string{ "Failed to create HDF5 string file type" };
  }
  const herr_t set_file_size_status{ H5Tset_size( file_type, H5T_VARIABLE ) };
  if( set_file_size_status < 0 )
  {
    throw std::string{ "Failed to set HDF5 string file size" };
  }
  const HDFTID mem_type{ H5Tcopy( H5T_C_S1 ) };
  if( mem_type < 0 )
  {
    throw std::string{ "Failed to create HDF5 string memory type" };
  }
  const herr_t set_memory_size_status{ H5Tset_size( mem_type, H5T_VARIABLE ) };
  if( set_memory_size_status < 0 )
  {
    throw std::string{ "Failed to set HDF5 string memory size" };
  }
  const hsize_t dims[1] = { 1 };
  const HDFSID space{ H5Screate_simple( 1, dims, nullptr ) };
  if( space < 0 )
  {
    throw std::string{ "Failed to create HDF space" };
  }
  assert( m_hdf_file_id >= 0 );

  // Open the requested group
  const HDFGID grp_id{ findOrCreateGroup( split_name.first ) };

  const HDFDID dset{ H5Dcreate2( grp_id, split_name.second.c_str(), file_type, space, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT ) };
  if( dset < 0 )
  {
    throw std::string{ "Failed to create HDF dataset" };
  }
  const herr_t write_status{ H5Dwrite( dset, mem_type, H5S_ALL, H5S_ALL, H5P_DEFAULT, string_data ) };
  if( write_status < 0 )
  {
    throw std::string{ "Failed to write HDF data" };
  }
}

HDFID<H5Gclose> HDF5File::findOrCreateGroup( const std::string& group_name ) const
{
  HDFGID group_id;
  if( group_name.empty() )
  {
    group_id = HDFGID{ H5Gopen2( m_hdf_file_id, "/", H5P_DEFAULT ) };
  }
  else if( 0 == H5Lexists( m_hdf_file_id, group_name.c_str(), H5P_DEFAULT ) )
  {
    group_id = HDFGID{ H5Gcreate2( m_hdf_file_id, group_name.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT ) };
  }
  else
  {
    group_id = HDFGID{ H5Gopen2( m_hdf_file_id, group_name.c_str(), H5P_DEFAULT ) };
  }
  if( group_id < 0 )
  {
    throw std::string{ "Failed to create group: " } + group_name;
  }
  return group_id;
}

HDFID<H5Gclose> HDF5File::findGroup( const std::string& group_name ) const
{
  HDFGID group_id;
  if( group_name.empty() )
  {
    group_id = HDFGID{ H5Gopen2( m_hdf_file_id, "/", H5P_DEFAULT ) };
  }
  else
  {
    group_id = HDFGID{ H5Gopen2( m_hdf_file_id, group_name.c_str(), H5P_DEFAULT ) };
  }
  if( group_id < 0 )
  {
    throw std::string{ "Failed to find group: " } + group_name;
  }
  return group_id;
}
