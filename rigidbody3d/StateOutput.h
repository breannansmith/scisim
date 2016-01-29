// StateOutput.h
//
// Breannan Smith
// Last updated: 10/01/2015

#ifndef STATE_OUTPUT_H
#define STATE_OUTPUT_H

// TODO: Template on float/double type, add cmake option to dump contents as different numerical type than the type used internally

#include <vector>
#include <iosfwd>
#include <memory>

class RigidBodyGeometry;
class HDF5File;
class StaticPlane;
class StaticCylinder;

namespace StateOutput
{

  #ifndef USE_HDF5
  [[noreturn]]
  #endif
  void writeGeometryIndices( const std::vector<std::unique_ptr<RigidBodyGeometry>>& geometry, const std::vector<unsigned>& indices, const std::string& group, HDF5File& output_file );

  void writeGeometry( const std::vector<std::unique_ptr<RigidBodyGeometry>>& geometry, const std::string& group, HDF5File& output_file );

  #ifndef USE_HDF5
  [[noreturn]]
  #endif
  void writeStaticPlanes( const std::vector<StaticPlane>& static_planes, const std::string& group, HDF5File& output_file );

  #ifndef USE_HDF5
  [[noreturn]]
  #endif
  void writeStaticCylinders( const std::vector<StaticCylinder>& static_cylinders, const std::string& group, HDF5File& output_file );

}

#endif
