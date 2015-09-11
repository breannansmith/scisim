// StateOutput.h
//
// Breannan Smith
// Last updated: 09/10/2015

#ifndef RIGID_BODY_2D_STATE_OUTPUT_H
#define RIGID_BODY_2D_STATE_OUTPUT_H

// TODO: Template on float/double type, add cmake option to dump contents as float

#include <vector>
#include <string>
#include <memory>

#include "SCISim/Math/MathDefines.h"

class RigidBody2DGeometry;
class HDF5File;
class RigidBody2DStaticPlane;
class PlanarPortal;

namespace RigidBody2DStateOutput
{

  void writeGeometryIndices( const std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry, const VectorXu& indices, const std::string& group, HDF5File& output_file );

  void writeGeometry( const std::vector<std::unique_ptr<RigidBody2DGeometry>>& geometry, const std::string& group, HDF5File& output_file );

  void writeStaticPlanes( const std::vector<RigidBody2DStaticPlane>& static_planes, const std::string& group, HDF5File& output_file );
  void writePlanarPortals( const std::vector<PlanarPortal>& planar_portals, const std::string& group, HDF5File& output_file );

}

#endif
