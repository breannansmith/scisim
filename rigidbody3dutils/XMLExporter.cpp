#include "XMLExporter.h"

#include <fstream>
#include <iostream>

#include "rigidbody3d/RigidBody3DState.h"
#include "rigidbody3d/Geometry/RigidBodyTriangleMesh.h"
#include "rigidbody3d/Geometry/RigidBodySphere.h"

static Vector3s matrixToAngleAxis( const Matrix33sr& R )
{
  const Eigen::AngleAxis<scalar> R_aa{ R };
  return R_aa.angle() * R_aa.axis();
}

bool XMLExporter::saveToXMLFile( const std::string& file_name, const RigidBody3DState& state )
{
  std::ofstream xml_file( file_name );

  if( !xml_file.is_open() )
  {
    std::cerr << "Error, failed to open file " << file_name << " for output." << std::endl;
    return false;
  }

  xml_file << "<rigidbody3d_scene>" << std::endl;

  // Save the geometry out
  for( const std::unique_ptr<RigidBodyGeometry>& geo : state.geometry() )
  {
    switch( geo->getType() )
    {
      case RigidBodyGeometryType::SPHERE:
      {
        const RigidBodySphere& sphere{ static_cast<RigidBodySphere&>( *geo.get() ) };
        xml_file << "  <geometry type=\"sphere\" r=\"" << sphere.r() << "\"/>" << std::endl;
        break;
      }
      case RigidBodyGeometryType::BOX:
      {
        std::cerr << "Code up box geometry xml output." << std::endl;
        std::exit( EXIT_FAILURE );
      }
      case RigidBodyGeometryType::STAPLE:
      {
        std::cerr << "Code up staple geometry xml output." << std::endl;
        std::exit( EXIT_FAILURE );
      }
      case RigidBodyGeometryType::TRIANGLE_MESH:
      {
        const RigidBodyTriangleMesh& tri_mesh{ static_cast<RigidBodyTriangleMesh&>( *geo.get() ) };
        xml_file << "  <geometry type=\"mesh\" filename=\"" << tri_mesh.inputFileName() << "\"/>" << std::endl;
        break;
      }
    }
  }

  // Save the bodies out
  for( unsigned bdy_idx = 0; bdy_idx < state.nbodies(); bdy_idx++ )
  {
    const Vector3s x{ state.q().segment<3>( 3 * bdy_idx ) };

    const Matrix33sr R{ Eigen::Map<const Matrix33sr>{ state.q().segment<9>( 3 * state.nbodies() + 9 * bdy_idx ).data() } };
    using std::fabs;
    assert( fabs( R.determinant() - 1.0 ) <= 1.0e-9 );
    assert( ( R.transpose() * R - Matrix33sr::Identity() ).lpNorm<Eigen::Infinity>() <= 1.0e-9 );
    const Vector3s R_aa{ matrixToAngleAxis( R ) };

    const Vector3s v{ state.v().segment<3>( 3 * bdy_idx ) };

    const Vector3s omega{ state.v().segment<3>( 3 * state.nbodies() + 3 * bdy_idx ) };

    const scalar density{ state.getTotalMass(bdy_idx) / state.getGeometryOfBody(bdy_idx).volume() };

    const bool is_fixed{ state.isKinematicallyScripted(bdy_idx) };

    const unsigned geo_idx{ state.getGeometryIndexOfBody(bdy_idx) };

    xml_file << "  <rigid_body_with_density x=\"" << x.x() << " " << x.y() << " " << x.z() << "\" R=\"" << R_aa.x() << " " << R_aa.y() << " " << R_aa.z() << "\" v=\"" << v.x() << " " << v.y() << " " << v.z() << "\" omega=\"" << omega.x() << " " << omega.y() << " " << omega.z() << "\" rho=\"" << density << "\" fixed=\"" << is_fixed << "\" geo_idx=\"" << geo_idx << "\"/>" << std::endl;
  }

  xml_file << "</rigidbody3d_scene>" << std::endl;

  return true;
}
