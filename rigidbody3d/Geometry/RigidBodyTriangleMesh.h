// RigidBodyTriangleMesh.h
//
// Breannan Smith
// Last updated: 09/22/2015

#ifndef RIGID_BODY_TRIANGLE_MESH
#define RIGID_BODY_TRIANGLE_MESH

#include "RigidBodyGeometry.h"

class RigidBodyTriangleMesh final : public RigidBodyGeometry
{

public:

  // TODO: Fix up constructors, custom copy not needed

  #ifndef USE_HDF5
  [[noreturn]]
  #endif
  explicit RigidBodyTriangleMesh( const std::string& input_file_name );

  RigidBodyTriangleMesh( const RigidBodyTriangleMesh& other );
  explicit RigidBodyTriangleMesh( std::istream& input_stream );
  virtual ~RigidBodyTriangleMesh() override;

  virtual RigidBodyGeometryType getType() const override;

  virtual std::unique_ptr<RigidBodyGeometry> clone() const override;

  virtual void computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const override;

  virtual void computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const override;

  virtual std::string name() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

  virtual scalar volume() const override;

  const Matrix3Xsc& vertices() const;

  const Matrix3Xuc& faces() const;

  const Matrix3Xsc& convexHullVertices() const;

  const std::string& inputFileName() const;

  const Matrix3s& R() const;
  const Matrix3Xsc& samples() const;

  // N.b: Sample point must be expressed in local frame of this body,
  // and if the point is inside the distance field, the returned normal
  // is expressed in the local frame
  bool detectCollision( const Vector3s& x, Vector3s& n ) const;

private:

  const scalar& v( const unsigned i, const unsigned j, const unsigned k ) const;

  const std::string m_input_file_name;

  // Note: These are not const because I don't have return value version of matrix reading from HDF5, yet
  Matrix3Xsc m_verts;
  Matrix3Xuc m_faces;

  scalar m_volume;
  Vector3s m_I_on_rho;
  Vector3s m_center_of_mass;
  Matrix3s m_R;

  Matrix3Xsc m_samples;
  Matrix3Xsc m_convex_hull_samples;

  Vector3s m_cell_delta;
  Vector3u m_grid_dimensions;
  Vector3s m_grid_origin;
  VectorXs m_signed_distance;
  // Derivable from the above quantities, just stored for convienience
  Vector3s m_grid_end;

};

#endif
