// RigidBodySphere.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "RigidBodySphere.h"

#include "scisim/Utilities.h"

void RigidBodySphere::computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const
{
  M = 4.0 * MathDefines::PI<scalar>() * m_r * m_r * m_r / 3.0;
  I.setConstant( 2.0 * M * m_r * m_r / 5.0 );
  M *= density;
  I *= density;
  CM.setZero();
  R.setIdentity();
}

RigidBodySphere::RigidBodySphere( const scalar& r )
: m_r( r )
{
  assert( m_r > 0.0 );
}

RigidBodySphere::RigidBodySphere( std::istream& input_stream )
: m_r( Utilities::deserialize<scalar>( input_stream ) )
{
  assert( m_r > 0.0 );
}

RigidBodySphere::~RigidBodySphere()
{}

RigidBodyGeometryType RigidBodySphere::getType() const
{
  return RigidBodyGeometryType::SPHERE;
}

std::unique_ptr<RigidBodyGeometry> RigidBodySphere::clone() const
{
  return std::unique_ptr<RigidBodyGeometry>{ new RigidBodySphere{ m_r } };
}

void RigidBodySphere::computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const
{
  min = cm.array() - m_r;
  max = cm.array() + m_r;
}

std::string RigidBodySphere::name() const
{
  return "sphere";
}

void RigidBodySphere::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  Utilities::serialize( RigidBodyGeometryType::SPHERE, output_stream );
  Utilities::serialize( m_r, output_stream );
}

scalar RigidBodySphere::volume() const
{
  return 4.0 * MathDefines::PI<scalar>() * m_r * m_r * m_r / 3.0;
}

const scalar& RigidBodySphere::r() const
{
  return m_r;
}
