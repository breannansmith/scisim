// RigidBodyBox.cpp
//
// Breannan Smith
// Last updated: 01/05/2016

#include "RigidBodyBox.h"

#include "scisim/Math/MathUtilities.h"
#include "scisim/Utilities.h"

void RigidBodyBox::computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const
{
  assert( density > 0.0 );
  M = density * volume();
  computeInertia( M, I );
  CM.setZero();
  R.setIdentity();
}

RigidBodyBox::RigidBodyBox( std::istream& input_stream )
: m_half_widths( MathUtilities::deserialize<Vector3s>( input_stream ) )
{
  assert( ( m_half_widths.array() > 0.0 ).all() );
}

RigidBodyBox::RigidBodyBox( const Vector3s& half_widths )
: m_half_widths( half_widths )
{
  assert( ( m_half_widths.array() > 0.0 ).all() );
}

RigidBodyBox::~RigidBodyBox()
{}

RigidBodyGeometryType RigidBodyBox::getType() const
{
  return RigidBodyGeometryType::BOX;
}

std::unique_ptr<RigidBodyGeometry> RigidBodyBox::clone() const
{
  return std::unique_ptr<RigidBodyGeometry>{ new RigidBodyBox{ m_half_widths } };
}

void RigidBodyBox::computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const
{
  const Array3s extents{ R.array().abs().matrix() * m_half_widths };
  min = cm.array() - extents;
  max = cm.array() + extents;
  assert( ( min < max ).all() );
}

void RigidBodyBox::computeInertia( const scalar& M, Vector3s& I ) const
{
  I.x() = m_half_widths.y() * m_half_widths.y() + m_half_widths.z() * m_half_widths.z();
  I.y() = m_half_widths.x() * m_half_widths.x() + m_half_widths.z() * m_half_widths.z();
  I.z() = m_half_widths.x() * m_half_widths.x() + m_half_widths.y() * m_half_widths.y();
  I *= M / 3.0;
}

std::string RigidBodyBox::name() const
{
  return "box";
}

void RigidBodyBox::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  Utilities::serialize( RigidBodyGeometryType::BOX, output_stream );
  MathUtilities::serialize( m_half_widths, output_stream );
}

scalar RigidBodyBox::volume() const
{
  return 8.0 * m_half_widths.x() * m_half_widths.y() * m_half_widths.z();
}

const Vector3s& RigidBodyBox::halfWidths() const
{
  return m_half_widths;
}
