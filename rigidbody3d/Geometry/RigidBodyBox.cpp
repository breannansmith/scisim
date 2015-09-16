// RigidBodyBox.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "RigidBodyBox.h"

#include "SCISim/Math/MathUtilities.h"
#include "SCISim/Utilities.h"

#include <iostream>

void RigidBodyBox::computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const
{
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
  min.setConstant(  std::numeric_limits<scalar>::infinity() );
  max.setConstant( -std::numeric_limits<scalar>::infinity() );

  // For each vertex of a unit-half-width cube
  for( signed char i = -1; i <= 1; i += 2 )
  {
    for( signed char j = -1; j <= 1; j += 2 )
    {
      for( signed char k = -1; k <= 1; k += 2 )
      {
        const Array3s transformed_vertex{ R * ( m_half_widths.array() * Array3s{ scalar(i), scalar(j), scalar(k) } ).matrix() + cm };
        min = min.min( transformed_vertex );
        max = max.max( transformed_vertex );
      }
    }
  }

  assert( ( min < max ).all() );
}

void RigidBodyBox::computeInertia( const scalar& M, Vector3s& I ) const
{
  I( 0 ) = m_half_widths.y() * m_half_widths.y() + m_half_widths.z() * m_half_widths.z();
  I( 1 ) = m_half_widths.x() * m_half_widths.x() + m_half_widths.z() * m_half_widths.z();
  I( 2 ) = m_half_widths.x() * m_half_widths.x() + m_half_widths.y() * m_half_widths.y();
  I *= M / 3.0;
}

std::string RigidBodyBox::name() const
{
  return "box";
}

void RigidBodyBox::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  Utilities::serializeBuiltInType( RigidBodyGeometryType::BOX, output_stream );
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
