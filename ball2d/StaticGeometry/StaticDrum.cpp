// StaticDrum.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "StaticDrum.h"

#include "SCISim/Math/MathUtilities.h"

// TODO: Serialize/deserialize with the buil in routines, make member vars const

StaticDrum::StaticDrum( const Vector2s& x, const scalar& r )
: m_x( x )
, m_r( r )
{
  assert( m_r > 0.0 );
}

StaticDrum::StaticDrum( std::istream& input_stream )
: m_x()
, m_r()
{
  assert( input_stream.good() );
  mathutils::deserialize( m_x, input_stream );
  input_stream.read( (char*) &m_r, sizeof(scalar) );
  assert( m_r > 0.0 );
}

const Vector2s& StaticDrum::x() const
{
  return m_x;
}

const scalar& StaticDrum::r() const
{
  return m_r;
}

void StaticDrum::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  mathutils::serialize( m_x, output_stream );
  output_stream.write( (char*) &m_r, sizeof(scalar) );
}
