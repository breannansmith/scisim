// StaticDrum.cpp
//
// Breannan Smith
// Last updated: 09/07/2015

#include "StaticDrum.h"

#include "scisim/Math/MathUtilities.h"
#include "scisim/Utilities.h"

StaticDrum::StaticDrum( const Vector2s& x, const scalar& r )
: m_x( x )
, m_r( r )
{
  assert( m_r > 0.0 );
}

StaticDrum::StaticDrum( std::istream& input_stream )
: m_x( MathUtilities::deserialize<Vector2s>( input_stream ) )
, m_r( Utilities::deserialize<scalar>( input_stream ) )
{
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
  MathUtilities::serialize( m_x, output_stream );
  Utilities::serializeBuiltInType( m_r, output_stream );
}
