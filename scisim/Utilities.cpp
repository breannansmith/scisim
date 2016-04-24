// Utilities.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "Utilities.h"

template<>
void Utilities::serialize<bool>( const std::vector<bool>& vector, std::ostream& output_stream )
{
  assert( output_stream.good() );
  serialize( vector.size(), output_stream );
  for( std::vector<bool>::size_type idx = 0; idx < vector.size(); ++idx )
  {
    const bool local_val = vector[idx];
    serialize( local_val, output_stream );
  }
}

template<>
std::vector<bool> Utilities::deserializeVector<bool>( std::istream& input_stream )
{
  assert( input_stream.good() );
  std::vector<bool> vector;
  const std::vector<bool>::size_type length{ deserialize<std::vector<bool>::size_type>( input_stream ) };
  vector.resize( length );
  for( std::vector<bool>::size_type idx = 0; idx < length; ++idx )
  {
    vector[idx] = deserialize<bool>( input_stream );
  }
  return vector;
}
