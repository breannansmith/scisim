// Utilities.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "Utilities.h"

template<> void Utilities::serializeVectorBuiltInType<bool>( const std::vector<bool>& vector, std::ostream& output_stream )
{
  assert( output_stream.good() );
  // Write out the length of the vector
  Utilities::serializeBuiltInType( vector.size(), output_stream );
  assert( output_stream.good() );
  // Output each element of the vector
  for( std::vector<bool>::size_type idx = 0; idx < vector.size(); ++idx )
  {
    const bool local_val = vector[idx];
    Utilities::serializeBuiltInType( local_val, output_stream );
    assert( output_stream.good() );
  }
}
