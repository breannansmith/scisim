// StringUtilities.h
//
// Breannan Smith
// Last updated: 09/02/2015

// TODO: Update serializatoin to use Utilities

#ifndef STRING_UTILITIES_H
#define STRING_UTILITIES_H

#include <string>
#include <vector>
#include <sstream>
#include <limits>

namespace Eigen { template <typename Derived> class DenseBase; }

namespace StringUtilities
{

  std::string deserializeString( std::istream& stm );
  void serializeString( const std::string& input_string, std::ostream& stm );

  // Splits a string at the final occurence of the given character
  void splitAtLastCharacterOccurence( const std::string& input_string, std::string& left_substring, std::string& right_substring, const char chr );
  
  // Computes the number of characters to the right of the first occurence of a given character in the input string
  std::string::size_type computeNumCharactersToRight( const std::string& input_string, const char chr );

  // Trims whitespace from the front of a string
  std::string trimFront( const std::string& input_string );

  // Trims whitespace from the back of a string
  std::string trimBack( const std::string& input_string );

  // Trims whitespace from the front and back of a string
  std::string trim( const std::string& input_string );

  // Removes whitespace entirely from a string
  std::string removeWhiteSpace( const std::string& input_string );

  // Trims a given character from the end of a string
  std::string trimCharacterRight( const std::string& input_string, const char chr );

  std::string padWithZeros( const std::string& input_string, const unsigned output_width );

  // Splits a string at given delimiter character
  void tokenize( const std::string& str, const char chr, std::vector<std::string>& tokens );
  std::vector<std::string> tokenize( const std::string& str, const char delimiter );

  template<class T>
  std::string convertToString( const T& tostring )
  {
    std::string out_string;
    std::stringstream ss;
    ss << tostring;
    ss >> out_string;
    return out_string;
  }

  template<class T>
  bool extractFromString( const std::string& in_string, T& output )
  {
    std::stringstream input_stream( in_string );
    input_stream >> output;
    return !input_stream.fail();
  }

  template<class T>
  bool extractScalarFromString( const std::string& in_string, T& output )
  {
    std::stringstream input_stream( in_string );
    input_stream >> output;
    if( !input_stream.fail() )
    {
      return true;
    }
    if( in_string == "-inf" )
    {
      output = -std::numeric_limits<T>::infinity();
      return true;
    }
    else if( in_string == "inf" )
    {
      output = std::numeric_limits<T>::infinity();
      return true;
    }
    return false;
  }

  // Reads a set number of scalars into an Eigen vector
  template<typename Derived>
  bool readScalarList( const std::string& input_text, const unsigned num_scalars, const char delimiter, Eigen::DenseBase<Derived>& list )
  {
    const std::vector<std::string> split_input = tokenize( input_text, delimiter );
    if( split_input.size() != num_scalars )
    {
      return false;
    }
    list.derived().resize( num_scalars );
    for( unsigned entry_number = 0; entry_number < num_scalars; ++entry_number )
    {
      if( !extractScalarFromString( split_input[entry_number], list(entry_number) ) )
      {
        return false;
      }
    }
    return true;
  }

}

#endif
