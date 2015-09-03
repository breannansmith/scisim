// StringUtilities.cpp
//
// Breannan Smith
// Last updated: 09/02/2015

#include "StringUtilities.h"

#include <cassert>
#include <iomanip>

std::string StringUtilities::deserializeString( std::istream& stm )
{
  assert( stm.good() );
  std::string::size_type len;
  stm.read( reinterpret_cast<char*>( &len ), sizeof(std::string::size_type) );
  std::vector<char> cstr( len );
  stm.read( reinterpret_cast<char*>( &cstr.front() ), len * sizeof(char) );
  return std::string( cstr.begin(), cstr.end() );
}

void StringUtilities::serializeString( const std::string& input_string, std::ostream& stm )
{
  assert( stm.good() );
  std::string::size_type len = input_string.length();
  stm.write( reinterpret_cast<char*>( &len ), sizeof(std::string::size_type) );
  stm.write( const_cast<char*>( reinterpret_cast<const char*>( input_string.c_str() ) ), len * sizeof(char) );
}

void StringUtilities::splitAtLastCharacterOccurence( const std::string& input_string, std::string& left_substring, std::string& right_substring, const char chr )
{
  const std::string::size_type position = input_string.find_last_of( chr );
  left_substring = input_string.substr( 0, position );
  if( position != std::string::npos )
  {
    right_substring = input_string.substr( position + 1 );
  }
  else
  {
    right_substring = "";
  }
}

std::string::size_type StringUtilities::computeNumCharactersToRight( const std::string& input_string, const char chr )
{
  const std::string::size_type position = input_string.find_first_of( chr );
  if( position == std::string::npos )
  {
    return 0;
  }
  return input_string.length() - input_string.find_first_of( chr ) - 1;
}

std::string StringUtilities::trimFront( const std::string& input_string )
{
  std::string output_string = input_string;
  output_string.erase( output_string.begin(), std::find_if( output_string.begin(), output_string.end(), []( const char x ){ return !std::isspace( x ); } ) );
  return output_string;
}

std::string StringUtilities::trimBack( const std::string& input_string )
{
  std::string output_string = input_string;
  output_string.erase( std::find_if( output_string.rbegin(), output_string.rend(), []( const char x ){ return !std::isspace( x ); } ).base(), output_string.end() );
  return output_string;
}

std::string StringUtilities::trim( const std::string& input_string )
{
  return trimFront( trimBack( input_string ) );
}

std::string StringUtilities::removeWhiteSpace( const std::string& input_string )
{
  std::string output_string = input_string;
  output_string.erase( std::remove_if( output_string.begin(), output_string.end(), []( const char x ){ return std::isspace( x ); } ), output_string.end() );
  return output_string;
}

std::string StringUtilities::trimCharacterRight( const std::string& input_string, const char chr )
{
  return input_string.substr( 0, input_string.find_last_not_of( chr ) + 1 );
}

std::string StringUtilities::padWithZeros( const std::string& input_string, const unsigned output_width )
{
  std::stringstream ss;
  ss << std::setfill( '0' ) << std::setw( output_width ) << input_string;
  return ss.str();
}

void StringUtilities::tokenize( const std::string& str, const char chr, std::vector<std::string>& tokens )
{
  std::string::size_type substring_start = 0;
  std::string::size_type substring_end = str.find_first_of( chr, substring_start );
  while( substring_end != std::string::npos )
  {
    tokens.emplace_back( str.substr( substring_start, substring_end - substring_start ) );
    substring_start = substring_end + 1;
    substring_end = str.find_first_of( chr, substring_start );
  }
  // Grab the trailing substring, if present
  if( substring_start < str.size() )
  {
    tokens.emplace_back( str.substr( substring_start ) );
  }
  // Case of final character the delimiter
  if( str.back() == chr )
  {
    tokens.emplace_back( "" );
  }
}

std::vector<std::string> StringUtilities::tokenize( const std::string& str, const char delimiter )
{
  std::vector<std::string> tokens;
  tokenize( str, delimiter, tokens );
  return tokens;
}
