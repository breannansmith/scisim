// string_utility_tests.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include <iostream>
#include <string>
#include <cstdlib>
#include <algorithm>

#include "scisim/StringUtilities.h"

int main( int argc, char** argv )
{
  if( argc != 2 )
  {
    std::cerr << "Usage: " << argv[0] << " test_name" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string test_name( argv[1] );

  if( test_name == "trim_character_right_00" )
  {
    const std::string input_string{ "123432. fdsai3nfj9000" };
    const std::string trimmed_string{ StringUtilities::trimCharacterRight( input_string, '0' ) };
    if( trimmed_string != "123432. fdsai3nfj9" )
    {
      std::cerr << "Error, input string " << input_string << " improperly trimmed of '0' to give " << trimmed_string << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_character_right_01" )
  {
    const std::string input_string{ "  123456789  " };
    const std::string trimmed_string{ StringUtilities::trimCharacterRight( input_string, '0' ) };
    if( trimmed_string != "  123456789  " )
    {
      std::cerr << "Error, input string " << input_string << " improperly trimmed of '0' to give " << trimmed_string << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_character_right_02" )
  {
    const std::string input_string{ " ccccccc  " };
    const std::string trimmed_string{ StringUtilities::trimCharacterRight( input_string, 'c' ) };
    if( trimmed_string != " ccccccc  " )
    {
      std::cerr << "Error, input string " << input_string << " improperly trimmed of 'c' to give " << trimmed_string << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_character_right_03" )
  {
    const std::string input_string{ "" };
    const std::string trimmed_string{ StringUtilities::trimCharacterRight( input_string, 'm' ) };
    if( trimmed_string != "" )
    {
      std::cerr << "Error, input string " << input_string << " improperly trimmed of 'm' to give " << trimmed_string << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_character_right_04" )
  {
    const std::string input_string{ "AAAAAAAAAAA" };
    const std::string trimmed_string{ StringUtilities::trimCharacterRight( input_string, 'A' ) };
    if( trimmed_string != "" )
    {
      std::cerr << "Error, input string " << input_string << " improperly trimmed of 'A' to give " << trimmed_string << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "remove_whitespace_00" )
  {
    const std::string input_string{ "  1 2 \t \t3\n 4\n\n 5678 \n9    " };
    const std::string trimmed_string{ StringUtilities::removeWhiteSpace( input_string ) };
    if( trimmed_string != "123456789" )
    {
      std::cerr << "Error, input string " << input_string << " improperly removed of whitespace to give '" << trimmed_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "remove_whitespace_01" )
  {
    const std::string input_string{ "    \t \t\n \n\n  \n    " };
    const std::string trimmed_string{ StringUtilities::removeWhiteSpace( input_string ) };
    if( trimmed_string != "" )
    {
      std::cerr << "Error, input string " << input_string << " improperly removed of whitespace to give " << trimmed_string << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "remove_whitespace_02" )
  {
    const std::string input_string{ "" };
    const std::string trimmed_string{ StringUtilities::removeWhiteSpace( input_string ) };
    if( trimmed_string != "" )
    {
      std::cerr << "Error, input string " << input_string << " improperly removed of whitespace to give " << trimmed_string << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "zero_pad_00" )
  {
    const std::string input_string{ "test" };
    const std::string output_string{ StringUtilities::padWithZeros( input_string, 8 ) };
    if( output_string != "0000test" )
    {
      std::cerr << "Error, input string " << input_string << " incorrectly padded to width 8 giving " << output_string << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "zero_pad_01" )
  {
    const std::string input_string{ "testtest" };
    const std::string output_string{ StringUtilities::padWithZeros( input_string, 7 ) };
    if( output_string != "testtest" )
    {
      std::cerr << "Error, input string " << input_string << " incorrectly padded to width 7 giving " << output_string << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "split_test_00" )
  {
    const std::string input_string{ "left123-right123" };
    std::string left_substring;
    std::string right_substring;
    StringUtilities::splitAtLastCharacterOccurence( input_string, left_substring, right_substring, '-' );
    if( left_substring != "left123" || right_substring != "right123" )
    {
      std::cerr << "Error, incorrectly split string " << input_string << " at character '-' into ";
      std::cerr << left_substring << " and " << right_substring << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "split_test_01" )
  {
    const std::string input_string{ "abcdefghixjk" };
    std::string left_substring;
    std::string right_substring;
    StringUtilities::splitAtLastCharacterOccurence( input_string, left_substring, right_substring, 'x' );
    if( left_substring != "abcdefghi" || right_substring != "jk" )
    {
      std::cerr << "Error, incorrectly split string " << input_string << " at character 'x' into ";
      std::cerr << left_substring << " and " << right_substring << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "split_test_02" )
  {
    const std::string input_string{ "123456789" };
    std::string left_substring;
    std::string right_substring;
    StringUtilities::splitAtLastCharacterOccurence( input_string, left_substring, right_substring, 'x' );
    if( left_substring != "123456789" || right_substring != "" )
    {
      std::cerr << "Error, incorrectly split string " << input_string << " at character 'x' into ";
      std::cerr << left_substring << " and " << right_substring << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "split_test_03" )
  {
    const std::string input_string{ "9876/" };
    std::string left_substring;
    std::string right_substring;
    StringUtilities::splitAtLastCharacterOccurence( input_string, left_substring, right_substring, '/' );
    if( left_substring != "9876" || right_substring != "" )
    {
      std::cerr << "Error, incorrectly split string " << input_string << " at character '/' into ";
      std::cerr << left_substring << " and " << right_substring << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "split_test_04" )
  {
    const std::string input_string{ "/9876" };
    std::string left_substring;
    std::string right_substring;
    StringUtilities::splitAtLastCharacterOccurence( input_string, left_substring, right_substring, '/' );
    if( left_substring != "" || right_substring != "9876" )
    {
      std::cerr << "Error, incorrectly split string " << input_string << " at character '/' into ";
      std::cerr << left_substring << " and " << right_substring << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "split_test_05" )
  {
    const std::string input_string{ "/" };
    std::string left_substring;
    std::string right_substring;
    StringUtilities::splitAtLastCharacterOccurence( input_string, left_substring, right_substring, '/' );
    if( left_substring != "" || right_substring != "" )
    {
      std::cerr << "Error, incorrectly split string " << input_string << " at character '/' into ";
      std::cerr << left_substring << " and " << right_substring << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "characters_to_right_00" )
  {
    const std::string input_string{ "abdk235k" };
    const std::string::size_type num_to_right{ StringUtilities::computeNumCharactersToRight( input_string, '2' ) };
    if( num_to_right != 3 )
    {
      std::cerr << "Error, computed wrong number of characters to right of '2' in " << input_string;
      std::cerr << " of " << num_to_right << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "characters_to_right_01" )
  {
    const std::string input_string{ "abdk235k" };
    const std::string::size_type num_to_right{ StringUtilities::computeNumCharactersToRight( input_string, '2' ) };
    if( num_to_right != 3 )
    {
      std::cerr << "Error, computed wrong number of characters to right of '2' in " << input_string;
      std::cerr << " of " << num_to_right << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "characters_to_right_02" )
  {
    const std::string input_string{ "_abdfikl43" };
    const std::string::size_type num_to_right{ StringUtilities::computeNumCharactersToRight( input_string, '_' ) };
    if( num_to_right != 9 )
    {
      std::cerr << "Error, computed wrong number of characters to right of '_' in " << input_string;
      std::cerr << " of " << num_to_right << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "characters_to_right_03" )
  {
    const std::string input_string{ "_abdfikl43" };
    const std::string::size_type num_to_right{ StringUtilities::computeNumCharactersToRight( input_string, '3' ) };
    if( num_to_right != 0 )
    {
      std::cerr << "Error, computed wrong number of characters to right of '3' in " << input_string;
      std::cerr << " of " << num_to_right << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "characters_to_right_04" )
  {
    const std::string input_string{ "_abdfi_kl4_3__" };
    const std::string::size_type num_to_right{ StringUtilities::computeNumCharactersToRight( input_string, '_' ) };
    if( num_to_right != 13 )
    {
      std::cerr << "Error, computed wrong number of characters to right of '_' in " << input_string;
      std::cerr << " of " << num_to_right << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "characters_to_right_05" )
  {
    const std::string input_string{ "_abdfi_kli4_3__" };
    const std::string::size_type num_to_right{ StringUtilities::computeNumCharactersToRight( input_string, 'i' ) };
    if( num_to_right != 9 )
    {
      std::cerr << "Error, computed wrong number of characters to right of 'i' in " << input_string;
      std::cerr << " of " << num_to_right << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "characters_to_right_06" )
  {
    const std::string input_string{ "_abdfi_kli4_3__" };
    const std::string::size_type num_to_right{ StringUtilities::computeNumCharactersToRight( input_string, 'c' ) };
    if( num_to_right != 0 )
    {
      std::cerr << "Error, computed wrong number of characters to right of 'c' in " << input_string;
      std::cerr << " of " << num_to_right << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_left_00" )
  {
    const std::string input_string{ "\t\n  \t\t \n   \t12345" };
    const std::string output_string{ StringUtilities::trimFront( input_string ) };
    if( output_string != "12345" )
    {
      std::cerr << "Error, incorrectly trimmed left whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_left_01" )
  {
    const std::string input_string{ "1 2 3 4\t5\t" };
    const std::string output_string{ StringUtilities::trimFront( input_string ) };
    if( output_string != "1 2 3 4\t5\t" )
    {
      std::cerr << "Error, incorrectly trimmed left whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_left_02" )
  {
    const std::string input_string{ "" };
    const std::string output_string{ StringUtilities::trimFront( input_string ) };
    if( output_string != "" )
    {
      std::cerr << "Error, incorrectly trimmed left whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_right_00" )
  {
    const std::string input_string{ "12345\t\n  \t\t \n   \t" };
    const std::string output_string{ StringUtilities::trimBack( input_string ) };
    if( output_string != "12345" )
    {
      std::cerr << "Error, incorrectly trimmed right whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_right_01" )
  {
    const std::string input_string{ "\t1 2 3 4\t5" };
    const std::string output_string{ StringUtilities::trimBack( input_string ) };
    if( output_string != "\t1 2 3 4\t5" )
    {
      std::cerr << "Error, incorrectly trimmed right whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_right_02" )
  {
    const std::string input_string{ "" };
    const std::string output_string{ StringUtilities::trimBack( input_string ) };
    if( output_string != "" )
    {
      std::cerr << "Error, incorrectly trimmed right whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_00" )
  {
    const std::string input_string{ "\t\n  \t1 2 3\t4 5\t\n  \t\t \n   \t" };
    const std::string output_string{ StringUtilities::trim( input_string ) };
    if( output_string != "1 2 3\t4 5" )
    {
      std::cerr << "Error, incorrectly trimmed whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_01" )
  {
    const std::string input_string{ "1 2 3\t4 5\t\n  \t\t \n   \t" };
    const std::string output_string{ StringUtilities::trim( input_string ) };
    if( output_string != "1 2 3\t4 5" )
    {
      std::cerr << "Error, incorrectly trimmed whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_02" )
  {
    const std::string input_string{ "\t\n  \t1 2 3\t4 5" };
    const std::string output_string{ StringUtilities::trim( input_string ) };
    if( output_string != "1 2 3\t4 5" )
    {
      std::cerr << "Error, incorrectly trimmed whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_03" )
  {
    const std::string input_string{ "abc" };
    const std::string output_string{ StringUtilities::trim( input_string ) };
    if( output_string != "abc" )
    {
      std::cerr << "Error, incorrectly trimmed whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "trim_04" )
  {
    const std::string input_string{ "" };
    const std::string output_string{ StringUtilities::trim( input_string ) };
    if( output_string != "" )
    {
      std::cerr << "Error, incorrectly trimmed whitespace to give '" << output_string << "'" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "tokenize_00" )
  {
    const std::string input_string{ "12345abcdefg" };
    const char delimiter{ ' ' };
    std::vector<std::string> solution;
    StringUtilities::tokenize( input_string, delimiter, solution );
    if( solution.size() != 1 )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to " << solution.size() << " tokens." << std::endl;
      return EXIT_FAILURE;
    }
    if( std::vector<std::string>{ "12345abcdefg" } != solution )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to: ";
      std::for_each( solution.begin(), solution.end(), []( const std::string& str){ std::cerr << "'" << str << "' "; } );
      std::cerr << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "tokenize_01" )
  {
    const std::string input_string{ "/12345abcdefg" };
    const char delimiter{ '/' };
    std::vector<std::string> solution;
    StringUtilities::tokenize( input_string, delimiter, solution );
    if( solution.size() != 2 )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to " << solution.size() << " tokens." << std::endl;
      return EXIT_FAILURE;
    }
    if( std::vector<std::string>{ "", "12345abcdefg" } != solution )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to: ";
      std::for_each( solution.begin(), solution.end(), []( const std::string& str){ std::cerr << "'" << str << "' "; } );
      std::cerr << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "tokenize_02" )
  {
    const std::string input_string{ "/12345abcdefg/" };
    const char delimiter{ '/' };
    std::vector<std::string> solution;
    StringUtilities::tokenize( input_string, delimiter, solution );
    if( solution.size() != 3 )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to " << solution.size() << " tokens." << std::endl;
      return EXIT_FAILURE;
    }
    if( std::vector<std::string>{ "", "12345abcdefg", "" } != solution )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to: ";
      std::for_each( solution.begin(), solution.end(), []( const std::string& str){ std::cerr << "'" << str << "' "; } );
      std::cerr << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "tokenize_03" )
  {
    const std::string input_string{ "/123/adfkdi//bghdkdk" };
    const char delimiter{ '/' };
    std::vector<std::string> solution;
    StringUtilities::tokenize( input_string, delimiter, solution );
    if( solution.size() != 5 )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to " << solution.size() << " tokens." << std::endl;
      return EXIT_FAILURE;
    }
    if( std::vector<std::string>{ "", "123", "adfkdi", "", "bghdkdk" } != solution )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to: ";
      std::for_each( solution.begin(), solution.end(), []( const std::string& str){ std::cerr << "'" << str << "' "; } );
      std::cerr << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "tokenize_04" )
  {
    const std::string input_string{ "/123/adfkdi//bghdkdk/" };
    const char delimiter{ '/' };
    std::vector<std::string> solution;
    StringUtilities::tokenize( input_string, delimiter, solution );
    if( solution.size() != 6 )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to " << solution.size() << " tokens." << std::endl;
      return EXIT_FAILURE;
    }
    if( std::vector<std::string>{ "", "123", "adfkdi", "", "bghdkdk", "" } != solution )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to: ";
      std::for_each( solution.begin(), solution.end(), []( const std::string& str){ std::cerr << "'" << str << "' "; } );
      std::cerr << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "tokenize_05" )
  {
    const std::string input_string{ "----------" };
    const char delimiter{ '-' };
    std::vector<std::string> solution;
    StringUtilities::tokenize( input_string, delimiter, solution );
    if( solution.size() != 11 )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to " << solution.size() << " tokens." << std::endl;
      return EXIT_FAILURE;
    }
    if( std::vector<std::string>{ "", "", "", "", "", "", "", "", "", "", "" } != solution )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to: ";
      std::for_each( solution.begin(), solution.end(), []( const std::string& str){ std::cerr << "'" << str << "' "; } );
      std::cerr << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "tokenize_06" )
  {
    const std::string input_string{ "" };
    const char delimiter{ 'a' };
    std::vector<std::string> solution;
    StringUtilities::tokenize( input_string, delimiter, solution );
    if( solution.size() != 0 )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to " << solution.size() << " tokens." << std::endl;
      return EXIT_FAILURE;
    }
    if( std::vector<std::string>{} != solution )
    {
      std::cerr << "Error, incorrectly tokenized " << input_string << " at '" << delimiter << "' to: ";
      std::for_each( solution.begin(), solution.end(), []( const std::string& str){ std::cerr << "'" << str << "' "; } );
      std::cerr << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "extract_from_string_00" )
  {
    int extraction_result{ -1 };
    const bool extraction_succeeded{ StringUtilities::extractFromString( "5", extraction_result ) };
    if( !extraction_succeeded || extraction_result != 5 )
    {
      std::cerr << "Failed to extract integer 5 from input string \"5\"" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "extract_from_string_01" )
  {
    int extraction_result{ -1 };
    const bool extraction_succeeded{ StringUtilities::extractFromString( "x", extraction_result ) };
    if( extraction_succeeded || extraction_result != 0 )
    {
      std::cerr << "Failed to not extract integer from input string \"x\"" << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }

  std::cerr << "Invalid test specified: " << test_name << std::endl;
  return EXIT_FAILURE;
}
