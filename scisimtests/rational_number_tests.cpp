// rational_number_tests.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>

#include "SCISim/Math/MathDefines.h"
#include "SCISim/Math/Rational.h"

int main( int argc, char** argv )
{
  if( argc != 2 )
  {
    std::cerr << "Usage: " << argv[0] << " test_name" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string test_name( argv[1] );

  if( test_name == "scalar_00" )
  {
    Rational<int> rational_number;
    if( !extractFromString( "- 51.2345", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from scalar." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( -51.2345 - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing scalar." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "scalar_01" )
  {
    Rational<int> rational_number;
    if( !extractFromString( ".92345", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from scalar." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( .92345 - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing scalar." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "scalar_02" )
  {
    Rational<int> rational_number;
    if( !extractFromString( " + 8675309.", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from scalar." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( 8675309. - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing scalar." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "scalar_03" )
  {
    Rational<int> rational_number;
    if( !extractFromString( " - . ", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from scalar." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( 0.0 - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing scalar." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "scalar_04" )
  {
    Rational<long> rational_number;
    if( !extractFromString( "1.0000", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from scalar." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( 1.0 - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing scalar." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "scalar_05" )
  {
    Rational<long> rational_number;
    if( !extractFromString( "0.00010", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from scalar." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( 0.00010 - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing scalar." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "integer_00" )
  {
    Rational<int> rational_number;
    if( !extractFromString( "   5678", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from integer." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( 5678.0 - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing integer." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "integer_01" )
  {
    Rational<int> rational_number;
    if( !extractFromString( " - 1234  ", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from integer." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( -1234.0 - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing integer." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "rational_00" )
  {
    Rational<int> rational_number;
    if( !extractFromString( "13/5", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from rational." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( 13.0 / 5.0 - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing rational." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "rational_01" )
  {
    Rational<int> rational_number;
    if( !extractFromString( " - 0 /  1234  ", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from rational." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( 0.0 - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing rational." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "rational_02" )
  {
    Rational<int> rational_number;
    if( !extractFromString( "   + 120 / 60  ", rational_number ) )
    {
      std::cerr << "Failed to parse rational number from rational." << std::endl;
      return EXIT_FAILURE;
    }
    if( fabs( 2.0 - scalar( rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from parsing rational." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "int_multiply_00" )
  {
    const Rational<int> rational_number{ 1, 2 };
    const Rational<int> new_rational_number{ rational_number * 6 };
    if( fabs( 3.0 - scalar( new_rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from multiplication test." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "int_multiply_01" )
  {
    const Rational<int> rational_number{ 1, 2 };
    const Rational<int> new_rational_number{ 9 * rational_number };
    if( fabs( 4.5 - scalar( new_rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from multiplication test." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "int_divide_00" )
  {
    const Rational<int> rational_number{ 1, 3 };
    const Rational<int> new_rational_number{ rational_number / 2 };
    if( fabs( 1.0 / 6.0 - scalar( new_rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from division test." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "int_divide_01" )
  {
    const Rational<int> rational_number{ 3, 5 };
    const Rational<int> new_rational_number{ 2 / rational_number };
    if( fabs( 10.0 / 3.0 - scalar( new_rational_number ) ) > 0 )
    {
      std::cerr << "Obtained incorrect result from division test." << std::endl;
      return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
  }
  else if( test_name == "equal_00" )
  {
    const Rational<int> rational_number_a{ 0, 1 };
    const Rational<int> rational_number_b{ 0, 123 };
    if( rational_number_a == rational_number_b )
    {
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }
  else if( test_name == "equal_01" )
  {
    const Rational<int> rational_number_a{ 15, 17 };
    const Rational<int> rational_number_b{ 45, 51 };
    if( rational_number_a == rational_number_b )
    {
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }
  else if( test_name == "equal_02" )
  {
    const Rational<int> rational_number_a{ 18, 2 };
    const int integer_number_a{ 9 };
    if( rational_number_a == integer_number_a )
    {
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }
  else if( test_name == "equal_03" )
  {
    const Rational<int> rational_number_a{ 18, 2 };
    const int integer_number_a{ 9 };
    if( integer_number_a == rational_number_a )
    {
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }
  else if( test_name == "not_equal_00" )
  {
    const Rational<int> rational_number_a{ 0, 10 };
    const Rational<int> rational_number_b{ 2, 3 };
    if( rational_number_a != rational_number_b )
    {
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }
  else if( test_name == "not_equal_01" )
  {
    const Rational<int> rational_number_a{ 15, 17 };
    const Rational<int> rational_number_b{ 46, 51 };
    if( rational_number_a != rational_number_b )
    {
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }


  std::cerr << "Invalid test specified: " << test_name << std::endl;
  return EXIT_FAILURE;
}
