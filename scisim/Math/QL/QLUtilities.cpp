// QLUtilities.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "QLUtilities.h"

#include <iostream>
#include "SCISim/StringUtilities.h"

std::string QLUtilities::QLReturnStatusToString( const int status )
{
  if( 0 == status )
  {
    return "The optimality conditions are satisfied";
  }
  else if( 1 == status )
  {
    return "The algorithm has been stopped after too many MAXIT iterations (40*(N+M)";
  }
  else if( 2 == status )
  {
    return "Termination accuracy insufficient to satisfy convergence criterion";
  }
  else if( 3 == status )
  {
    return "Internal inconsistency of QL, division by zero";
  }
  else if( 5 == status )
  {
    return "Length of a working array is too short";
  }
  else if( 100 < status )
  {
    return "Constraints are inconsistent and IFAIL=100+ICON, where ICON denotes a constraint causing the conflict: " + StringUtilities::convertToString( status );
  }

  std::cerr << "Unhandled error message in LCPOperatorQL::QLReturnStatusToString. Probably a bug." << std::endl;
  std::exit( EXIT_FAILURE );
}
