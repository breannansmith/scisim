// TimeUtils.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "TimeUtils.h"

#include <ctime>
#include "SCISim/StringUtilities.h"

std::string TimeUtils::currentTime()
{
  std::time_t result = std::time( nullptr );
  return StringUtilities::trim( std::ctime( &result ) );
}
