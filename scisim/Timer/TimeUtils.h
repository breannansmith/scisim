// TimeUtils.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <iosfwd>

namespace TimeUtils
{

  // Returns current time as: Day Month DayOfMonth Hour:Minute:Second Year.
  //   For example: Fri Jun 21 14:55:24 2013.
  std::string currentTime();

}

#endif
