// CompileDefinitions.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef COMPILE_DEFINITIONS_H
#define COMPILE_DEFINITIONS_H

#include <string>

namespace CompileDefinitions
{

  // SHA1 hash of the Git commit for this build
  extern const std::string GitSHA1;

  // Build mode (Debug, Release, etc)
  extern const std::string BuildMode;

  // C compiler used for this build
  extern const std::string CCompiler;

  // C++ compiler used for this build
  extern const std::string CXXCompiler;

  // Fortran compiler used for this build
  extern const std::string FortranCompiler;

};

#endif
