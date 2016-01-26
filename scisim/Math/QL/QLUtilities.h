// QLUtilities.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef QL_UTILITIES_H
#define QL_UTILITIES_H

#include <iosfwd>

extern "C"
{
  void ql_( int* m, int* me, int* mmax,
            int* n, int* nmax,
            int* mnn,
            double* c, double* d,
            double* a, double* b,
            double* xl, double* xu,
            double* x, double* u,
            double* eps,
            int* mode, int* iout, int* ifail, int* iprint,
            double* war, int* lwar,
            int* iwar, int* liwar );
}

namespace QLUtilities
{
  std::string QLReturnStatusToString( const int status );
}

#endif
