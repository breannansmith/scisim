// IpoptUtilities.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "IpoptUtilities.h"

#include <string>

bool IpoptUtilities::linearSolverSupported( const std::string& linear_solver_name )
{
  if( linear_solver_name != "ma27" && linear_solver_name != "ma57" && linear_solver_name != "mumps" && linear_solver_name != "ma86" && linear_solver_name != "ma97" )
  {
    return false;
  }
  return true;
}

bool IpoptUtilities::containsDuplicates( const std::vector<std::string>& linear_solvers )
{
  for( std::vector<std::string>::size_type idx0 = 0; idx0 < linear_solvers.size(); ++idx0 )
  {
    for( std::vector<std::string>::size_type idx1 = idx0 + 1; idx1 < linear_solvers.size(); ++idx1 )
    {
      if( linear_solvers[idx0] == linear_solvers[idx1] )
      {
        return true;
      }
    }
  }
  return false;
}

std::string IpoptUtilities::ipoptReturnStatusToString( const Ipopt::SolverReturn& status )
{
  switch( status )
  {
    case Ipopt::SUCCESS:
    {
      return "SUCCESS";
    }
    case Ipopt::MAXITER_EXCEEDED:
    {
      return "MAXITER_EXCEEDED";
    }
    case Ipopt::CPUTIME_EXCEEDED:
    {
      return "CPUTIME_EXCEEDED";
    }
    case Ipopt::STOP_AT_TINY_STEP:
    {
      return "STOP_AT_TINY_STEP";
    }
    case Ipopt::STOP_AT_ACCEPTABLE_POINT:
    {
      return "STOP_AT_ACCEPTABLE_POINT";
    }
    case Ipopt::LOCAL_INFEASIBILITY:
    {
      return "LOCAL_INFEASIBILITY";
    }
    case Ipopt::USER_REQUESTED_STOP:
    {
      return "USER_REQUESTED_STOP";
    }
    case Ipopt::FEASIBLE_POINT_FOUND:
    {
      return "FEASIBLE_POINT_FOUND";
    }
    case Ipopt::DIVERGING_ITERATES:
    {
      return "DIVERGING_ITERATES";
    }
    case Ipopt::RESTORATION_FAILURE:
    {
      return "RESTORATION_FAILURE";
    }
    case Ipopt::ERROR_IN_STEP_COMPUTATION:
    {
      return "ERROR_IN_STEP_COMPUTATION";
    }
    case Ipopt::INVALID_NUMBER_DETECTED:
    {
      return "INVALID_NUMBER_DETECTED";
    }
    case Ipopt::TOO_FEW_DEGREES_OF_FREEDOM:
    {
      return "TOO_FEW_DEGREES_OF_FREEDOM";
    }
    case Ipopt::INVALID_OPTION:
    {
      return "INVALID_OPTION";
    }
    case Ipopt::OUT_OF_MEMORY:
    {
      return "OUT_OF_MEMORY";
    }
    case Ipopt::INTERNAL_ERROR:
    {
      return "INTERNAL_ERROR";
    }
    case Ipopt::UNASSIGNED:
    {
      return "UNASSIGNED";
    }
  }
  return "ERROR: UNHANDLED CASE IN IpoptUtilities::ipoptReturnStatusToString";
}

int IpoptUtilities::nzLowerTriangular( const SparseMatrixsc& A )
{
  int num{ 0 };
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      // Skip entries above the diagonal
      if( col > it.row() )
      {
        continue;
      }
      ++num;
    }
  }
  return num;
}

int IpoptUtilities::sparsityPatternLowerTriangular( const SparseMatrixsc& A, int* rows, int* cols )
{
  assert( rows != nullptr );
  assert( cols != nullptr );

  int curel{ 0 };
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      if( col > it.row() )
      {
        continue;
      }
      rows[curel] = it.row();
      cols[curel] = col;
      ++curel;
    }
  }

  return curel;
}

int IpoptUtilities::values( const SparseMatrixsc& A, scalar* vals )
{
  assert( vals != nullptr );

  int curel{ 0 };
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      vals[curel] = it.value();
      ++curel;
    }
  }

  assert( curel == A.nonZeros() );
  return curel;
}

int IpoptUtilities::valuesLowerTriangular( const SparseMatrixsc& A, scalar* vals )
{
  assert( vals != nullptr );

  int curel{ 0 };
  for( int col = 0; col < A.outerSize(); ++col )
  {
    for( SparseMatrixsc::InnerIterator it( A, col ); it; ++it )
    {
      if( col > it.row() )
      {
        continue;
      }
      vals[curel] = it.value();
      ++curel;
    }
  }

  return curel;
}
