// sparse_matrix_tests.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include <iostream>
#include <cstdlib>
#include <string>

#include "SCISim/Math/MathDefines.h"
#include "SCISim/Math/SparseMatrixUtilities.h"
#include <Eigen/OrderingMethods>

// 0 1 1 0
// 1 0 0 1
// 1 0 1 0
// 0 1 0 0
static const unsigned test_00_size[]{4,4};
static const unsigned test_00_rows_input[]{1,2,0,3,0,2,1};
static const unsigned test_00_cols_input[]{0,0,1,1,2,2,3};
static const int test_00_permutation[]{2,0,1,3};

// 60 vertex buckyball
static const unsigned test_01_size[]{60,60};
static const unsigned test_01_rows_input[]{1,4,5,0,2,10,1,3,15,2,4,20,0,3,25,0,6,9,5,7,29,6,8,41,7,9,37,5,8,11,1,11,14,9,10,12,11,13,36,12,14,32,10,13,16,2,16,19,14,15,17,16,18,31,17,19,52,15,18,21,3,21,24,19,20,22,21,23,51,22,24,47,20,23,26,4,26,29,24,25,27,26,28,46,27,29,42,6,25,28,31,34,53,17,30,32,13,31,33,32,34,35,30,33,55,33,36,39,12,35,37,8,36,38,37,39,40,35,38,56,38,41,44,7,40,42,28,41,43,42,44,45,40,43,57,43,46,49,27,45,47,23,46,48,47,49,50,45,48,58,48,51,54,22,50,52,18,51,53,30,52,54,50,53,59,34,56,59,39,55,57,44,56,58,49,57,59,54,55,58};
static const unsigned test_01_cols_input[]{0,0,0,1,1,1,2,2,2,3,3,3,4,4,4,5,5,5,6,6,6,7,7,7,8,8,8,9,9,9,10,10,10,11,11,11,12,12,12,13,13,13,14,14,14,15,15,15,16,16,16,17,17,17,18,18,18,19,19,19,20,20,20,21,21,21,22,22,22,23,23,23,24,24,24,25,25,25,26,26,26,27,27,27,28,28,28,29,29,29,30,30,30,31,31,31,32,32,32,33,33,33,34,34,34,35,35,35,36,36,36,37,37,37,38,38,38,39,39,39,40,40,40,41,41,41,42,42,42,43,43,43,44,44,44,45,45,45,46,46,46,47,47,47,48,48,48,49,49,49,50,50,50,51,51,51,52,52,52,53,53,53,54,54,54,55,55,55,56,56,56,57,57,57,58,58,58,59,59,59};
static const int test_01_permutation[]{0,5,1,4,6,25,29,9,10,11,2,3,7,26,28,8,14,12,15,16,20,24,41,27,42,37,36,13,19,17,21,23,40,46,43,38,35,32,18,31,22,47,44,45,39,33,52,30,51,48,57,49,56,34,53,50,58,55,54,59};


static int executeRCMOrderingTest( const SparseMatrixsc& input_matrix, const VectorXi& expected_permutation )
{
  VectorXi permutation{ expected_permutation.size() };
  SparseMatrixUtilities::reverseCuthillMckee( input_matrix, permutation );

  // Permute using our permutation and compute the bandwidth
  int local_bandwidth;
  {
    SparseMatrixsc local_matrix;
    SparseMatrixUtilities::permuteSparseMatrix( input_matrix, permutation, local_matrix );
    local_bandwidth = SparseMatrixUtilities::computeMatrixBandwidth( local_matrix );
  }

  // Permute using the oracle permutation and compute the bandwidth
  int oracle_bandwidth;
  {
    SparseMatrixsc oracle_matrix;
    SparseMatrixUtilities::permuteSparseMatrix( input_matrix, permutation, oracle_matrix );
    oracle_bandwidth = SparseMatrixUtilities::computeMatrixBandwidth( oracle_matrix );
  }

  if( local_bandwidth != oracle_bandwidth )
  {
    std::cerr << "Warning, bandwidths do not match: " << local_bandwidth << " " << oracle_bandwidth << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

template<size_t N>
static int readInputMatrix( const unsigned (&dimensions)[2], const unsigned (&input_rows)[N], const unsigned (&input_cols)[N], SparseMatrixsc& input_matrix )
{
  // Resize the sparse matrix
  if( dimensions[0] != dimensions[1] )
  {
    std::cerr << "Bad input dimensions in readInputMatrix" << std::endl;
    return EXIT_FAILURE;
  }
  input_matrix.resize( dimensions[0], dimensions[1] );

  // Load in the sparse matrix
  {
    std::vector< Eigen::Triplet<scalar> > matrix_entries(N);
    for( unsigned entry_num = 0; entry_num < N; ++entry_num )
    {
      matrix_entries[entry_num] = Eigen::Triplet<scalar>( input_rows[entry_num], input_cols[entry_num], 1.0 );
      if( input_rows[entry_num] >= unsigned( input_matrix.rows() ) || input_cols[entry_num] >= unsigned( input_matrix.cols() ) )
      {
        std::cerr << "Bad index in readInputMatrix" << std::endl;
        return EXIT_FAILURE;
      }
    }
    input_matrix.setFromTriplets( matrix_entries.begin(), matrix_entries.end() );
  }
  input_matrix.makeCompressed();

  return EXIT_SUCCESS;
}

template<size_t N>
static int readInputPermutation( const int matrix_size, const int (&input_permutation)[N], VectorXi& permutation )
{
  // Read the permutation
  permutation.resize( N );
  for( size_t i = 0; i < N; ++i )
  {
    permutation( i ) = input_permutation[i];
    // Check that the index is valid
    if( permutation( i ) < 0 || permutation( i ) >= matrix_size )
    {
      std::cerr << "Bad permutation value in readInputPermutation" << std::endl;
      return EXIT_FAILURE;
    }
  }
  // Check that each element of the permutation appears once
  {
    const double expected_total{ 0.5 * ( ( double( matrix_size ) - 1.0 ) *  double( matrix_size ) ) };
    double actual_total = 0.0;
    for( size_t i = 0; i < N; ++i )
    {
      actual_total += double( permutation( i ) );
    }
    if( fabs( expected_total - actual_total ) > 1.0e-6 )
    {
      std::cerr << "Bad total in readInputPermutation" << std::endl;
      return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}

int main( int argc, char** argv )
{
  if( argc != 2 )
  {
    std::cerr << "Usage: " << argv[0] << " test_name" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string test_name{ argv[1] };

  if( test_name == "rcm_ordering_00" )
  {
    SparseMatrixsc matrix;
    if( readInputMatrix( test_00_size, test_00_rows_input, test_00_cols_input, matrix ) != EXIT_SUCCESS )
    {
      return EXIT_FAILURE;
    }
    VectorXi permutation;
    if( readInputPermutation( test_00_size[0], test_00_permutation, permutation ) != EXIT_SUCCESS )
    {
      return EXIT_FAILURE;
    }
    return executeRCMOrderingTest( matrix, permutation );
  }
  else if( test_name == "rcm_ordering_01" )
  {
    SparseMatrixsc matrix;
    if( readInputMatrix( test_01_size, test_01_rows_input, test_01_cols_input, matrix ) != EXIT_SUCCESS )
    {
      return EXIT_FAILURE;
    }
    VectorXi permutation;
    if( readInputPermutation( test_01_size[0], test_01_permutation, permutation ) != EXIT_SUCCESS )
    {
      return EXIT_FAILURE;
    }
    return executeRCMOrderingTest( matrix, permutation );
  }

  std::cerr << "Invalid test specified: " << test_name << std::endl;
  return EXIT_FAILURE;
}
