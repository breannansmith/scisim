// SparseMatrixUtilities.h
//
// Breannan Smith
// Last updated: 09/03/2015

#ifndef SPARSE_MATRIX_UTILITIES_H
#define SPARSE_MATRIX_UTILITIES_H

#include "MathDefines.h"

namespace SparseMatrixUtilities
{

  void cuthillMckee( const SparseMatrixsc& input_matrix, VectorXi& permutation );

  void reverseCuthillMckee( const SparseMatrixsc& input_matrix, VectorXi& permutation );

  void permuteSparseMatrix( const SparseMatrixsc& input_matrix, const VectorXi& permutation, SparseMatrixsc& output_matrix );

  int computeMatrixBandwidth( const SparseMatrixsc& input_matrix );

}

#endif
