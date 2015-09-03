// FrictionOperatorUtilities.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "FrictionOperatorUtilities.h"

#include "SCISim/Constraints/Constraint.h"

static void reserveSpaceInBasisMatrix( const int nsamples, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D )
{
  assert( D.cols() % nsamples == 0 );

  const int ncons{ D.cols() / nsamples };

  VectorXi column_nonzeros{ D.cols() };
  std::vector<std::unique_ptr<Constraint>>::const_iterator itr{ K.begin() };
  for( int con_idx = 0; con_idx < ncons; ++con_idx )
  {
    for( int smpl_num = 0; smpl_num < nsamples; ++smpl_num )
    {
      column_nonzeros( con_idx * nsamples + smpl_num ) = (*itr)->frictionStencilSize();
    }
    ++itr;
  }
  assert( itr == K.end() );
  D.reserve( column_nonzeros );
}

static void buildLinearFrictionBasis( const VectorXs& q, const VectorXs& v, const int nsamples, const std::vector<std::unique_ptr<Constraint>>& K, SparseMatrixsc& D, VectorXs& drel )
{
  assert( D.cols() % nsamples == 0 );

  const unsigned ncons{ static_cast<unsigned>( D.cols() ) / nsamples };

  std::vector<std::unique_ptr<Constraint>>::const_iterator itr{ K.begin() };
  for( unsigned con_idx = 0; con_idx < ncons; ++con_idx )
  {
    (*itr)->computeGeneralizedFrictionDisk( q, v, con_idx * nsamples, nsamples, D, drel );
    ++itr;
  }
  assert( itr == K.end() );
}

void FrictionOperatorUtilities::formGeneralizedFrictionBasis( const VectorXs& q0, const VectorXs& v0, const std::vector<std::unique_ptr<Constraint>>& K, const int num_samples, SparseMatrixsc& D, VectorXs& drel )
{
  assert( num_samples > 0 );
  assert( D.rows() == v0.size() );
  assert( num_samples * int( K.size() ) == D.cols() );

  // Reserve space for entries
  reserveSpaceInBasisMatrix( num_samples, K, D );

  // Build the matrix
  buildLinearFrictionBasis( q0, v0, num_samples, K, D, drel );

  D.makeCompressed();
}

void FrictionOperatorUtilities::formLinearFrictionDiskConstraint( const int num_samples, SparseMatrixsc& E )
{
  {
    const VectorXi column_nonzeros{ VectorXi::Constant( E.cols(), num_samples ) };
    E.reserve( column_nonzeros );
  }
  // For each column
  for( int col = 0; col < E.cols(); ++col )
  {
    for( int samplenum = 0; samplenum < num_samples; ++samplenum )
    {
      // Note the negative for QL
      E.insert( num_samples * col + samplenum, col ) = 1.0;
    }
  }
  E.makeCompressed();
  assert( E.nonZeros() == E.cols() * num_samples );
  assert( E.sum() == E.nonZeros() );
}

void FrictionOperatorUtilities::computeMDPLambda( const VectorXs& vrel, VectorXs& lambda )
{
  assert( vrel.size() % 2 == 0 );
  lambda.conservativeResize( vrel.size() / 2 );
  for( int con_num = 0; con_num < lambda.size(); ++con_num )
  {
    lambda( con_num ) = vrel.segment<2>( 2 * con_num ).norm();
  }
}

void FrictionOperatorUtilities::projectOnFrictionDisc( const VectorXs& disc_bounds, VectorXs& beta )
{
  assert( beta.size() % 2 == 0 ); assert( beta.size() / 2 == disc_bounds.size() ); assert( ( disc_bounds.array() >= 0.0 ).all() );
  for( int con_num = 0; con_num < disc_bounds.size(); ++con_num )
  {
    const scalar dsc_nrm_sqrd{ beta.segment<2>( 2 * con_num ).squaredNorm() };
    // If the squared norm of the friction impulse is greater than the \mu \alpha squared
    if( dsc_nrm_sqrd > disc_bounds( con_num ) * disc_bounds( con_num ) )
    {
      // Normalize beta and rescale so its magnitude is \mu \alpha
      beta.segment<2>( 2 * con_num ) *= disc_bounds( con_num ) / sqrt( dsc_nrm_sqrd );
      assert( fabs( beta.segment<2>( 2 * con_num ).norm() - disc_bounds( con_num ) ) <= 1.0e-6 );
    }
  }
}
