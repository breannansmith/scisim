// JacobiOperator.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "JacobiOperator.h"

// Temp include
#include <iostream>

JacobiOperator::JacobiOperator( const scalar& v_tol )
: m_v_tol( v_tol )
{}

void JacobiOperator::flow( const std::vector<std::unique_ptr<Constraint>>& cons, const SparseMatrixsc& M, const SparseMatrixsc& Minv, const VectorXs& q0, const VectorXs& v0, const VectorXs& v0F, const SparseMatrixsc& N, const SparseMatrixsc& Q, const VectorXs& nrel, const VectorXs& CoR, VectorXs& alpha )
{
  std::cerr << "Error, JacobiOperator::flow has not been updated to work with new interface." << std::endl;
  std::exit( EXIT_FAILURE );
//  if( lambda.size() != 0 )
//  {
//    std::cerr << "Tracking of global lagrange multipliers not yet supporrted in JacobiOperator" << std::endl;
//    std::exit(EXIT_FAILURE);
//  }
//
//  // v1 will store this iteration's solution
//  v1 = v0;
//  // vel will store last iteration's solution
//  VectorXs vel = v0;
//
//  // Iterate until all constraint violations fall below the threshold
//  bool collision_happened = !K.empty();
//  while( collision_happened )
//  {
//    collision_happened = false;
//
//    // For each constraint
//    for( std::vector<std::unique_ptr<Constraint>>::iterator itr = K.begin(); itr != K.end(); itr++ )
//    {
//      // If the relative velocity along the constraint is below the threshold
//      if( (*itr)->evalNdotV( v1 ) < -m_v_tol )
//      {
//        // Reflect about this constraint
//        (*itr)->resolveImpact( CoR, fsys.M(), v1, vel );
//        // And remember that a collision happend
//        collision_happened = true;
//      }
//    }
//
//    // This iteration's solution is the start of next iteration
//    v1 = vel;
//  }
}

std::string JacobiOperator::name() const
{
  return "jacobi";
}

std::unique_ptr<ImpactOperator> JacobiOperator::clone() const
{
  return std::unique_ptr<ImpactOperator>{ new JacobiOperator{ m_v_tol } };
}

void JacobiOperator::serialize( std::ostream& output_stream ) const
{
  std::cerr << "Code up JacobiOperator::serialize" << std::endl;
  std::exit( EXIT_FAILURE );
}
