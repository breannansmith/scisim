// SphereBodyConstraint.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "SphereBodyConstraint.h"

#include <iostream>

SphereBodyConstraint::SphereBodyConstraint( const int isphere, const int ibody, const Vector3s& p, const Vector3s& n, const VectorXs& q )
: m_idx_sphere( isphere )
, m_idx_body( ibody )
, m_n( n )
, m_r_sphere( p - q.segment<3>( 3 * m_idx_sphere ) )
, m_r_body( p - q.segment<3>( 3 * m_idx_body ) )
{
  assert( m_idx_sphere >= 0 );
  assert( m_idx_body >= 0 );
  assert( m_idx_sphere != m_idx_body );
  assert( fabs( m_n.norm() - 1.0 ) <= 1.0e-6 );
  
  //std::cout << "sphere body constraint" << std::endl;
  //std::cout << "  n: " << m_n.transpose() << std::endl;
  //std::cout << "  r_sphere: " << m_r_sphere.transpose() << std::endl;
  //std::cout << "  r_body: " << m_r_body.transpose() << std::endl;
}

SphereBodyConstraint::~SphereBodyConstraint()
{}

scalar SphereBodyConstraint::evalNdotV( const VectorXs& q, const VectorXs& v ) const
{
  std::cerr << "SphereBodyConstraint::evalNdotV not done!" << std::endl;
  //return m_n.dot( computeRelativeVelocity( v ) );
  return SCALAR_NAN; // To prevent warnings
}

void SphereBodyConstraint::evalgradg( const VectorXs& q, const int col, SparseMatrixsc& G, const FlowableSystem& fsys ) const
{
  assert( q.size() % 12 == 0 );
  assert( col >= 0 );
  assert( col < G.cols() );
  const int nbodies{ static_cast<int>( q.size() / 12 ) };

  //const Vector3s ntilde_sphere = m_r_sphere.cross( m_n );
  const Vector3s ntilde_body{ m_r_body.cross( m_n ) };

  // MUST BE ADDED GOING DOWN THE COLUMN. DO NOT TOUCH ANOTHER COLUMN.
  if( m_idx_body < m_idx_sphere )
  {
    assert( 3 * ( m_idx_body + nbodies ) + 2 < G.rows() );
    G.insert( 3 * m_idx_body + 0, col ) = m_n.x();
    G.insert( 3 * m_idx_body + 1, col ) = m_n.y();
    G.insert( 3 * m_idx_body + 2, col ) = m_n.z();
    G.insert( 3 * ( m_idx_body + nbodies ) + 0, col ) = ntilde_body.x();
    G.insert( 3 * ( m_idx_body + nbodies ) + 1, col ) = ntilde_body.y();
    G.insert( 3 * ( m_idx_body + nbodies ) + 2, col ) = ntilde_body.z();

    assert( 3 * m_idx_sphere + 2 < G.rows() );
    G.insert( 3 * m_idx_sphere + 0, col ) =  -m_n.x();
    G.insert( 3 * m_idx_sphere + 1, col ) =  -m_n.y();
    G.insert( 3 * m_idx_sphere + 2, col ) =  -m_n.z();
  }
  else
  {
    assert( 3 * m_idx_sphere + 2 < G.rows() );
    G.insert( 3 * m_idx_sphere + 0, col ) =  -m_n.x();
    G.insert( 3 * m_idx_sphere + 1, col ) =  -m_n.y();
    G.insert( 3 * m_idx_sphere + 2, col ) =  -m_n.z();

    assert( 3 * ( m_idx_body + nbodies ) + 2 < G.rows() );
    G.insert( 3 * m_idx_body + 0, col ) = m_n.x();
    G.insert( 3 * m_idx_body + 1, col ) = m_n.y();
    G.insert( 3 * m_idx_body + 2, col ) = m_n.z();
    G.insert( 3 * ( m_idx_body + nbodies ) + 0, col ) = ntilde_body.x();
    G.insert( 3 * ( m_idx_body + nbodies ) + 1, col ) = ntilde_body.y();
    G.insert( 3 * ( m_idx_body + nbodies ) + 2, col ) = ntilde_body.z();
  }
}

int SphereBodyConstraint::impactStencilSize() const
{
  // 3 DoFs from sphere, 6 from other body (normal impulse can't torque sphere)
  return 9;
}

int SphereBodyConstraint::frictionStencilSize() const
{
  std::cerr << "SphereBodyConstraint::frictionStencilSize not done!" << std::endl;
  std::exit( EXIT_FAILURE );
  //return 12;
}

void SphereBodyConstraint::getSimulatedBodyIndices( std::pair<int,int>& bodies ) const
{
  std::cerr << "SphereBodyConstraint::getSimulatedBodyIndices not done!" << std::endl;
  std::exit( EXIT_FAILURE );
//  bodies.first = m_i;
//  bodies.second = m_j;
}

void SphereBodyConstraint::evalKinematicNormalRelVel( const VectorXs& q, const int strt_idx, VectorXs& gdotN ) const
{
  assert( strt_idx >= 0 );
  assert( strt_idx < gdotN.size() );

  // No kinematic DoFs here
  gdotN( strt_idx ) = 0.0;
}

bool SphereBodyConstraint::conservesTranslationalMomentum() const
{
  return true;
}

bool SphereBodyConstraint::conservesAngularMomentumUnderImpact() const
{
  return true;
}

bool SphereBodyConstraint::conservesAngularMomentumUnderImpactAndFriction() const
{
  return true;
}

std::string SphereBodyConstraint::name() const
{
  return "sphere_body";
}

VectorXs SphereBodyConstraint::computeKinematicRelativeVelocity( const VectorXs& q, const VectorXs& v ) const
{
  std::cerr << "Code up: SphereBodyConstraint::computeKinematicRelativeVelocity" << std::endl;
  std::exit( EXIT_FAILURE );
}

//Vector3s BodyBodyConstraint::computeRelativeVelocity( const VectorXs& v ) const
//{
//  assert( v.size() % 6 == 0 );
//  assert( v.size() / 2 + 3 * m_i + 2 < v.size() );
//  assert( v.size() / 2 + 3 * m_j + 2 < v.size() );
//  
//  const int nbodies = v.size() / 6;
//  
//  // v_j + omega_j x r_j - ( v_i + omega_i x r_i )
//  return v.segment<3>( 3 * m_j ) + v.segment<3>( 3 * ( nbodies + m_j ) ).cross( m_r_j ) - v.segment<3>( 3 * m_i ) - v.segment<3>( 3 * ( nbodies + m_i ) ).cross( m_r_i );
//}
