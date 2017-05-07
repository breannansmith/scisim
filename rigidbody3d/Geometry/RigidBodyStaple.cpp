// RigidBodyStaple.cpp
//
// Breannan Smith
// Last updated: 09/15/2015

#include "RigidBodyStaple.h"

#include <iostream>

bool RigidBodyStaple::inertia_warning_printed = false;

void RigidBodyStaple::computeMassAndInertia( const scalar& density, scalar& M, Vector3s& CM, Vector3s& I, Matrix33sr& R ) const
{
  if( !inertia_warning_printed )
  {
    std::cerr << "Warning, approximating staple inertia as three cylinders." << std::endl;
    inertia_warning_printed = true;
  }

  // Mass of the horizontal component and the vertical components
  const scalar Mx = density * m_w * PI<scalar> * m_r * m_r;
  const scalar My = density * m_l * PI<scalar> * m_r * m_r;

  M = Mx + 2.0 * My;

  CM << 0.0, computeCenterOfMassY(), 0.0;

  // Inertia tensor
  {
    // Center of mass of the horizontal component
    const Vector3s CMx{ 0.0, -CM.y(), 0.0 };
    // Center of mass of the left vertical component
    const Vector3s CMy{ -0.5 * m_w, 0.5 * m_l - CM.y(), 0.0 }; // Just use this since all values are squared
    // Center of mass of the right vertical component
    //const Vector3s CMyr(  0.5 * m_w, 0.5 * m_l - CM.y(), 0.0 );

    // Contributions from translating the cylinders
    I.x() = Mx * ( CMx.y() * CMx.y() + CMx.z() * CMx.z() );
    I.x() += 2.0 * My * ( CMy.y() * CMy.y() + CMy.z() * CMy.z() );
    // Contributions from cylinders' centers of mass
    I.x() += Mx * ( m_r * m_r ) / 2.0;
    I.x() += 2.0 * My * ( 3.0 * m_r * m_r + m_l * m_l * m_l ) / 12.0;

    // Contributions from translating the cylinders
    I.y() = Mx * ( CMx.x() * CMx.x() + CMx.z() * CMx.z() );
    I.y() += 2.0 * My * ( CMy.x() * CMy.x() + CMy.z() * CMy.z() );
    // Contributions from cylinders' centers of mass
    I.y() += Mx * ( 3.0 * m_r * m_r + m_w * m_w * m_w ) / 12.0;
    I.y() += My * ( m_r * m_r );

    // Contributions from translating the cylinders
    I.z() = Mx * ( CMx.x() * CMx.x() + CMx.y() * CMx.y() );
    I.z() += 2.0 * My * ( CMy.x() * CMy.x() + CMy.y() * CMy.y() );
    // Contributions from cylinders' centers of mass
    I.z() += Mx * ( 3.0 * m_r * m_r + m_w * m_w * m_w ) / 12.0;
    I.z() += 2.0 * My * ( 3.0 * m_r * m_r + m_l * m_l * m_l ) / 12.0;
  }

  R.setIdentity();
}

RigidBodyStaple::RigidBodyStaple( const scalar& w, const scalar& l, const scalar& D )
: m_w( w - D )
, m_l( l - D )
, m_r( 0.5 * D )
, m_p()
{
  assert( w > D );
  assert( l > D );
  assert( D > 0.0 );

  // Compute the four corners of the staple
  m_p.resize( 4 );
  const scalar ycm{ computeCenterOfMassY() };
  m_p[0] << -0.5 * m_w, m_l - ycm, 0.0;
  m_p[1] << -0.5 * m_w,     - ycm, 0.0;
  m_p[2] <<  0.5 * m_w,     - ycm, 0.0;
  m_p[3] <<  0.5 * m_w, m_l - ycm, 0.0;
}

RigidBodyGeometryType RigidBodyStaple::getType() const
{
  return RigidBodyGeometryType::STAPLE;
}

std::unique_ptr<RigidBodyGeometry> RigidBodyStaple::clone() const
{
  RigidBodyStaple* newStaple = new RigidBodyStaple;

  newStaple->m_w = m_w;
  newStaple->m_l = m_l;
  newStaple->m_r = m_r;
  newStaple->m_p = m_p;

  return std::unique_ptr<RigidBodyGeometry>{ newStaple };
}

void RigidBodyStaple::computeAABB( const Vector3s& cm, const Matrix33sr& R, Array3s& min, Array3s& max ) const
{
  min.setConstant(  std::numeric_limits<scalar>::infinity() );
  max.setConstant( -std::numeric_limits<scalar>::infinity() );

  // For each corner of the staple
  assert( m_p.size() == 4 );
  for( char i = 0; i < 4; ++i )
  {
    const Array3s transformed_vertex{ R * m_p[i] + cm };
    min = min.min( transformed_vertex );
    max = max.max( transformed_vertex );
  }
  assert( ( min < max ).all() );
}

std::string RigidBodyStaple::name() const
{
  return "staple";
}

void RigidBodyStaple::serialize( std::ostream& output_stream ) const
{
  std::cerr << "Code up RigidBodyStaple::serialize" << std::endl;
  std::exit( EXIT_FAILURE );
}

scalar RigidBodyStaple::volume() const
{
  std::cerr << "Code up RigidBodyStaple::volume" << std::endl;
  std::exit( EXIT_FAILURE );
}

const std::vector<Vector3s>& RigidBodyStaple::points() const
{
  return m_p;
}

const scalar& RigidBodyStaple::r() const
{
  return m_r;
}

scalar RigidBodyStaple::computeCenterOfMassY() const
{
  return ( m_l * m_l ) / ( 2.0 * m_l + m_w );
}
