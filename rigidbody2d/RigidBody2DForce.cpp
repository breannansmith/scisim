// RigidBody2DForce.cpp
//
// Breannan Smith
// Last updated: 09/22/2015

#include "RigidBody2DForce.h"

#include "scisim/StringUtilities.h"

RigidBody2DForce::~RigidBody2DForce()
{}

std::string RigidBody2DForce::name() const
{
  return forceName();
}

void RigidBody2DForce::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  StringUtilities::serializeString( forceName(), output_stream );
  serializeState( output_stream );
}
