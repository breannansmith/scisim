// RigidBody2DForce.cpp
//
// Breannan Smith
// Last updated: 09/10/2015

#include "RigidBody2DForce.h"

#include "scisim/StringUtilities.h"

RigidBody2DForce::~RigidBody2DForce()
{}

std::string RigidBody2DForce::name() const
{
  return getName();
}

void RigidBody2DForce::serialize( std::ostream& output_stream ) const
{
  assert( output_stream.good() );
  StringUtilities::serializeString( getName(), output_stream );
  serializeState( output_stream );
}
