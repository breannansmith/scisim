// FlowableSystem.cpp
//
// Breannan Smith
// Last updated: 09/03/2015

#include "FlowableSystem.h"

FlowableSystem::~FlowableSystem()
{}

unsigned FlowableSystem::numBodies() const
{
  return nvdofs() / numVelDoFsPerBody();
}
