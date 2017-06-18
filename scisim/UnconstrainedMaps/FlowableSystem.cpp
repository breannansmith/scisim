#include "FlowableSystem.h"

FlowableSystem::~FlowableSystem() = default;

unsigned FlowableSystem::numBodies() const
{
  return nvdofs() / numVelDoFsPerBody();
}
