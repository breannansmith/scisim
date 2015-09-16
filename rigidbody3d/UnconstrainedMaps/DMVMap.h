// DMVMap.h
//
// Breannan Smith
// Last updated: 09/15/2015

#ifndef DMV_MAP_H
#define DMV_MAP_H

#include "SCISim/UnconstrainedMaps/UnconstrainedMap.h"

class DMVMap final : public UnconstrainedMap
{

public:

  virtual ~DMVMap() override;

  virtual void flow( const VectorXs& q0, const VectorXs& v0, FlowableSystem& fsys, const unsigned iteration, const scalar& dt, VectorXs& q1, VectorXs& v1 ) override;

  virtual void linearInertialConfigurationUpdate( const VectorXs& q0, const VectorXs& v0, const scalar& dt, VectorXs& q1 ) override;

  virtual std::string name() const override;

  virtual void serialize( std::ostream& output_stream ) const override;

};

#endif
