#ifndef BOX_GEOMETRY_RENDERER_H
#define BOX_GEOMETRY_RENDERER_H

#include "RigidBodyRenderer2D.h"

class BoxGeometry;

class BoxGeometryRenderer final : public RigidBodyRenderer2D
{

public:

  explicit BoxGeometryRenderer( const BoxGeometry& geo );

  virtual ~BoxGeometryRenderer() override = default;

  virtual void render( const Eigen::Matrix<GLdouble,3,1>& color ) override;

  virtual void renderTeleported( const Eigen::Matrix<GLdouble,3,1>& color ) override;

private:

  const BoxGeometry& m_geo;

};

#endif
