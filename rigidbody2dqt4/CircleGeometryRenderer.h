#ifndef CIRCLE_GEOMETRY_RENDERER_H
#define CIRCLE_GEOMETRY_RENDERER_H

#include "RigidBodyRenderer2D.h"

class CircleGeometry;
class GLCircleRenderer2D;

class CircleGeometryRenderer final : public RigidBodyRenderer2D
{

public:

  CircleGeometryRenderer( const CircleGeometry& geo, const GLCircleRenderer2D& circle_renderer );

  virtual ~CircleGeometryRenderer() override = default;

  virtual void render( const Eigen::Matrix<GLdouble,3,1>& color ) override;

  virtual void renderTeleported( const Eigen::Matrix<GLdouble,3,1>& color ) override;

private:

  const CircleGeometry& m_circle_geo;
  const GLCircleRenderer2D& m_circle_renderer;

};

#endif
