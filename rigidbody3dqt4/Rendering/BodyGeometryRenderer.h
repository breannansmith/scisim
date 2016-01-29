#ifndef BODY_GEOMETRY_RENDERER_H
#define BODY_GEOMETRY_RENDERER_H

#include <Eigen/Core>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

class BodyGeometryRenderer
{

public:

  virtual ~BodyGeometryRenderer() = 0;

  virtual void renderBody( const Eigen::Matrix<GLfloat,3,1>& color ) = 0;

};

#endif
