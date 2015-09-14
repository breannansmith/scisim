#ifndef RIGID_BODY_RENDERER_2D_H
#define RIGID_BODY_RENDERER_2D_H

#include "SCISim/Math/MathDefines.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

class RigidBodyRenderer2D
{

public:

  virtual ~RigidBodyRenderer2D() = 0;

  virtual void render( const Eigen::Matrix<GLdouble,3,1>& color ) = 0;

};

#endif
