#ifndef GL_CIRCLE_RENDERER_2D_H
#define GL_CIRCLE_RENDERER_2D_H

// TODO: Options to render with display lists, vbo, etc

#include "SCISim/Math/MathUtilities.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

class GLCircleRenderer2D
{

public:

  // num_points_single_half: number of points in a single half of the circle
  GLCircleRenderer2D( const unsigned num_points_single_half = 16 );

  void renderCircle( const Eigen::Matrix<GLdouble,3,1>& color ) const;

private:

  Eigen::Matrix<GLdouble,Eigen::Dynamic,1> m_x_crds;
  Eigen::Matrix<GLdouble,Eigen::Dynamic,1> m_y_crds;

};

#endif
