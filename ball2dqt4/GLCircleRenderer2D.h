#ifndef GL_CIRCLE_RENDERER_2D_H
#define GL_CIRCLE_RENDERER_2D_H

// TODO: Options to render with display lists, vbo, etc

#include "scisim/Math/MathUtilities.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

class GLCircleRenderer2D
{

public:

  // num_pts: Number of points to sample the circle with
  explicit GLCircleRenderer2D( const unsigned num_pts = 16 );

  // x: Position of the circle's center
  // r: Radius of the circle
  void renderCircle( const Vector2s& x, const scalar& r ) const;

  // x: Position of the circle's center
  // r: Radius of the circle
  void renderSolidCircle( const Vector2s& x, const scalar& r ) const;

private:

  Eigen::Matrix<GLdouble,Eigen::Dynamic,1> m_x_crds;
  Eigen::Matrix<GLdouble,Eigen::Dynamic,1> m_y_crds;

};

#endif
