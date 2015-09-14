#ifndef DISPLAY_CONTROLLER_2D_H
#define DISPLAY_CONTROLLER_2D_H

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

class DisplayController2D
{

public:

  DisplayController2D();

  void reshape( const unsigned w, const unsigned h );

  void translateView( const GLdouble& dx, const GLdouble& dy );

  void zoomView( const GLdouble& delta );

  void setCenter( const GLdouble& x, const GLdouble& y );

  void setScaleFactor( const GLdouble& scale );

  unsigned width() const;
  unsigned height() const;
  const GLdouble& scaleFactor() const;
  const GLdouble& centerX() const;
  const GLdouble& centerY() const;

  void reset();

private:

  // Width of the window in pixels
  unsigned m_window_width;

  // Height of the window in pixels
  unsigned m_window_height;

  // Factor to 'zoom' in or out by
  GLdouble m_scale_factor;

  // Center of the display, x coord
  GLdouble m_center_x;

  // Center of the display, y coord
  GLdouble m_center_y;

};

#endif
