#include "DisplayController2D.h"

#include <algorithm>

DisplayController2D::DisplayController2D()
: m_window_width( 512 )
, m_window_height( 512 )
, m_scale_factor( 1.0 )
, m_center_x( 0.0 )
, m_center_y( 0.0 )
{}

void DisplayController2D::reshape()
{
  reshape( m_window_width, m_window_height );
}

void DisplayController2D::reshape( const unsigned w, const unsigned h )
{
  // Record the new width and height
  m_window_width = w;
  m_window_height = h;
  // Reset the coordinate system before modifying
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  // Set the coordinate system to achieve the desired zoom level, center
  const GLdouble ratio{ GLdouble( h ) / GLdouble( w ) };
  const GLdouble left{ m_center_x - m_scale_factor / ratio };
  const GLdouble right{ m_center_x + m_scale_factor / ratio };
  const GLdouble bottom{ m_center_y - m_scale_factor };
  const GLdouble top{ m_center_y + m_scale_factor };
  const GLdouble nearVal{ -1.0 };
  const GLdouble farVal{ 1.0 };
  glOrtho( left, right, bottom, top, nearVal, farVal );
  // Set the viewport to be the entire window
  glViewport( 0, 0, w, h );
}

void DisplayController2D::translateView( const GLdouble& dx, const GLdouble& dy )
{
  const GLdouble translate_x{ 2.0 * m_scale_factor * dx / GLdouble( m_window_height ) };
  const GLdouble translate_y{ 2.0 * m_scale_factor * dy / GLdouble( m_window_height ) };
  m_center_x -= translate_x;
  m_center_y += translate_y;
  reshape( m_window_width, m_window_height );
}

void DisplayController2D::zoomView( const GLdouble& delta )
{
  m_scale_factor = std::max( 0.00001, m_scale_factor + delta );
  reshape( m_window_width, m_window_height );
}

void DisplayController2D::setCenter( const GLdouble& x, const GLdouble& y )
{
  m_center_x = x;
  m_center_y = y;
}

void DisplayController2D::setScaleFactor( const GLdouble& scale )
{
  m_scale_factor = scale;
}

unsigned DisplayController2D::width() const
{
  return m_window_width;
}

unsigned DisplayController2D::height() const
{
  return m_window_height;
}

const GLdouble& DisplayController2D::scaleFactor() const
{
  return m_scale_factor;
}

const GLdouble& DisplayController2D::centerX() const
{
  return m_center_x;
}

const GLdouble& DisplayController2D::centerY() const
{
  return m_center_y;
}

void DisplayController2D::reset()
{
  m_scale_factor = 1.0;
  m_center_x = 0.0;
  m_center_y = 0.0;
}
