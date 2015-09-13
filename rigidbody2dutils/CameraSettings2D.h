// CameraSettings2D.h
//
// Breannan Smith
// Last updated: 09/11/2015

#ifndef CAMERA_SETTINGS_2D_H
#define CAMERA_SETTINGS_2D_H

#include "scisim/Math/MathDefines.h"

class CameraSettings2D
{

public:

  CameraSettings2D();

  bool set;
  Vector2s center;
  scalar scale;
  unsigned fps;
  bool render_at_fps;
  bool locked;
  
};

#endif
