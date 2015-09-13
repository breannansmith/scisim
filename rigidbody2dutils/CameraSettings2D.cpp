// CameraSettings2D.cpp
//
// Breannan Smith
// Last updated: 09/11/2015

#include "CameraSettings2D.h"

CameraSettings2D::CameraSettings2D()
: set( false )
, center( Vector2s::Zero() )
, scale( 1.0 )
, fps( 60 )
, render_at_fps( false )
, locked( false )
{}
