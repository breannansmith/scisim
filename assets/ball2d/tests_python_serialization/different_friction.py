import balls2d
import numpy
import math

def frictionCoefficient():
  grain_floor_mu = 0.6
  grain_grain_mu = 0.2
  coeffs = balls2d.mu()
  ncollisions = balls2d.numCollisions()
  for col_idx in range( 0, ncollisions ):
    col_type = balls2d.collisionType( col_idx )
    if col_type == 'ball_ball':
      coeffs[col_idx] = grain_grain_mu
    elif col_type == 'teleported_ball_ball':
      coeffs[col_idx] = grain_grain_mu
    elif col_type == 'static_plane_constraint':
      coeffs[col_idx] = grain_floor_mu
    else:
      print 'Unexpected constraint type encountered:', col_type
      sys.exit( 1 )
