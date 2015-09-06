import balls2d
import numpy
import math

def startOfStep():
  t = balls2d.nextIteration() * balls2d.timestep()
  t0 = 2.0
  t1 = 4.0
  if t < t0:
    y_offset = 0.0
    vy = 0.0
  else:
    y_offset = - 0.1 * math.sin( 4.0 * math.pi * ( t - t0 ) )
    vy = - 0.4 * math.pi * math.cos( 4.0 * math.pi * ( t - t0 ) )
  if t < t1:
    x_offset = 0.0
    vx = 0.0
  else:
    x_offset = - 0.1 * math.sin( 3.0 * math.pi * ( t - t1 ) )
    vx = - 0.3 * math.pi * math.cos( 3.0 * math.pi * ( t - t1 ) )
  assert balls2d.numStaticPlanes() == 4
  balls2d.setStaticPlanePosition( 0, x_offset, -1.0 + y_offset )
  balls2d.setStaticPlanePosition( 1, x_offset,  1.0 + y_offset )
  balls2d.setStaticPlanePosition( 2, -1.0 + x_offset, y_offset )
  balls2d.setStaticPlanePosition( 3,  1.0 + x_offset, y_offset )
  balls2d.setStaticPlaneVelocity( 0, vx, vy )
  balls2d.setStaticPlaneVelocity( 1, vx, vy )
  balls2d.setStaticPlaneVelocity( 2, vx, vy )
  balls2d.setStaticPlaneVelocity( 3, vx, vy )
