import math
import rigidbody2d

def startOfStep():
  next_iteration = rigidbody2d.nextIteration()
  assert next_iteration >= 1
  current_time = next_iteration * rigidbody2d.timestep()
  if current_time >= 2.0 and current_time <= 6.0:
    x_plane = 0.0
    y_plane = -2.0 - math.sin( 2.0 * math.pi * ( current_time - 2.0 ) )
    vx_plane = 0.0
    vy_plane = - 2.0 * math.pi * math.cos( 2.0 * math.pi * ( current_time - 2.0 ) )
  elif current_time >= 10.0:
    x_plane = math.sin( 2.0 * math.pi * ( current_time - 10.0 ) )
    y_plane = -2.0
    vx_plane = 2.0 * math.pi * math.cos( 2.0 * math.pi * ( current_time - 10.0 ) )
    vy_plane = 0.0
  else:
    x_plane = 0.0
    y_plane = -2.0
    vx_plane = 0.0
    vy_plane = 0.0
  rigidbody2d.setStaticPlanePosition( 0, x_plane, y_plane )
  rigidbody2d.setStaticPlaneVelocity( 0, vx_plane, vy_plane )
