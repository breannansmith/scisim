import math
import rigidbody2d

def startOfStep():
  next_iteration = rigidbody2d.nextIteration()
  assert next_iteration >= 1
  current_time = next_iteration * rigidbody2d.timestep()
  if current_time <= 1.0:
    nx = - math.sin( 0.5 * math.pi * current_time )
    ny = math.cos( 0.5 * math.pi * current_time )
    omega = 0.5 * math.pi
  else:
    nx = -1.0
    ny = 0.0
    omega = 0.0
  rigidbody2d.setStaticPlaneNormal( 1, nx, ny )
  rigidbody2d.setStaticPlaneAngularVelocity( 1, omega )
