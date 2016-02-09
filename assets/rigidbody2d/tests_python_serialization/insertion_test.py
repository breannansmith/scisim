import math
import rigidbody2d

def startOfStep():
  assert rigidbody2d.nextIteration() >= 1
  start_of_step_time = ( rigidbody2d.nextIteration() - 1 ) * rigidbody2d.timestep()
  if abs( 4.0 * start_of_step_time - float( int( 4.0 * start_of_step_time ) ) ) <= 1.0e-9:
    rigidbody2d.addCircleGeometry( 0.25 )
    x = 8.0 * math.sin( 2.0 * start_of_step_time )
    y = 0.0
    theta = 0.0
    vx = 0.0
    vy = 0.0
    omega = 0.0
    rho = 1.0
    geo_idx = rigidbody2d.numGeometryInstances() - 1
    fixed = 0
    rigidbody2d.addBody( x, y, theta, vx, vy, omega, rho, geo_idx, fixed )
