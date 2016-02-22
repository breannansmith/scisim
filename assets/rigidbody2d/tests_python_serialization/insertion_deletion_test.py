import math
import numpy
import rigidbody2d

def startOfStep():
  assert rigidbody2d.nextIteration() >= 1
  if ( rigidbody2d.nextIteration() - 1 ) % 25 == 0:
    if ( rigidbody2d.nextIteration() - 1 ) % 400 != 0:
      rigidbody2d.addCircleGeometry( 0.5 )
      start_of_step_time = ( rigidbody2d.nextIteration() - 1 ) * rigidbody2d.timestep()
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
    else:
      assert rigidbody2d.num_bodies() == rigidbody2d.num_geometry()
      bodies_to_delete = numpy.arange( rigidbody2d.num_bodies() - 1, dtype=numpy.uint32 )
      rigidbody2d.delete_bodies( bodies_to_delete )
      rigidbody2d.delete_geometry( bodies_to_delete )
