import rigidbody3d
import numpy
import math

def startOfStep():
  assert rigidbody3d.numStaticPlanes() == 1
  t0 = 5.0;
  t = rigidbody3d.nextIteration() * rigidbody3d.timestep()
  if t >= t0:
    rigidbody3d.setStaticPlanePosition( 0, 0.0, math.sin( 2.0 * math.pi * ( t - t0 ) ), 0.0 )
    rigidbody3d.setStaticPlaneVelocity( 0, 0.0, 2.0 * math.pi * math.cos( 2.0 * math.pi * ( t - t0 ) ), 0.0 )
