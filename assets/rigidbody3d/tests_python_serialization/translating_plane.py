import rigidbody3d
import numpy
import math

def startOfStep():
  assert rigidbody3d.numStaticPlanes() == 1
  t0 = 5.0
  t1 = 20.0
  t = rigidbody3d.nextIteration() * rigidbody3d.timestep()
  if t >= t0 and t < t1:
    xpos = 1.0 - math.cos( math.pi * ( t - t0 ) )
    ypos = 0.0
    zpos = math.sin( 2.0 * math.pi * ( t - t0 ) )
    rigidbody3d.setStaticPlanePosition( 0, xpos, ypos, zpos )
    vx = math.pi * math.sin( math.pi * ( t - t0 ) )
    vy = 0.0
    vz = 2.0 * math.pi * math.cos( 2.0 * math.pi * ( t - t0 ) )
    rigidbody3d.setStaticPlaneVelocity( 0, vx, vy, vz )
  elif t >= t1:
    xpos = 1.0 - math.cos( math.pi * ( t - t0 ) )
    ypos = math.sin( math.pi * ( t - t1 ) )
    zpos = math.sin( 2.0 * math.pi * ( t - t0 ) )
    rigidbody3d.setStaticPlanePosition( 0, xpos, ypos, zpos )
    vx = math.pi * math.sin( math.pi * ( t - t0 ) )
    vy = math.pi * math.cos( math.pi * ( t - t1 ) )
    vz = 2.0 * math.pi * math.cos( 2.0 * math.pi * ( t - t0 ) )
    rigidbody3d.setStaticPlaneVelocity( 0, vx, vy, vz )
