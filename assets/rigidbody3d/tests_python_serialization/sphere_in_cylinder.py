import rigidbody3d
import math

def startOfStep():
  assert rigidbody3d.numStaticCylinders() == 1
  t0 = 5.0
  omega = 2.0 * math.pi
  t = rigidbody3d.nextIteration() * rigidbody3d.timestep()
  if t >= t0:
    theta = omega * ( t - t0 )
    rigidbody3d.setStaticCylinderOrientation( 0, 0.0, 0.0, 1.0, theta )
    rigidbody3d.setStaticCylinderAngularVelocity( 0, 0.0, 0.0, omega )
