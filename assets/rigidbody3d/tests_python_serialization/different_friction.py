import rigidbody3d
import numpy
import math

def frictionCoefficient():
  assert rigidbody3d.numCollisions() == 3
  coeffs = rigidbody3d.mu()
  coeffs[0] = 0.0
  coeffs[1] = 0.2
  coeffs[2] = 5.0
