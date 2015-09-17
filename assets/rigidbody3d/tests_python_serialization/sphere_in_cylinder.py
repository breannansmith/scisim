import rigidbody3d
import numpy
import numpy.linalg
import math

_EPS = numpy.finfo(float).eps * 4.0

def vector_norm(data, axis=None, out=None):
  data = numpy.array(data, dtype=numpy.float64, copy=True)
  if out is None:
    if data.ndim == 1:
      return math.sqrt(numpy.dot(data, data))
    data *= data
    out = numpy.atleast_1d(numpy.sum(data, axis=axis))
    numpy.sqrt(out, out)
    return out
  else:
    data *= data
    numpy.sum(data, axis=axis, out=out)
    numpy.sqrt(out, out)

def quaternion_about_axis(angle, axis):
  q = numpy.array([0.0, axis[0], axis[1], axis[2]])
  qlen = vector_norm(q)
  if qlen > _EPS:
    q *= math.sin(angle/2.0) / qlen
  q[0] = math.cos(angle/2.0)
  return q

def angle_axis_from_two_vectors( a, b ):
  axis = numpy.cross( a, b )
  axis_norm = numpy.linalg.norm( axis )
  axis = axis / axis_norm
  absin = numpy.dot( axis, axis ) / axis_norm
  abcos = numpy.dot( a, b )
  theta = math.atan2( absin, abcos )
  return axis, theta

def quaternion_from_two_vectors( a, b ):
  axis, theta = angle_axis_from_two_vectors( a, b )
  return quaternion_about_axis( theta, axis )

def quaternion_multiply( quaternion1, quaternion0 ):
  w0, x0, y0, z0 = quaternion0
  w1, x1, y1, z1 = quaternion1
  return numpy.array([-x1*x0 - y1*y0 - z1*z0 + w1*w0,
                       x1*w0 + y1*z0 - z1*y0 + w1*x0,
                      -x1*z0 + y1*w0 + z1*x0 + w1*y0,
                       x1*y0 - y1*x0 + z1*w0 + w1*z0], dtype=numpy.float64)

def startOfStep():
  assert rigidbody3d.numStaticCylinders() == 1
  t0 = 5.0
  omega = 2.0 * math.pi
  t = rigidbody3d.nextIteration() * rigidbody3d.timestep()
  if t >= t0:
    theta = omega * ( t - t0 )
    rotation0 = quaternion_about_axis( theta, numpy.array([0.0,0.0,1.0]) )
    rotation1 = quaternion_from_two_vectors( numpy.array([0.0,1.0,0.0]), numpy.array([0.0,0.0,1.0]) )
    rotation2 = quaternion_multiply( rotation0, rotation1 )
    rigidbody3d.setStaticCylinderOrientation( 0, rotation2[0], rotation2[1], rotation2[2], rotation2[3] )
    rigidbody3d.setStaticCylinderAngularVelocity( 0, 0.0, 0.0, omega )
