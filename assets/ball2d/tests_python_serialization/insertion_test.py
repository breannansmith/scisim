import balls2d
import numpy
import math

def startOfStep():
  assert balls2d.nextIteration() >= 1
  start_of_step_time = balls2d.nextIteration() * balls2d.timestep()
  if abs( 4.0 * start_of_step_time - float( int( 4.0 * start_of_step_time ) ) ) <= 1.0e-9:
    x = 8.0 * math.sin( 2.0 * start_of_step_time )
    balls2d.insertBall( x, 0.0, 0.0, 0.0, 0.25, 1.0, 0 )
