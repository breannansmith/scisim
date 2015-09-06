import balls2d
import numpy
import math

def startOfStep():
  start_of_step_time = balls2d.nextIteration() * balls2d.timestep()
  if start_of_step_time >= 2 and balls2d.numStaticPlanes() == 4:
    balls2d.deleteStaticPlane( 2 )
