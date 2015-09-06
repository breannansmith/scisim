import balls2d
import numpy

def restitutionCoefficient():
  cor = balls2d.cor()
  ncollisions = balls2d.numCollisions()
  assert cor.shape == (ncollisions,)
  for col_idx in range( 0, ncollisions ):
    indices = balls2d.collisionIndices( col_idx )
    assert indices[0] <= 2
    assert indices[1] == -1
    if indices[0] == 0:
      cor[col_idx] = 0.0
    elif indices[0] == 1:
      cor[col_idx] = 0.5
    else:
      cor[col_idx] = 1.0
