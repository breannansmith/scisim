import rigidbody3d
import numpy

def restitutionCoefficient():
  cor = rigidbody3d.cor()
  ncollisions = rigidbody3d.numCollisions()
  assert cor.shape == (ncollisions,)
  for col_idx in range( 0, ncollisions ):
    indices = rigidbody3d.collisionIndices( col_idx )
    assert indices[0] <= 2
    assert indices[1] == -1
    if indices[0] == 0:
      cor[col_idx] = 0.0
    elif indices[0] == 1:
      cor[col_idx] = 0.5
    else:
      cor[col_idx] = 1.0
