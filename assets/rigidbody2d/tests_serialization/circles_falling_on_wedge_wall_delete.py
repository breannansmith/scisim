import rigidbody2d

def startOfStep():
  next_iteration = rigidbody2d.nextIteration()
  assert next_iteration >= 1
  current_iteration = next_iteration - 1
  current_time = current_iteration * rigidbody2d.timestep()
  if abs( current_time - 1.5 ) <= 1.0e-10:
    assert rigidbody2d.numStaticPlanes() == 4
    rigidbody2d.deleteStaticPlane( 3 )
