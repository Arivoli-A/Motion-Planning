import numpy as np

CLEARANCE = 0.1

class MyPlanner:
  __slots__ = ['boundary', 'blocks','name']

  def __init__(self, boundary, blocks, name=None):
    self.boundary = boundary
    self.blocks = blocks
    self.name = name


  def plan(self,start,goal, name ='default'):
    
    self.name = name
    path = [start]
    numofdirs = 26
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1]) # 3x3x3
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten())) # 3x27
    dR = np.delete(dR,13,axis=1) # removes (0,0,0) action 
    dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0 # Normalization and halving
    
    for _ in range(2000):
      mindisttogoal = 1000000
      node = None
      for k in range(numofdirs):
        next = path[-1] + dR[:,k]
        
        # Check if this direction is valid
        # First with boundary
        if( next[0] < self.boundary[0,0] or next[0] > self.boundary[0,3] or \
            next[1] < self.boundary[0,1] or next[1] > self.boundary[0,4] or \
            next[2] < self.boundary[0,2] or next[2] > self.boundary[0,5] ):
          continue
        
        valid = True
        # With each AABB
        for k in range(self.blocks.shape[0]):
          if( next[0] >= self.blocks[k,0]-CLEARANCE and next[0] <= self.blocks[k,3]+CLEARANCE and\
              next[1] >= self.blocks[k,1]-CLEARANCE and next[1] <= self.blocks[k,4]+CLEARANCE and\
              next[2] >= self.blocks[k,2]-CLEARANCE and next[2] <= self.blocks[k,5]+CLEARANCE ):
            valid = False
            break
        if not valid:
          continue
        
        # Update next node
        disttogoal = sum((next - goal)**2)
        if( disttogoal < mindisttogoal):
          mindisttogoal = disttogoal
          node = next
      
      if node is None:
        break
      
      path.append(node)
      
      # Check if done
      if sum((path[-1]-goal)**2) <= 0.1:
        break
      
    return np.array(path)

