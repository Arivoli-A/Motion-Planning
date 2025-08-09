# priority queue for OPEN list
from pqdict import pqdict
from collections import deque

import math
import numpy as np
import os

import matplotlib.cm as cm
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.animation import FuncAnimation, PillowWriter


from .utils import draw_map


grid_step_x = 0.25
grid_step_y = 0.25
grid_step_z = 0.25 

EPS = 0.5*math.sqrt(grid_step_x**2+grid_step_y**2+grid_step_z**2)
CLEARANCE = 0.15

class AStarNode(object):
  def __init__(self, pqkey, coord):
    self.pqkey = pqkey
    self.coord = coord
    self.g = math.inf
    self.h = 0   
    self.parent_node = None
    self.parent_action = None
    self.closed = False

  def __lt__(self, other):
    return self.g < other.g     

class Environment:
  def __init__(self,boundary, blocks, heuristic = 'manhattan', start=np.zeros(3),goal=np.zeros(3)):
    self.start = start
    self.goal = goal
    self.boundary = boundary
    self.blocks = blocks
    self.heuristic = heuristic

    dx = grid_step_x * np.array([-1, 0, 1])
    dy = grid_step_y * np.array([-1, 0, 1])
    dz = grid_step_z * np.array([-1, 0, 1])

    dX, dY, dZ = np.meshgrid(dx, dy, dz, indexing='ij') # 3x3x3 
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten())) # 3x27
    dR = dR[:, ~(np.all(dR == 0, axis=0))] # np.delete(dR,13,axis=1) # removes (0,0,0) action 
    # dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0 # Normalization and halving

    self.action = dR
    #print(dR)

  def isGoal(self, node):
    # check if the position is around a ball of radius EPS
    return np.linalg.norm(self.goal-node.coord)<EPS

  def getSuccessors(self, node):
    
    numofdirs = 3*3*3-1

    successor_list = []
    cost_list = []
    action_list = []

    for i in range(numofdirs):
      next_pos = node.coord + self.action[:,i]
      #print(next_pos)      
      cost = self.getCost(self.action[:,i])

      # Check if this direction is valid
      
      # With each AABB
      for k in range(self.blocks.shape[0]):
        if( next_pos[0] >= self.blocks[k,0]-CLEARANCE and next_pos[0] <= self.blocks[k,3]+CLEARANCE and\
            next_pos[1] >= self.blocks[k,1]-CLEARANCE and next_pos[1] <= self.blocks[k,4]+CLEARANCE and\
            next_pos[2] >= self.blocks[k,2]-CLEARANCE and next_pos[2] <= self.blocks[k,5]+CLEARANCE ):
          cost = math.inf  # If it is inside in atleast one obstacle
          break
      
      # With boundary
      if( next_pos[0] < self.boundary[0,0] or next_pos[0] > self.boundary[0,3] or \
          next_pos[1] < self.boundary[0,1] or next_pos[1] > self.boundary[0,4] or \
          next_pos[2] < self.boundary[0,2] or next_pos[2] > self.boundary[0,5] ):
        cost = math.inf  # If it is outside boundary 

      if cost != math.inf:
        successor_list.append(next_pos)
        cost_list.append(cost)
        action_list.append(self.action[:,i])
    
    return successor_list, cost_list, action_list

  def getHeuristic(self, node):
      
    # self.heuristic: One of ['manhattan', 'euclidean', 'chebyshev']
    
    delta = self.goal - node.coord
    
    if self.heuristic == "manhattan":
      return np.linalg.norm(delta, ord=1)
    elif self.heuristic == "euclidean":
      return np.linalg.norm(delta, ord=2)
    elif self.heuristic == "chebyshev":
      return np.linalg.norm(delta, ord=np.inf)
    else:
      raise ValueError(f"Unknown heuristic_type '{self.heuristic}'. Choose from 'manhattan', 'euclidean', 'chebyshev'")

  def getCost(self,dr):
    return np.linalg.norm(dr)  # stage cost

class AStar:

  def __init__(self, boundary, blocks, name = None):
    self.boundary = boundary
    self.blocks = blocks
    self.env = Environment(boundary, blocks)
    self.name = name

  #@staticmethod
  
  def plan(self, start_coord, goal_coord,  name ='default', heuristic = 'manhattan', epsilon = 1):
    
    self.name = name
    
    # Initialize the graph and open list
    Graph = {}  # Construct graph based on nearest neighbours
    OPEN = pqdict() # Maintain  open list
    OPENED_NODES = []

    self.env.start = start_coord
    self.env.goal = goal_coord
    self.env.heuristic = heuristic

    # current node
    curr = AStarNode(tuple(start_coord), start_coord)
    curr.g = 0
    curr.h = self.env.getHeuristic(curr)

    Graph[curr.pqkey] = curr

    # TODO: Implement A* here

    while True:
      if self.env.isGoal(curr):
        path = self.recoverPath(curr)
        #self.create_opened_nodes_gif(OPENED_NODES, path)
        return path
        

      curr.closed = True

      self.updateData(curr, Graph, OPEN, epsilon)

      if not OPEN:
        return # If OPEN is empty, no path is found

      # remove the element with smallest f value
      curr = OPEN.popitem()[1][1]
      OPENED_NODES.append(curr.coord)

      #ax = plt.gca()
      #x, y, z = curr.coord  # Unpack the coordinates
      #ax.scatter(x, y, z)  # 'r' = red, 'o' = circle marker
      #plt.draw()
      #plt.pause(0.001)


  def updateData(self, current, Graph, OPEN, epsilon):
    reopen_nodes = True
    successor_list, cost_list, action_list = self.env.getSuccessors(current)

    for s_coord, s_cost, s_action in zip(successor_list, cost_list, action_list):

      s_key = tuple(s_coord)
      if s_key not in Graph:   # Construct graph based on nearest neighbours
        Graph[s_key] = AStarNode(s_key, s_coord)
        Graph[s_key].h = self.env.getHeuristic(Graph[s_key])
      child = Graph[s_key]

      tentative_g = current.g + s_cost
      
      if( tentative_g < child.g ):
        child.parent_node, child.parent_action = current, s_action # Change the parent (connection) of the expanded node 
        child.g = tentative_g # Correct label

        fval = tentative_g + epsilon*child.h
        if s_key in OPEN: # if OPEN, update priority
          OPEN[s_key] = (fval, child)
          OPEN.heapify(s_key)

        elif child.closed and reopen_nodes: # if CLOSED, consider reopening
          OPEN[s_key] = (fval, child)
          child.closed = False

        else: # new node, add to heap
          OPEN[s_key] = (fval, child)
    
  def recoverPath(self, node):
    path = deque()
    current = node

    while current:
      path.appendleft(current.coord)
      current = current.parent_node
    
    return np.array(path)
  
  def create_opened_nodes_gif(self, nodes, path, duration=0.2):
    """
    Create a 3D GIF from the opened nodes during A* search.
    
    :param gif_name: Output GIF filename
    :param duration: Frame duration in seconds
    """
    os.makedirs("gif", exist_ok=True)
    
    n = len(nodes)
    cmap = cm.get_cmap('rainbow')

    def animate(i):
      node = nodes[i]
      xs, ys, zs = [node[0]], [node[1]], [node[2]]
      ax.scatter(xs, ys, zs, c=[colors[i]], s=4)
      if i == n - 1:
        ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r-', linewidth=2)  # Plot final path
        
        

    # Create the figure
    fig, ax, hb, hs, hg = draw_map(self.env.boundary, self.env.blocks, self.env.start, self.env.goal)
    ax.set_title("Opened nodes in weighted A*")
    fig.tight_layout()
    norm = Normalize(vmin=0, vmax=n-1)
    sm = plt.cm.ScalarMappable(cmap=cmap,norm=norm)
    sm.set_array([])  # Required for colorbar
    cbar = plt.colorbar(sm, ax=ax, pad=0.1)
    cbar.set_label("Time")
    # Pre-calculate the color map
    colors = [cmap(j / (n - 1)) for j in range(n)]

    # Create the animation
    ani = FuncAnimation(fig, animate, frames=n, repeat=False)

    # Save the animation as a GIF or video
    gif_loc = os.path.join("gif", f"{self.name}_opened_nodes.gif")
    # Create PillowWriter with loop=0 (plays once)
    writer = PillowWriter(fps=30)
    ani.save(gif_loc, writer=writer)  

    plt.close()  # Close the plot when done

    print(f"GIF saved in {gif_loc}")
    



