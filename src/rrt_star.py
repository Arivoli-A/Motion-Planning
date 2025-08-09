from ompl import util as ou
from ompl import base as ob
from ompl import geometric as og

import numpy as np
import os

import matplotlib.cm as cm
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from .utils import draw_map

EPS = 0.05
CLEARANCE = 0.15

#output_file = ou.OutputHandlerFile("log/ompl_output.txt")
#ou.set = output_file
# Set log level (optional)
#ou.setLogLevel(ou.LOG_DEBUG)

# class CustomSampler(ob.StateSampler):
#   def __init__(self, space):
#       super().__init__(space)

#   def sampleGaussian(self, state, mean, stdDev):
#       self.space.sampleGaussian(state, mean, stdDev)

# Overriding the default sampler in the state space
def allocCustomSampler(space):
    return CustomSampler(space)

class Environment:
  def __init__(self,boundary, blocks, start=np.zeros(3),goal=np.zeros(3)):
  
    self.boundary = boundary
    self.blocks = blocks
    self.node_opened = []

    # create an R^3 state space
    space = ob.RealVectorStateSpace(3)

    # set lower and upper bounds
    bounds = ob.RealVectorBounds(3)

    bounds.setLow(0, float(boundary[0,0]))   # Set dimension 0 low bound 
    bounds.setLow(1, float(boundary[0,1]))   # Set dimension 1 low bound
    bounds.setLow(2, float(boundary[0,2]))  # Set dimension 2 low bound 
    
    bounds.setHigh(0, float(boundary[0,3]))   # Set dimension 0 upper bound 
    bounds.setHigh(1, float(boundary[0,4]))   # Set dimension 1 upper bound
    bounds.setHigh(2, float(boundary[0,5]))   # Set dimension 2 upper bound 
    
    space.setBounds(bounds)

    # space.setStateSamplerAllocator(ob.StateSamplerAllocator(allocCustomSampler))

    si = ob.SpaceInformation(space)

    si.setStateValidityChecker(ob.StateValidityCheckerFn(self.StateValidity))

    si.setup()

    self.start = ob.State(space)
    self.goal = ob.State(space)

    self.si = si

  def StateValidity(self, coord):
    # Check if this direction is valid
    valid = True

    self.node_opened.append([coord[i] for i in range(3)])
    
    # With each AABB
    for k in range(self.blocks.shape[0]):
      if( coord[0] >= self.blocks[k,0]-CLEARANCE and coord[0] <= self.blocks[k,3]+CLEARANCE and\
          coord[1] >= self.blocks[k,1]-CLEARANCE and coord[1] <= self.blocks[k,4]+CLEARANCE and\
          coord[2] >= self.blocks[k,2]-CLEARANCE and coord[2] <= self.blocks[k,5]+CLEARANCE ):
        valid = False  # If it is inside in atleast one obstacle
        break
    
    # With boundary
    if( coord[0] < self.boundary[0,0] or coord[0] > self.boundary[0,3] or \
        coord[1] < self.boundary[0,1] or coord[1] > self.boundary[0,4] or \
        coord[2] < self.boundary[0,2] or coord[2] > self.boundary[0,5] ):
      valid = False  # If it is outside boundary 

    return valid

class RRTStar:

  def __init__(self, boundary, blocks, name = None):
    self.boundary = boundary
    self.blocks = blocks
    self.env = Environment(boundary, blocks)
    self.name = name

  #@staticmethod
  
  def plan(self, start_coord, goal_coord,  name ='default', heuristic = 'manhattan', epsilon = 1):
    tmax = 60 # 1 sec
    int_point = 500 # Interpolation point for RRT*
    self.name = name
    self.start_coord = start_coord
    self.goal_coord = goal_coord

    self.env.start()[0] = float(start_coord[0])
    self.env.start()[1] = float(start_coord[1])
    self.env.start()[2] = float(start_coord[2])

    self.env.goal()[0] = float(goal_coord[0])
    self.env.goal()[1] = float(goal_coord[1])
    self.env.goal()[2] = float(goal_coord[2])

    # Create a problem instance
    pdef = ob.ProblemDefinition(self.env.si)
    
    # Set start and goal states in the problem
    pdef.setStartAndGoalStates(self.env.start, self.env.goal, EPS)
    
    # Set objective function - Minimize path length
    pdef.setOptimizationObjective(ob.PathLengthOptimizationObjective(self.env.si))

    planner = og.RRTstar(self.env.si)
    # planner.setGoalBias(0.5) # default 0.05

    planner.setProblemDefinition(pdef)
    planner.setup()

    # Solve the planning problem using RRT*. solve - true/false
    solve = planner.solve(tmax)

    if solve:
      # Check if a solution was found
      if pdef.hasSolution():

        # Get the solution path
        path_sol = pdef.getSolutionPath()
        
        # Optional: simplify the path
        path_simplifier = og.PathSimplifier(self.env.si)
        path_simplifier.simplifyMax(path_sol)
        path_sol.interpolate(int_point)  # makes the path smoother with interpolation point

        # Extract coordinates
        states = path_sol.getStates()
        coords = []
        for state in states:
            coord = [state[i] for i in range(self.env.si.getStateSpace().getDimension())]
            coords.append(coord)
        
        # self.create_opened_nodes_gif(np.array(coords))

        return np.array(coords)
      else:
          print("No solution path found.")
          return None
    else:
      print("Planner failed to solve within the time limit.")
      return None

  def create_opened_nodes_gif(self, path, duration=0.2):
    """
    Create a 3D GIF from the opened nodes during A* search.
    
    :param gif_name: Output GIF filename
    :param duration: Frame duration in seconds
    """
    os.makedirs("gif", exist_ok=True)
    
    sample_rate = 10  # Take every 5th node
    sampled_nodes = self.env.node_opened[::sample_rate]

    n = len(sampled_nodes)
    cmap = cm.get_cmap('rainbow')

    # Create the figure
    fig, ax, hb, hs, hg = draw_map(self.env.boundary, self.env.blocks, self.start_coord, self.goal_coord)

    # Pre-calculate the color map
    colors = [cmap(j / (n - 1)) for j in range(n)]

    # Initialize scatter object for animation
    sc = ax.scatter([], [], [], c=[], s=4)
    path_line, = ax.plot([], [], [], 'r-', linewidth=2)

    def init():
      sc._offsets3d = ([], [], [])
      sc.set_color([])
      path_line.set_data([], [])
      path_line.set_3d_properties([])
      
      return sc, path_line

    def animate(i):
      node = sampled_nodes[i]
      xs, ys, zs = [node[0]], [node[1]], [node[2]]
      #ax.scatter(xs, ys, zs, c=[colors[i]], s=4)
      sc._offsets3d = (xs, ys, zs)
      sc.set_color([colors[i]])
      if i == n - 1:
        # ax.plot(path[:, 0], path[:, 1], path[:, 2], 'r-', linewidth=2)  # Plot final path
        path_line.set_data(path[:, 0], path[:, 1])
        path_line.set_3d_properties(path[:, 2])
    
      return sc, path_line

    
    # Create the animation
    ani = FuncAnimation(fig, animate, frames=n, repeat=False, blit = True)

    # Save the animation as a GIF or video
    gif_loc = os.path.join("gif", f"{self.name}_opened_nodes.gif")
    ani.save(gif_loc, writer='pillow', fps=30)  

    plt.close()  # Close the plot when done

    print(f"GIF saved in {gif_loc}")
  


