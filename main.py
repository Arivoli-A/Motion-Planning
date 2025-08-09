import numpy as np
import matplotlib.pyplot as plt; plt.ion()
import os
from datetime import datetime

from src import Planner, astar, rrt_star, collision
from src.utils import tic, toc, load_map, draw_map

os.makedirs('log', exist_ok=True)

def runtest(mapfile, name, start, goal, planner_class = Planner.MyPlanner, heuristic = 'manhattan', verbose = True, log_file = None):
  '''
  This function:
   * loads the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(mapfile)
  MP = planner_class(boundary, blocks) # TODO: replace this with your own planner implementation
  
  # Display the environment
  if verbose:
    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)

  # Call the motion planner
  t0 = tic()
  if isinstance(MP, Planner.MyPlanner):
    name = f'Greedy_{name}'
    path = MP.plan(start, goal, name =name)
  elif isinstance(MP, astar.AStar):
    name =f'AStar_{name}'
    path = MP.plan(start, goal, heuristic = heuristic, name =name)
  elif isinstance(MP, rrt_star.RRTStar):
    name =f'RRTStar_{name}'
    path = MP.plan(start, goal, name =name)
  toc(t0,"Planning")
  
  # Plot the path
  if verbose:
    ax.plot(path[:,0],path[:,1],path[:,2],'r-')

    os.makedirs("Figure", exist_ok=True)
    

    # Dictionary of views: name -> (elevation, azimuth)
    views = {"top_z": (90, -90),    # Top view (Z direction)
          "front_x": (0, -90),   # Front view (X direction)
          "side_y": (0, 0),      # Side view (Y direction)
          "perspective": (30, -60)  # Optional perspective view
          }
    for view_name, (elev, azim) in views.items():
      ax.view_init(elev=elev, azim=azim)
      plot_path = os.path.join("Figure", f"{name}_{view_name}.png")
      # Save the figure
      plt.savefig(plot_path, bbox_inches='tight')

    plt.close()

    print(f"Path plot saved to: {plot_path}")

  # TODO: You should verify whether the path actually intersects any of the obstacles in continuous space
  # TODO: You can implement your own algorithm or use an existing library for segment and 
  #       axis-aligned bounding box (AABB) intersection

  collision_check = collision.detect_collision(path, blocks)
  print('Did collision happen: ', collision_check)
  goal_reached = sum((path[-1]-goal)**2) <= 0.1
  print('Goal reached: ',goal_reached)
  success = (not collision_check) and goal_reached
  pathlength = np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))

  current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

  if log_file != None:
    with open(log_file, 'a') as f:
      f.write(f"Run started at: {current_time}\n")
      f.write("Running"+ name + "...\n")
      f.write(f"Success: {success}\n")
      f.write(f"Path length: {pathlength:.4f}\n\n")

    print(f"log saved to: {log_file}")

  return success, pathlength


def test_single_cube(name = 'single_cube', planner_class = Planner.MyPlanner, heuristic = 'manhattan', verbose = True, log=None):
  print('Running single cube test...\n') 
  start = np.array([7.0, 7.0, 5.5])
  goal = np.array([2.3, 2.3, 1.3])
  success, pathlength = runtest('./maps/single_cube.txt',name ,start, goal, planner_class, heuristic, verbose,log)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')
  

def test_maze(name = 'maze',planner_class = Planner.MyPlanner, heuristic = 'manhattan', verbose = True, log=None):
  print('Running maze test...\n') 
  start = np.array([0.0, 0.0, 1.0])
  goal = np.array([12.0, 12.0, 5.0])
  success, pathlength = runtest('./maps/maze.txt',name, start, goal, planner_class,  heuristic, verbose,log)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')

    
def test_window(name = 'window',planner_class = Planner.MyPlanner, heuristic = 'manhattan', verbose = True, log=None):
  print('Running window test...\n') 
  start = np.array([6, -4.9, 2.8])
  goal = np.array([2.0, 19.5, 5.5])
  success, pathlength = runtest('./maps/window.txt',name, start, goal, planner_class, heuristic, verbose,log)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')

  
def test_tower(name = 'tower', planner_class = Planner.MyPlanner, heuristic = 'manhattan', verbose = True, log=None):
  print('Running tower test...\n') 
  start = np.array([4.0, 2.5, 19.5])
  goal = np.array([2.5, 4.0, 0.5])
  success, pathlength = runtest('./maps/tower.txt', name  ,start, goal, planner_class, heuristic, verbose,log)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')

     
def test_flappy_bird(name = 'flappy_bird',planner_class = Planner.MyPlanner,  heuristic = 'manhattan',  verbose = True, log=None):
  print('Running flappy bird test...\n') 
  start = np.array([0.5, 4.5, 5.5])
  goal = np.array([19.5, 1.5, 1.5])
  success, pathlength = runtest('./maps/flappy_bird.txt',name ,start, goal, planner_class, heuristic, verbose,log)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength) 
  print('\n')

  
def test_room(name = 'room',planner_class = Planner.MyPlanner,  heuristic = 'manhattan',  verbose = True, log=None):
  print('Running room test...\n') 
  start = np.array([1.0, 5.0, 1.5])
  goal = np.array([9.0, 7.0, 1.5])
  success, pathlength = runtest('./maps/room.txt',name ,start, goal, planner_class, heuristic, verbose,log)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')

def test_pillars(name = 'pillars',planner_class = Planner.MyPlanner, heuristic = 'manhattan', verbose = True, log=None):
  print('Running pillars test...\n') 
  start = np.array([0.5, 0.5, 0.5])
  goal = np.array([19, 19, 9])
  success, pathlength = runtest('./maps/pillars.txt',name, start, goal, planner_class, heuristic, verbose,log)
  print('Success: %r'%success)
  print('Path length: %f'%pathlength)
  print('\n')


if __name__=="__main__":
  
  manhattan_flag = True
  gif_flag = False

  # Default planner : greedy
  
  log_file = os.path.join('log','Greedy.txt')

  test_single_cube(log = log_file)
  test_maze(log = log_file)
  test_flappy_bird(log = log_file)
  test_pillars(log = log_file)
  test_window(log = log_file)
  test_tower(log = log_file)
  test_room(log = log_file)

  # Weighted A* algorithmn
  
  log_file = os.path.join('log','Astar.txt')

  test_single_cube(planner_class = astar.AStar, heuristic = 'euclidean',log = log_file)
  test_maze(planner_class = astar.AStar,heuristic = 'euclidean',log = log_file)
  test_flappy_bird(planner_class = astar.AStar, heuristic = 'euclidean',log = log_file)
  test_pillars(planner_class = astar.AStar,heuristic = 'euclidean',log = log_file)
  test_window(planner_class = astar.AStar,heuristic = 'euclidean',log = log_file)
  test_tower(planner_class = astar.AStar,heuristic = 'euclidean',log = log_file)
  test_room(planner_class = astar.AStar,heuristic = 'euclidean',log = log_file)
  

  # RRT* algorithmn

  log_file = os.path.join('log','RRTstar.txt')

  test_single_cube(planner_class = rrt_star.RRTStar,log = log_file)
  test_maze(planner_class = rrt_star.RRTStar,log = log_file)
  test_flappy_bird(planner_class = rrt_star.RRTStar, log = log_file)
  test_pillars(planner_class = rrt_star.RRTStar,log = log_file)
  test_window(planner_class = rrt_star.RRTStar,log = log_file)
  test_tower(planner_class = rrt_star.RRTStar,log = log_file)
  test_room(planner_class = rrt_star.RRTStar,log = log_file)
  
  

  
  # Weighted A* algorithmn - heuristic, gif
  
  if manhattan_flag:
    log_file = os.path.join('log','Astar_manhattan.txt')

    test_single_cube(planner_class = astar.AStar,log = log_file)
    test_maze(planner_class = astar.AStar,log = log_file)
    test_flappy_bird(planner_class = astar.AStar, log = log_file)
    test_pillars(planner_class = astar.AStar,log = log_file)
    test_window(planner_class = astar.AStar,log = log_file)
    test_tower(planner_class = astar.AStar,log = log_file)
    test_room(planner_class = astar.AStar,log = log_file)

  if gif_flag:  
    log_file = os.path.join('log','Astar_gif.txt')
    test_single_cube(planner_class = astar.AStar,log = log_file)
    test_single_cube(planner_class = astar.AStar, heuristic = 'euclidean',log = log_file)
    test_room(planner_class = astar.AStar,heuristic = 'euclidean',log = log_file)
    test_room(planner_class = astar.AStar,log = log_file)
      
  

  plt.show(block=True)






