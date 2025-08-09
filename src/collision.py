from distance3d.distance import line_segment_to_box
import numpy as np

def detect_collision(path, aabb_blocks):
    collide = False

    box_center = []
    box_size = []

    for block in aabb_blocks:
        aabb_min = block[:3]    # minimum corner
        aabb_max = block[3:6]   # maximum corner

        # Convert AABB to box center and dimension
        pose = np.eye(4)
        pose[:-1,-1] = (aabb_min + aabb_max) / 2.0
        box_center.append(pose)
        box_size.append(aabb_max - aabb_min)

    for i in range(path.shape[0]-1): # Number of line segments we need to check
        for j in range(aabb_blocks.shape[0]): # number of obstacles
            distance,_,_ = line_segment_to_box(path[i], path[i+1], box_center[j], box_size[j])
            if distance == 0.0: # If the distance from the obstacle and path is 0, it is colliding.
                collide = True
                break
        if distance == 0.0:
            break

    return collide
